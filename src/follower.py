# OpenAMS inline follower (hardware driver for a firmware-controlled stepper)
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# A [follower <name>] is an inline stepper feeder on a spare stepper port whose
# motion is closed-loop controlled by CUSTOM KLIPPER MCU FIRMWARE (see
# FOLLOWER_PROTOCOL.md) — never by the motion queue. The host merely:
#   - forwards the FPS pressure value (the slow trim input),
#   - streams feed-forward velocity segments sampled AHEAD of real time from
#     the toolhead motion queue (the queue is known in advance; using its
#     knowledge does not mutate it),
#   - starts/stops ops and the follow loop, and mirrors telemetry.
# To the rest of the plugin this driver is a SINGLE-BAY AMS unit: it presents
# the same interface as the OAMS driver (bay 0 only), joins filament groups,
# and participates in runout auto-reload. Sensor mapping: the PRE-follower
# switch plays the f1s-HES role ("ready") and the POST-follower switch (before
# the FPS) plays the hub-HES role ("loaded").
#
# All wire values are integers (Klipper MCU firmware is float-free): distances
# in µsteps, velocities in steps/s, FPS as 0..65535, PID gains in Q12. This
# driver owns every mm<->steps conversion.

import logging
import math

import mcu

from . import oams_state as S
from .oams_state import (
    OAMS_OP_CODE_SUCCESS,
    OAMS_OP_CODE_ERROR_UNSPECIFIED,
    OAMS_OP_CODE_ERROR_BUSY,
    OAMS_OP_CODE_ERROR_KLIPPER_CALL,
    OAMS_OP_CODE_CANCEL,
    OAMS_OP_CODE_TIMEOUT,
    FOLLOWER_REVERSE,
    FOLLOWER_FORWARD,
)

# Follower firmware protocol (see FOLLOWER_PROTOCOL.md). Gen echo and
# firmware-owned liveness are mandatory from v1 — there is no legacy mode, so
# a missing FOLLOWER_PROTOCOL_VERSION is a configuration error (wrong/plain
# firmware on that MCU), not a fallback.
MIN_FOLLOWER_PROTOCOL = 1

# Action ids in follower_action_status (same values as the OAMS action enum).
# Fallbacks only; the dictionary-published values are adopted at connect.
FOLLOWER_STATUS_LOADING = 0
FOLLOWER_STATUS_UNLOADING = 1
FOLLOWER_STATUS_ERROR = 7

# Wire scale for the FPS value: host float 0..1 -> u16 0..65535.
FPS_WIRE_SCALE = 65535.0
# PID gains ride as Q12 fixed point: steps/s of trim per count of FPS error
# (per second for ki, seconds for kd).
PID_Q = 4096.0
# find_past_position() only knows about GENERATED steps. Klipper's
# background flusher generates steps just 0.45-0.7 s ahead of the clock
# (motion_queuing BGFLUSH_*), while toolhead print_time (the PLANNED
# horizon) runs up to ~1 s ahead — sampling beyond the generated window
# returns a flat position (zero velocity). Cap the ff window safely inside
# the generated horizon.
FF_GENERATED_HORIZON = 0.35
# Refresh the last ff velocity at least this often even when delta
# suppression would skip it: the firmware holds the last segment and
# declares underrun only after ~2 s of host silence.
FF_REFRESH_INTERVAL = 0.5

# (published firmware name, host reducer value): the op-code enum flows into
# the shared reducer, so it is VALIDATED, not substituted.
_VALIDATED_ENUM_NAMES = (
    ("FOLLOWER_OP_CODE_SUCCESS", OAMS_OP_CODE_SUCCESS),
    ("FOLLOWER_OP_CODE_ERROR_UNSPECIFIED", OAMS_OP_CODE_ERROR_UNSPECIFIED),
    ("FOLLOWER_OP_CODE_ERROR_KLIPPER_CALL", OAMS_OP_CODE_ERROR_KLIPPER_CALL),
    ("FOLLOWER_OP_CODE_CANCEL_LOAD_SPOOL", OAMS_OP_CODE_CANCEL),
    ("FOLLOWER_OP_CODE_TIMEOUT", OAMS_OP_CODE_TIMEOUT),
    ("FOLLOWER_REVERSE", FOLLOWER_REVERSE),
    ("FOLLOWER_FORWARD", FOLLOWER_FORWARD),
)

# follower_stats flags bits
FLAG_FOLLOWING = 1 << 0
FLAG_DIRECTION = 1 << 1
FLAG_OP_IN_FLIGHT = 1 << 2
FLAG_FPS_STALE = 1 << 3
FLAG_FF_UNDERRUN = 1 << 4
FLAG_ERROR_LATCHED = 1 << 5


class Follower:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()               # "follower <name>"
        self.short_name = self.name.split()[-1]
        self.reactor = self.printer.get_reactor()
        self.mcu = mcu.get_printer_mcu(self.printer, config.get("mcu", "mcu"))
        self.oid = self.mcu.create_oid()

        kind = config.get("type", "stepper")
        if kind != "stepper":
            raise config.error(
                "[%s] type '%s' is not supported yet; only 'stepper' is"
                " implemented ('bldc' is reserved)." % (self.name, kind))

        # Lane + unit identity ('oams_idx' attribute name kept: it is the
        # cross-kind unit index the manager/runtime/reducer route by).
        self.fps_name = config.get("fps", None)
        self.oams_idx = config.getint("unit_idx", minval=0)

        # ------------------------------------------------- stepper geometry
        self.step_pin = config.get("step_pin")
        self.dir_pin = config.get("dir_pin")
        self.enable_pin = config.get("enable_pin")
        rotation_distance = config.getfloat("rotation_distance", above=0.0)
        microsteps = config.getint("microsteps", 16, minval=1)
        full_steps = config.getint("full_steps_per_rotation", 200, minval=1)
        gear_ratio = 1.0
        for pair in config.get("gear_ratio", "").split(","):
            pair = pair.strip()
            if not pair:
                continue
            try:
                num, den = pair.split(":")
                gear_ratio *= float(num) / float(den)
            except (ValueError, ZeroDivisionError):
                raise config.error(
                    "[%s] invalid gear_ratio element '%s'; expected 'a:b'"
                    % (self.name, pair))
        self.steps_per_mm = full_steps * microsteps * gear_ratio \
            / rotation_distance

        # ---------------------------------------------------------- sensors
        self.pre_switch_pin = config.get("pre_switch_pin")
        self.post_switch_pin = config.get("post_switch_pin")
        self.switch_debounce = config.getfloat("switch_debounce", 0.005,
                                               above=0.0, maxval=0.25)

        # --------------------------------------------------------- geometry
        # POST-switch -> extruder gears, in mm. 0 = uncalibrated: the runout
        # logic then pauses instead of auto-reloading (same rule as an OAMS
        # with ptfe_length 0).
        self.path_length = config.getfloat("path_length", 0.0, minval=0.0)
        self.switch_travel = config.getfloat("switch_travel", 150.0, above=0.0)
        self.park_extra = config.getfloat("park_extra", 20.0, minval=0.0)

        # ------------------------------------------------- control tunables
        self.fps_lower_threshold = config.getfloat(
            "fps_lower_threshold", 0.3, minval=0.0, maxval=1.0)
        self.fps_upper_threshold = config.getfloat(
            "fps_upper_threshold", 0.7, minval=0.0, maxval=1.0,
            above=self.fps_lower_threshold)
        self.fps_target = config.getfloat(
            "fps_target", 0.5, above=self.fps_lower_threshold,
            below=self.fps_upper_threshold)
        self.fps_is_reversed = config.getboolean("fps_is_reversed", False)
        self.kp = config.getfloat("kp", 40.0, minval=0.0)
        self.ki = config.getfloat("ki", 2.0, minval=0.0)
        self.kd = config.getfloat("kd", 0.0, minval=0.0)
        self.max_speed = config.getfloat("max_speed", 300.0, above=0.0)
        self.accel = config.getfloat("accel", 1500.0, above=0.0)
        self.load_speed = config.getfloat("load_speed", 100.0, above=0.0)
        self.unload_speed = config.getfloat("unload_speed", 60.0, above=0.0)
        self.fps_stale_ms = config.getint("fps_stale_ms", 500, minval=50)
        # The firmware rejects (MCU shutdown) speeds beyond its step budget;
        # catch it here as a readable config error instead.
        if self._steps_s(self.max_speed) > 100000:
            raise config.error(
                "[%s] max_speed %.0f mm/s is %d steps/s, above the firmware's"
                " 100000 steps/s budget; lower max_speed or microsteps."
                % (self.name, self.max_speed, self._steps_s(self.max_speed)))
        if self.load_speed > self.max_speed \
                or self.unload_speed > self.max_speed:
            raise config.error(
                "[%s] load_speed/unload_speed must not exceed max_speed"
                % (self.name,))
        self.telemetry_ms = config.getint("telemetry_ms", 500, minval=100)

        # --------------------------------------------------- host loop rates
        self.fps_forward_interval = config.getfloat(
            "fps_forward_interval", 0.1, above=0.0)
        self.ff_interval = config.getfloat("ff_interval", 0.1, above=0.0)
        self.ff_sample_period = config.getfloat(
            "ff_sample_period", 0.05, above=0.0)
        self.ff_horizon = config.getfloat("ff_horizon", 1.5, above=0.0)

        # Optional embedded TMC UART bring-up (a plain [tmc2209] section can't
        # drive this stepper: it requires a registered stepper enable line,
        # which would take the pins away from the follower firmware).
        self._tmc = None
        if config.get("uart_pin", None) is not None:
            self._tmc = _TmcUartInit(config, microsteps)

        # ------------------------------------------------------ live mirrors
        # 4-lists so the manager's world builder treats this like any unit;
        # bays 1-3 are permanently absent (unreachable via topology).
        self.f1s_hes_value = [0, 0, 0, 0]     # [pre-switch, 0, 0, 0]
        self.hub_hes_value = [0, 0, 0, 0]     # [post-switch, 0, 0, 0]
        self.encoder_clicks = 0               # firmware step_count
        self.velocity = 0                     # steps/s, signed
        self.fps_stale = False
        self.ff_underrun = False
        self.error_latched = False
        self.current_spool = None             # 0 while loaded, else None

        self.protocol_version = None
        self.status_loading = FOLLOWER_STATUS_LOADING
        self.status_unloading = FOLLOWER_STATUS_UNLOADING
        self.status_error = FOLLOWER_STATUS_ERROR

        # Firmware command handles (resolved at klippy:connect).
        self.cmd_load = None
        self.cmd_unload = None
        self.cmd_load_cancel = None
        self.cmd_set = None
        self.cmd_fps = None
        self.cmd_ff = None
        self.cmd_clear = None

        # Host stream state. _local_change_time guards the telemetry
        # reconciliation: a stats report older than a just-made local change
        # must not immediately overwrite it.
        self._local_change_time = -10.0
        self._following = False
        self._direction = FOLLOWER_FORWARD
        self._op_in_flight = False
        self._ff_last_pt = None
        self._ff_last_v = None
        self._ff_last_send = 0.0
        self._ff_capable = False
        self._fps = None
        self._extruder = None
        self._toolhead = None
        self._fps_timer = None
        self._ff_timer = None

        # Store binding (set by the manager)
        self.runtime = None
        self.on_action_complete = None

        self.mcu.register_serial_response(
            self._action_status_received,
            "follower_action_status oid=%c action=%c code=%c value=%u gen=%c",
            self.oid)
        self.mcu.register_serial_response(
            self._stats_received,
            "follower_stats oid=%c pre=%c post=%c flags=%c step_count=%i"
            " velocity=%i", self.oid)
        self.mcu.register_config_callback(self._build_config)

        self.register_commands()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    # ------------------------------------------------------ unit conversion

    def _steps(self, millimetres):
        return max(0, int(round(millimetres * self.steps_per_mm)))

    def _steps_s(self, mm_per_s):
        return max(0, int(round(mm_per_s * self.steps_per_mm)))

    def _fps16(self, value):
        return max(0, min(65535, int(round(value * FPS_WIRE_SCALE))))

    def _gain_q12(self, gain_mm_s):
        # steps/s of trim per COUNT of fps16 error, Q12 fixed point.
        raw = int(round(gain_mm_s * self.steps_per_mm / FPS_WIRE_SCALE * PID_Q))
        if raw > 0xFFFF:
            logging.warning("follower[%s]: PID gain %.1f clamps to the Q12"
                            " wire maximum; the effective gain is weaker than"
                            " configured", self.short_name, gain_mm_s)
        return max(0, min(0xFFFF, raw))

    # ---------------------------------------------------------- mcu config

    def _build_config(self):
        ppins = self.printer.lookup_object("pins")

        def pin(desc, pullup_ok=False):
            params = ppins.lookup_pin(desc, can_invert=True,
                                      can_pullup=pullup_ok)
            if params.get("pullup", 0) < 0:
                raise self.printer.config_error(
                    "[%s]: pull-down ('~') is not supported on follower"
                    " switch pin '%s'; use '^' or a plain pin."
                    % (self.name, desc))
            if params["chip"] is not self.mcu:
                raise self.printer.config_error(
                    "[%s]: pin '%s' is not on mcu '%s'; every follower pin"
                    " must live on the follower's MCU."
                    % (self.name, desc, self.mcu.get_name() or "mcu"))
            return params

        step = pin(self.step_pin)
        dirp = pin(self.dir_pin)
        en = pin(self.enable_pin)
        pre = pin(self.pre_switch_pin, pullup_ok=True)
        post = pin(self.post_switch_pin, pullup_ok=True)
        flags = ((1 if step["invert"] else 0)
                 | (2 if dirp["invert"] else 0)
                 | (4 if en["invert"] else 0))
        self.mcu.add_config_cmd(
            "config_follower oid=%d step_pin=%s dir_pin=%s enable_pin=%s"
            " flags=%d" % (self.oid, step["pin"], dirp["pin"], en["pin"],
                           flags))
        self.mcu.add_config_cmd(
            "config_follower_switches oid=%d pre_pin=%s pre_pullup=%d"
            " pre_invert=%d post_pin=%s post_pullup=%d post_invert=%d"
            " debounce_ms=%d"
            % (self.oid, pre["pin"], pre.get("pullup", 0),
               1 if pre["invert"] else 0, post["pin"], post.get("pullup", 0),
               1 if post["invert"] else 0,
               int(self.switch_debounce * 1000.0)))
        self.mcu.add_config_cmd(
            "config_follower_tuning oid=%d kp=%d ki=%d kd=%d fps_target=%d"
            " fps_lower=%d fps_upper=%d fps_reversed=%d"
            % (self.oid, self._gain_q12(self.kp), self._gain_q12(self.ki),
               self._gain_q12(self.kd), self._fps16(self.fps_target),
               self._fps16(self.fps_lower_threshold),
               self._fps16(self.fps_upper_threshold),
               1 if self.fps_is_reversed else 0))
        self.mcu.add_config_cmd(
            "config_follower_limits oid=%d max_v=%d accel=%d load_v=%d"
            " unload_v=%d"
            % (self.oid, self._steps_s(self.max_speed),
               self._steps_s(self.accel), self._steps_s(self.load_speed),
               self._steps_s(self.unload_speed)))
        self.mcu.add_config_cmd(
            "config_follower_geometry oid=%d path_steps=%d"
            " switch_travel_steps=%d park_extra_steps=%d"
            % (self.oid, self._steps(self.path_length),
               self._steps(self.switch_travel), self._steps(self.park_extra)))
        self.mcu.add_config_cmd(
            "config_follower_watchdog oid=%d fps_stale_ms=%d telemetry_ms=%d"
            % (self.oid, self.fps_stale_ms, self.telemetry_ms))

    # -------------------------------------------------------------- connect

    def handle_connect(self):
        self._resolve_protocol()
        self.cmd_load = self.mcu.lookup_command("follower_cmd_load oid=%c gen=%c")
        self.cmd_unload = self.mcu.lookup_command(
            "follower_cmd_unload oid=%c gen=%c")
        self.cmd_load_cancel = self.mcu.lookup_command(
            "follower_cmd_load_cancel oid=%c")
        self.cmd_set = self.mcu.lookup_command(
            "follower_cmd_set oid=%c enable=%c direction=%c")
        self.cmd_fps = self.mcu.lookup_command(
            "follower_cmd_fps oid=%c value=%u")
        self.cmd_ff = self.mcu.lookup_command(
            "follower_cmd_ff oid=%c clock=%u velocity=%i")
        self.cmd_clear = self.mcu.lookup_command(
            "follower_cmd_clear_errors oid=%c")
        if self._tmc is not None:
            self._tmc.init_registers()

    def _resolve_protocol(self):
        try:
            consts = self.mcu.get_constants() or {}
        except Exception:
            consts = {}

        def int_const(name, default):
            value = consts.get(name, default)
            try:
                return int(value)
            except (TypeError, ValueError):
                logging.warning("follower[%s]: ignoring non-integer firmware"
                                " constant %s=%r", self.short_name, name, value)
                return default

        if "FOLLOWER_PROTOCOL_VERSION" not in consts:
            raise self.printer.config_error(
                "[%s]: MCU '%s' publishes no FOLLOWER_PROTOCOL_VERSION — its"
                " firmware has no follower support. Flash the follower-enabled"
                " Klipper firmware on that board (see FOLLOWER_PROTOCOL.md)."
                % (self.name, self.mcu.get_name() or "mcu"))
        self.protocol_version = int_const("FOLLOWER_PROTOCOL_VERSION", 0)
        if self.protocol_version < MIN_FOLLOWER_PROTOCOL:
            raise self.printer.config_error(
                "[%s]: follower protocol version %s is older than the minimum"
                " this plugin supports (%s)."
                % (self.name, self.protocol_version, MIN_FOLLOWER_PROTOCOL))
        logging.info("follower[%s]: firmware protocol version %s",
                     self.short_name, self.protocol_version)

        self.status_loading = int_const("FOLLOWER_STATUS_LOADING",
                                        FOLLOWER_STATUS_LOADING)
        self.status_unloading = int_const("FOLLOWER_STATUS_UNLOADING",
                                          FOLLOWER_STATUS_UNLOADING)
        self.status_error = int_const("FOLLOWER_STATUS_ERROR",
                                      FOLLOWER_STATUS_ERROR)
        mismatches = ["%s: firmware=%s host=%s" % (name, consts[name], default)
                      for name, default in _VALIDATED_ENUM_NAMES
                      if name in consts and int_const(name, default) != default]
        if mismatches:
            logging.error(
                "follower[%s]: firmware protocol enum mismatch (%s); result"
                " code handling may be wrong — update plugin or firmware",
                self.short_name, "; ".join(mismatches))

    # ---------------------------------------------------------------- ready

    def handle_ready(self):
        # Resolve the lane objects the host streams depend on. fps_name is
        # finalized by the manager during topology build (before ready).
        try:
            self._fps = self.printer.lookup_object(
                "fps" if self.fps_name in (None, "fps")
                else "fps %s" % self.fps_name)
        except Exception:
            # Manager-less configs are unsupported; without an FPS the
            # follower cannot run — make it loud but don't kill ready.
            logging.exception("follower[%s]: FPS lane '%s' not found; the"
                              " follower will not stream FPS/feed-forward",
                              self.short_name, self.fps_name)
            return
        self._extruder = getattr(self._fps, "extruder", None)
        self._toolhead = self.printer.lookup_object("toolhead")
        self._ff_capable = (self._extruder is not None and
                            getattr(self._extruder, "find_past_position", None)
                            is not None)
        if not self._ff_capable:
            logging.warning("follower[%s]: extruder lacks find_past_position;"
                            " running FPS-trim-only (no feed-forward)",
                            self.short_name)
        self._fps_timer = self.reactor.register_timer(
            self._fps_forward_event, self.reactor.NOW)
        self._ff_timer = self.reactor.register_timer(
            self._ff_stream_event, self.reactor.NOW)

    # ------------------------------------------------------- runtime binding

    def bind_runtime(self, runtime, fps_name):
        """Called by the manager so firmware replies feed the store."""
        self.runtime = runtime
        self.fps_name = fps_name
        self.on_action_complete = (
            lambda code, value, gen: runtime.dispatch(
                S.OpCompleted(fps_name, code, value, gen=gen)))

    # ------------------------------------------------------------ properties

    @property
    def firmware_owns_liveness(self):
        # Mandatory from protocol v1: the firmware runs the no-progress and
        # FPS-staleness watchdogs and always completes an op.
        return (self.protocol_version is not None
                and self.protocol_version >= MIN_FOLLOWER_PROTOCOL)

    @property
    def connected(self):
        return self.cmd_load is not None

    @property
    def path_length_mm(self):
        return self.path_length

    @property
    def filament_path_length(self):
        # Interface parity (diagnostics only); canonical value is path_length.
        return self.path_length

    def protocol_summary(self):
        return "protocol=FOLLOWER v%s%s" % (
            self.protocol_version if self.protocol_version is not None
            else "?", ", ff" if self._ff_capable else ", no-ff")

    def is_bay_ready(self, bay_index):
        return bool(self.f1s_hes_value[bay_index])

    def is_bay_loaded(self, bay_index):
        return bool(self.hub_hes_value[bay_index])

    def get_status(self, eventtime):
        return {
            "current_spool": self.current_spool,
            "pre_switch": bool(self.f1s_hes_value[0]),
            "post_switch": bool(self.hub_hes_value[0]),
            "following": self._following,
            "velocity": self.velocity,
            "step_count": self.encoder_clicks,
            "fps_stale": self.fps_stale,
            "ff_underrun": self.ff_underrun,
            "op_in_flight": self._op_in_flight,
            "error_latched": self.error_latched,
            "protocol_version": self.protocol_version,
        }

    def get_webhook_status(self):
        return self.get_status(None)

    def stats(self, eventtime):
        return (False,
                "follower[%s]: pre=%d post=%d following=%d velocity=%d"
                " step_count=%d fps_stale=%d ff_underrun=%d"
                % (self.short_name, self.f1s_hes_value[0],
                   self.hub_hes_value[0], self._following, self.velocity,
                   self.encoder_clicks, self.fps_stale, self.ff_underrun))

    # ------------------------------------------------------------ op senders

    def _require_cmd(self, cmd, what):
        if cmd is None:
            raise self.printer.command_error(
                "follower[%s]: %s unavailable (MCU not connected or firmware"
                " initialization failed; see klippy.log)"
                % (self.short_name, what))
        return cmd

    @staticmethod
    def _wire_gen(gen):
        return 0 if gen is None else (gen & 0xFF)

    def start_load_spool(self, spool_idx, gen=None):
        if spool_idx != 0:
            raise self.printer.command_error(
                "follower[%s] has a single bay (0); bay %d does not exist"
                % (self.short_name, spool_idx))
        if self.path_length <= 0.0:
            # Firmware phase-2 budget would be 0 steps: the load would feed to
            # the POST switch and then TIMEOUT immediately with filament stuck
            # between the switches. Refuse with an actionable message instead.
            raise self.printer.command_error(
                "follower[%s]: path_length is not configured; measure the"
                " POST-switch to extruder distance and set path_length"
                % (self.short_name,))
        cmd = self._require_cmd(self.cmd_load, "load command")
        cmd.send([self.oid, self._wire_gen(gen)])
        self._op_in_flight = True
        self._local_change_time = self.reactor.monotonic()

    def start_unload_spool(self, gen=None):
        cmd = self._require_cmd(self.cmd_unload, "unload command")
        cmd.send([self.oid, self._wire_gen(gen)])
        self._op_in_flight = True
        self._local_change_time = self.reactor.monotonic()

    def start_calibrate(self, kind, bay, gen=None):
        # No calibrate op in follower protocol v1 (path_length is configured
        # manually). Fail the op cleanly instead of wedging the lane.
        logging.warning("follower[%s]: calibrate '%s' is not supported",
                        self.short_name, kind)

        def fail(eventtime):
            if self.on_action_complete is not None:
                self.on_action_complete(OAMS_OP_CODE_ERROR_UNSPECIFIED, None,
                                        self._wire_gen(gen))
        self.reactor.register_async_callback(fail)

    def load_spool_cancel(self):
        if self.cmd_load_cancel is None:
            return "follower load cancel unavailable"
        self.cmd_load_cancel.send([self.oid])
        return "follower load cancel requested"

    def set_oams_follower(self, enable, direction):
        # Name kept for effect routing parity with the OAMS driver: this IS
        # the "enable the follower loop" call.
        if self.cmd_set is None:
            return
        if not enable and self._op_in_flight:
            # Firmware cmd_set enable=0 hard-aborts an in-flight op. Broadcast
            # follower stops (e.g. _stop_followers on a no-op unload path)
            # must not cancel a load/unload/auto-reload: explicit cancellation
            # goes through load_spool_cancel/clear_errors.
            logging.info("follower[%s]: ignoring follower stop while an op is"
                         " in flight", self.short_name)
            return
        self.cmd_set.send([self.oid, 1 if enable else 0, direction])
        self._following = bool(enable)
        self._direction = direction
        self._local_change_time = self.reactor.monotonic()
        if enable:
            # prime the loop promptly rather than waiting a full interval
            self._send_fps_now()

    def set_led_error(self, idx, value):
        logging.info("follower[%s]: (no LED) error indication %s=%s",
                     self.short_name, idx, value)

    def clear_errors(self):
        if self.cmd_clear is not None:
            self.cmd_clear.send([self.oid])
        self.error_latched = False
        # Firmware cmd_clear_errors hard-stops the follow loop and any op.
        self._following = False
        self._op_in_flight = False
        self._local_change_time = self.reactor.monotonic()
        self.current_spool = 0 if self.hub_hes_value[0] else None

    # ------------------------------------------------------------ host loops

    def _streams_active(self):
        return self._following or self._op_in_flight

    def _send_fps_now(self):
        if self.cmd_fps is None or self._fps is None:
            return
        self.cmd_fps.send([self.oid, self._fps16(self._fps.get_value())])

    def _fps_forward_event(self, eventtime):
        # The FPS value is the loop's slow trim; firmware arms its staleness
        # watchdog only while following/op-active, so an idle follower is
        # never tripped by us not sending.
        if self._streams_active():
            try:
                self._send_fps_now()
            except Exception:
                logging.exception("follower[%s]: FPS forward failed",
                                  self.short_name)
        return eventtime + self.fps_forward_interval

    def _ff_stream_event(self, eventtime):
        if self._following and self._ff_capable \
                and self._direction == FOLLOWER_FORWARD:
            try:
                self._stream_feed_forward(eventtime)
            except Exception:
                logging.exception("follower[%s]: feed-forward stream failed",
                                  self.short_name)
        else:
            self._ff_last_pt = None      # resync the window on re-enable
            self._ff_last_v = None
        return eventtime + self.ff_interval

    def _stream_feed_forward(self, eventtime):
        """Sample commanded extruder velocity over the already-generated
        (flushed) window ahead of real time and stream it as (clock, steps/s)
        segments. find_past_position() resolves from the generated step
        history, so print times inside the flushed horizon are valid — the
        same mechanism filament-width sensors use, queried ahead of the clock
        instead of behind it. Never query beyond the flushed horizon."""
        if self.cmd_ff is None:
            return
        # toolhead print_time is the PLANNED horizon; steps are only
        # GENERATED ~0.45-0.7 s ahead (see FF_GENERATED_HORIZON). Sampling
        # past the generated window would read flat positions (v=0), so the
        # window is bounded by BOTH.
        flushed_pt = self._toolhead.get_status(eventtime)["print_time"]
        now_pt = self.mcu.estimated_print_time(eventtime)
        horizon = min(self.ff_horizon, FF_GENERATED_HORIZON)
        end = min(flushed_pt, now_pt + horizon)
        start = self._ff_last_pt
        if start is None or start < now_pt:
            start = now_pt
        if end - start < self.ff_sample_period:
            # Idle/paused window: keepalive so firmware doesn't latch a
            # spurious ff underrun.
            if eventtime - self._ff_last_send >= 1.0:
                self._send_ff(start, 0)
                self._ff_last_send = eventtime
            return
        max_v = self._steps_s(self.max_speed)
        pos = self._extruder.find_past_position(start)
        pt = start
        while pt + self.ff_sample_period <= end:
            nxt = pt + self.ff_sample_period
            npos = self._extruder.find_past_position(nxt)
            v_mm_s = (npos - pos) / self.ff_sample_period
            v = int(round(v_mm_s * self.steps_per_mm))
            v = max(-max_v, min(max_v, v))
            last = self._ff_last_v
            # Delta suppression: only segments that meaningfully change the
            # commanded velocity ride the wire — but refresh at least every
            # FF_REFRESH_INTERVAL so the firmware (which holds the last
            # velocity and treats ~2 s of silence as underrun) keeps seeing a
            # live stream during long constant-velocity cruises.
            if (last is None or abs(v - last) > max(2, abs(last) // 50)
                    or eventtime - self._ff_last_send >= FF_REFRESH_INTERVAL):
                self._send_ff(pt, v)
                self._ff_last_send = eventtime
            pos, pt = npos, nxt
        self._ff_last_pt = pt

    def _send_ff(self, print_time, v_steps_s):
        clock = self.mcu.print_time_to_clock(print_time) & 0xFFFFFFFF
        self.cmd_ff.send([self.oid, clock, v_steps_s])
        self._ff_last_v = v_steps_s

    # ------------------------------------------------------------- gcode

    def register_commands(self):
        follower_id = str(self.oams_idx)
        gcode = self.printer.lookup_object("gcode")
        for name, handler, desc in (
            ("FOLLOWER_LOAD_SPOOL", self.cmd_FOLLOWER_LOAD_SPOOL,
             "Load filament through this follower"),
            ("FOLLOWER_UNLOAD_SPOOL", self.cmd_FOLLOWER_UNLOAD_SPOOL,
             "Unload filament from this follower"),
            ("FOLLOWER_SET", self.cmd_FOLLOWER_SET,
             "Enable/disable this follower's firmware follow loop"),
        ):
            gcode.register_mux_command(name, "FOLLOWER", follower_id, handler,
                                       desc=desc)

    def cmd_FOLLOWER_LOAD_SPOOL(self, gcmd):
        if self.runtime is None:
            raise gcmd.error("[oams_manager] is required")
        result = self.runtime.request(
            self.fps_name, S.LoadBay(self.fps_name, (self.oams_idx, 0))).wait()
        if result.ok or result.code == OAMS_OP_CODE_CANCEL:
            gcmd.respond_info(result.message)
        else:
            raise gcmd.error(result.message)

    def cmd_FOLLOWER_UNLOAD_SPOOL(self, gcmd):
        if self.runtime is None:
            raise gcmd.error("[oams_manager] is required")
        lane = self.runtime.get_state().lanes.get(self.fps_name)
        unit = lane.unit if lane is not None else None
        if unit is None or unit[0] != self.oams_idx:
            self.set_oams_follower(0, FOLLOWER_REVERSE)
            gcmd.respond_info("Nothing loaded through this follower"
                              " (follow loop stopped)")
            return
        result = self.runtime.request(self.fps_name,
                                      S.Unload(self.fps_name)).wait()
        if not result.ok:
            raise gcmd.error(result.message)
        gcmd.respond_info(result.message)

    def cmd_FOLLOWER_SET(self, gcmd):
        enable = gcmd.get_int("ENABLE", minval=0, maxval=1)
        direction = gcmd.get_int("DIRECTION", FOLLOWER_FORWARD,
                                 minval=0, maxval=1)
        lane = None
        if self.runtime is not None:
            lane = self.runtime.get_state().lanes.get(self.fps_name)
        if lane is not None and lane.unit is not None \
                and lane.unit[0] == self.oams_idx:
            # Through the store so LaneState.following stays truthful.
            self.runtime.dispatch(S.Follow(self.fps_name, enable, direction))
        else:
            self.set_oams_follower(enable, direction)
        gcmd.respond_info("Follower %s" % ("enabled" if enable else "stopped"))

    # ------------------------------------------------------ firmware replies

    def _action_status_received(self, params):
        # Serial reader thread -> marshal onto the reactor thread.
        self.reactor.register_async_callback(
            lambda et, params=params: self._apply_action_status(params))

    def _apply_action_status(self, params):
        action = params["action"]
        code = params["code"]
        if action not in (self.status_loading, self.status_unloading,
                          self.status_error):
            # Follow-state notifications are non-terminal; never completions.
            logging.info("follower[%s]: ignoring non-completion status"
                         " action=%d code=%d", self.short_name, action, code)
            return
        if code != OAMS_OP_CODE_ERROR_BUSY:
            # BUSY is the rejection of a NEW op while another is still in
            # flight — the running op (and its FPS stream!) must live on.
            self._op_in_flight = False
        self._local_change_time = self.reactor.monotonic()
        if action == self.status_loading and code == OAMS_OP_CODE_SUCCESS:
            self.current_spool = 0
            # Firmware auto-starts forward following after a load SUCCESS
            # (protocol guarantee, OAMS parity) — mirror that.
            self._following = True
            self._direction = FOLLOWER_FORWARD
        elif action == self.status_unloading and code == OAMS_OP_CODE_SUCCESS:
            self.current_spool = None
            self._following = False
        if self.on_action_complete is not None:
            self.on_action_complete(code, params["value"], params["gen"])

    def _stats_received(self, params):
        # Serial reader thread: plain int/replacement writes only (same
        # atomic-publish discipline as the OAMS driver).
        flags = params["flags"]
        self.f1s_hes_value = [1 if params["pre"] else 0, 0, 0, 0]
        self.hub_hes_value = [1 if params["post"] else 0, 0, 0, 0]
        self.encoder_clicks = params["step_count"]
        self.velocity = params["velocity"]
        self.fps_stale = bool(flags & FLAG_FPS_STALE)
        self.ff_underrun = bool(flags & FLAG_FF_UNDERRUN)
        self.error_latched = bool(flags & FLAG_ERROR_LATCHED)
        # The firmware also reports its OWN following/op/direction state:
        # adopt it (on the reactor thread) so mirror drift self-heals — e.g.
        # a lost terminal status leaving _op_in_flight stuck, or an enable
        # the firmware ignored because an op was running.
        following = bool(flags & FLAG_FOLLOWING)
        op_in_flight = bool(flags & FLAG_OP_IN_FLIGHT)
        direction = FOLLOWER_FORWARD if flags & FLAG_DIRECTION \
            else FOLLOWER_REVERSE
        self.reactor.register_async_callback(
            lambda et, a=following, b=op_in_flight, c=direction:
                self._reconcile_flags(et, a, b, c))

    def _reconcile_flags(self, eventtime, following, op_in_flight, direction):
        # Telemetry is the firmware truth, but a report generated BEFORE a
        # just-made local change must not immediately overwrite it; the next
        # report (<= telemetry_ms later) reconciles for real.
        if eventtime - self._local_change_time < 1.0:
            return
        self._following = following
        self._op_in_flight = op_in_flight
        self._direction = direction


class _TmcUartInit:
    """Minimal embedded TMC2209 bring-up over single-wire UART, reusing
    mainline's MCU_TMC_uart transport. Only run/hold current and microsteps
    are programmed (GCONF/CHOPCONF/IHOLD_IRUN); everything else keeps the
    chip's reset defaults. A full-featured alternative is running the driver
    in standalone mode (no uart_pin) with straps."""

    # IFCNT is required: MCU_TMC_uart.set_register() reads it back to
    # verify every write.
    _REGS = {"GCONF": 0x00, "IFCNT": 0x02, "IHOLD_IRUN": 0x10,
             "CHOPCONF": 0x6C}
    _CHOPCONF_RESET = 0x10000053         # TMC2209 datasheet reset value

    class _FieldsStub:
        def get_reg_fields(self, reg_name, value):
            return {}

    def __init__(self, config, microsteps):
        from . import tmc_uart
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.run_current = config.getfloat("run_current", above=0.0)
        self.hold_current = config.getfloat("hold_current", self.run_current,
                                            above=0.0)
        self.sense_resistor = config.getfloat("sense_resistor", 0.110,
                                              above=0.0)
        self.interpolate = config.getboolean("interpolate", True)
        if microsteps not in (1, 2, 4, 8, 16, 32, 64, 128, 256):
            raise config.error("[%s] microsteps must be a power of two <= 256"
                               % self.name)
        self.mres = 8 - (microsteps.bit_length() - 1)   # 256->0 ... 1->8
        self.mcu_tmc = tmc_uart.MCU_TMC_uart(config, self._REGS,
                                             self._FieldsStub(), 3, 12000000)

    def _current_bits(self, current, vsense):
        # Same formula as mainline TMCCurrentHelper (tmc2130.py).
        sense = self.sense_resistor + 0.020
        vref = 0.18 if vsense else 0.32
        cs = int(32.0 * sense * current * math.sqrt(2.0) / vref + 0.5) - 1
        return max(0, min(31, cs))

    def init_registers(self):
        try:
            # pdn_disable + mstep_reg_select: configured over UART, not straps.
            self.mcu_tmc.set_register("GCONF", 0x000000C0)
            chopconf = self._CHOPCONF_RESET
            chopconf &= ~((0xF << 24) | (1 << 28) | (1 << 17))
            chopconf |= (self.mres & 0xF) << 24
            if self.interpolate:
                chopconf |= 1 << 28
            chopconf |= 1 << 17                          # vsense=1 (low power)
            self.mcu_tmc.set_register("CHOPCONF", chopconf)
            irun = self._current_bits(self.run_current, True)
            ihold = self._current_bits(min(self.hold_current,
                                           self.run_current), True)
            self.mcu_tmc.set_register(
                "IHOLD_IRUN", (8 << 16) | (irun << 8) | ihold)
        except Exception as e:
            raise self.printer.config_error(
                "[%s]: TMC UART initialization failed: %s (check uart_pin/"
                "uart_address wiring, or remove uart_pin to run the driver"
                " in standalone mode)" % (self.name, e))


def load_config_prefix(config):
    return Follower(config)
