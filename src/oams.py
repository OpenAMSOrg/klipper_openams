# OpenAMS Mainboard (hardware driver)
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# OAMS is a thin hardware driver for one OpenAMS mainboard: it sends firmware
# commands, mirrors telemetry, and reports operation completion. All system-level
# orchestration (what is loaded, runout, follower policy) lives in the store
# (oams_state.py) driven by the runtime (oams_runtime.py). When bound to a
# runtime, every firmware action-status reply is turned into an OpCompleted
# action; otherwise the blocking helpers below provide a standalone fallback for
# the low-level OAMS_* commands.

import logging
import struct
from collections import deque
from math import pi

import mcu

from . import oams_state as S
from .oams_state import (
    OAMS_OP_CODE_SUCCESS,
    OAMS_OP_CODE_ERROR_UNSPECIFIED,
    OAMS_OP_CODE_ERROR_KLIPPER_CALL,
    OAMS_OP_CODE_CANCEL,
    OAMS_OP_CODE_TIMEOUT,
    FOLLOWER_REVERSE,
    FOLLOWER_FORWARD,
    POLL_INTERVAL,
    OAMS_ACTION_TIMEOUT,
)

# Firmware "action" identifiers reported by oams_action_status. These are the
# BUILT-IN FALLBACK values: when the firmware publishes the enum in the data
# dictionary (OAMS_PROTOCOL_VERSION >= 1) the resolved values are read at
# connect (see _resolve_protocol); on older firmware these defaults are used.
OAMS_STATUS_LOADING = 0
OAMS_STATUS_UNLOADING = 1
OAMS_STATUS_FORWARD_FOLLOWING = 2
OAMS_STATUS_REVERSE_FOLLOWING = 3
OAMS_STATUS_COASTING = 4
OAMS_STATUS_STOPPED = 5
OAMS_STATUS_CALIBRATING = 6
OAMS_STATUS_ERROR = 7

# Lowest firmware protocol version this plugin understands. The host also runs
# against firmware that publishes no version at all (legacy mode), so this is
# only a forward-compatibility floor, not a hard requirement.
MIN_PROTOCOL_VERSION = 1

# From this protocol version the firmware owns per-op liveness: it runs its own
# no-progress watchdog, stops the motors on a stall, and completes the op with
# code OAMS_OP_CODE_TIMEOUT. The host then drops its authoritative op deadline
# and its own stall detection (see Runtime.set_firmware_liveness).
LIVENESS_PROTOCOL_VERSION = 3

# Driver-local action-enum names resolved from the dictionary, paired with the
# module fallback. Only the action enum is resolved dynamically: it is consumed
# solely here in the driver. The op-code and follower-direction enums are
# shared with the pure reducer (which cannot read the dictionary), so those are
# VALIDATED against the published values rather than substituted.
_ACTION_ENUM_NAMES = (
    ("OAMS_STATUS_LOADING", OAMS_STATUS_LOADING),
    ("OAMS_STATUS_UNLOADING", OAMS_STATUS_UNLOADING),
    ("OAMS_STATUS_CALIBRATING", OAMS_STATUS_CALIBRATING),
    ("OAMS_STATUS_ERROR", OAMS_STATUS_ERROR),
)

# (published firmware name, host module value) for the enums that flow into the
# pure reducer and must therefore match exactly. The firmware renamed CANCEL ->
# CANCEL_LOAD_SPOOL but kept the value (6).
_VALIDATED_ENUM_NAMES = (
    ("OAMS_OP_CODE_SUCCESS", OAMS_OP_CODE_SUCCESS),
    ("OAMS_OP_CODE_ERROR_UNSPECIFIED", OAMS_OP_CODE_ERROR_UNSPECIFIED),
    ("OAMS_OP_CODE_ERROR_KLIPPER_CALL", OAMS_OP_CODE_ERROR_KLIPPER_CALL),
    ("OAMS_OP_CODE_CANCEL_LOAD_SPOOL", OAMS_OP_CODE_CANCEL),
    ("OAMS_OP_CODE_TIMEOUT", OAMS_OP_CODE_TIMEOUT),
    ("OAMS_FOLLOWER_REVERSE", FOLLOWER_REVERSE),
    ("OAMS_FOLLOWER_FORWARD", FOLLOWER_FORWARD),
)


class OAMS:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()              # full section, e.g. "oams oams1"
        self.reactor = self.printer.get_reactor()
        self.mcu = mcu.get_printer_mcu(self.printer, config.get("mcu", "mcu"))

        # FPS lane this unit belongs to (short name). Optional: the manager
        # defaults it to the sole lane when only one FPS exists.
        self.fps_name = config.get("fps", None)

        self.fps_upper_threshold = config.getfloat("fps_upper_threshold")
        self.fps_lower_threshold = config.getfloat("fps_lower_threshold")
        self.fps_is_reversed = config.getboolean("fps_is_reversed")

        self.f1s_hes_on = [float(x.strip())
                           for x in config.get("f1s_hes_on").split(",")]
        self.f1s_hes_is_above = config.getboolean("f1s_hes_is_above")
        self.hub_hes_on = [float(x.strip())
                           for x in config.get("hub_hes_on").split(",")]
        self.hub_hes_is_above = config.getboolean("hub_hes_is_above")
        # 0 is allowed so a fresh machine can boot and run
        # OAMS_CALIBRATE_PTFE_LENGTH; the runout logic guards an uncalibrated
        # (<= 0) length rather than rejecting it here.
        self.filament_path_length = config.getfloat("ptfe_length")
        self.oams_idx = config.getint("oams_idx")

        self.kd = config.getfloat("kd", 0.0)
        self.ki = config.getfloat("ki", 0.0)
        self.kp = config.getfloat("kp", 6.0)
        self.current_kp = config.getfloat("current_kp", 0.375)
        self.current_ki = config.getfloat("current_ki", 0.0)
        self.current_kd = config.getfloat("current_kd", 0.0)
        self.fps_target = config.getfloat(
            "fps_target", 0.5, minval=0.0, maxval=1.0,
            above=self.fps_lower_threshold, below=self.fps_upper_threshold)
        self.current_target = config.getfloat(
            "current_target", 0.3, minval=0.1, maxval=0.4)

        # Index of the spool currently loaded through the hub (0..3), or None.
        # This is the driver's own firmware mirror, kept in step with the
        # commands this unit issues (see _apply_action_status).
        self.current_spool = None
        self._pending_bay = None

        # Protocol contract resolved at connect (see _resolve_protocol).
        self.protocol_version = None
        # Action-enum values used to interpret oams_action_status; start at the
        # module fallbacks, overwritten from the dictionary when published.
        self.status_loading = OAMS_STATUS_LOADING
        self.status_unloading = OAMS_STATUS_UNLOADING
        self.status_calibrating = OAMS_STATUS_CALIBRATING
        self.status_error = OAMS_STATUS_ERROR

        # Generation-matched protocol (oams_cmd_*2 / oams_action_status2). When
        # the firmware advertises the *2 commands, the gen rides on the wire and
        # the firmware echoes it, so completions are matched authoritatively.
        self._use_gen_protocol = False
        # Legacy fallback: FIFO of op_gen values for ops whose completion has
        # not arrived yet. The firmware answers ops in order and (pre-*2) does
        # not echo a correlation id, so pairing replies with the OLDEST pending
        # gen keeps a late reply from a timed-out op off its retry. Unused once
        # the *2 protocol is active.
        self._gen_queue = deque()

        # Firmware command handles (resolved at klippy:connect; None until then).
        self.oams_load_spool_cmd = None
        self.oams_unload_spool_cmd = None
        self.oams_load_spool_cancel_cmd = None
        self.oams_follower_cmd = None
        self.oams_calibrate_ptfe_length_cmd = None
        self.oams_calibrate_hub_hes_cmd = None
        self.oams_pid_cmd = None
        self.oams_set_led_error_cmd = None
        self.oams_spool_query_spool_cmd = None
        # Generation-matched (*2) command handles; None unless the firmware
        # advertises them (set in handle_connect).
        self.oams_load_spool2_cmd = None
        self.oams_unload_spool2_cmd = None
        self.oams_calibrate_ptfe_length2_cmd = None
        self.oams_calibrate_hub_hes2_cmd = None

        # Live telemetry.
        self.fps_value = 0.0
        self.i_value = 0.0
        self.encoder_clicks = 0
        self.f1s_hes_value = [0, 0, 0, 0]
        self.hub_hes_value = [0, 0, 0, 0]

        # Completion signalling. Only written on the reactor thread: the serial
        # reader thread marshals every action-status message onto the reactor via
        # register_async_callback (see _action_status_received), and the sentinel
        # action_status is published last so a waiter sees a settled code.
        self.action_status = None
        self.action_status_code = None
        self.action_status_value = None

        # Set by the manager via bind_runtime(); turns firmware replies into store
        # actions. None when running the low-level commands standalone.
        self.runtime = None
        self.on_action_complete = None

        self.mcu.register_serial_response(
            self._action_status_received,
            "oams_action_status action=%c code=%c value=%u")
        self.mcu.register_serial_response(
            self._oams_cmd_stats,
            "oams_cmd_stats fps_value=%u hub_hes_value_0=%c hub_hes_value_1=%c"
            " hub_hes_value_2=%c hub_hes_value_3=%c f1s_hes_value_0=%c"
            " f1s_hes_value_1=%c f1s_hes_value_2=%c f1s_hes_value_3=%c"
            " encoder_clicks=%u")
        self.mcu.register_serial_response(
            self._oams_cmd_current_stats,
            "oams_cmd_current_status current_value=%u")
        self.mcu.register_config_callback(self._build_config)

        self.register_commands()
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    # ------------------------------------------------------- runtime binding

    def bind_runtime(self, runtime, fps_name):
        """Called by the manager so firmware replies feed the store."""
        self.runtime = runtime
        self.fps_name = fps_name
        self.on_action_complete = (
            lambda code, value, gen: runtime.dispatch(
                S.OpCompleted(fps_name, code, value, gen=gen)))

    # ------------------------------------------------------------------ status

    def get_status(self, eventtime):
        return {"current_spool": self.current_spool}

    @property
    def firmware_owns_liveness(self):
        """True when this unit's firmware runs its own per-op no-progress
        watchdog (protocol >= 3), so the host need not impose its own deadline."""
        return (self.protocol_version is not None
                and self.protocol_version >= LIVENESS_PROTOCOL_VERSION)

    def is_bay_ready(self, bay_index):
        return bool(self.f1s_hes_value[bay_index])

    def is_bay_loaded(self, bay_index):
        return bool(self.hub_hes_value[bay_index])

    def get_spool_status(self, bay_index):
        return self.f1s_hes_value[bay_index]

    def get_current(self):
        return self.i_value

    def stats(self, eventtime):
        return (False,
                "OAMS[%s]: current_spool=%s fps_value=%s"
                " f1s_hes_value_0=%d f1s_hes_value_1=%d f1s_hes_value_2=%d"
                " f1s_hes_value_3=%d hub_hes_value_0=%d hub_hes_value_1=%d"
                " hub_hes_value_2=%d hub_hes_value_3=%d kp=%d ki=%d kd=%d"
                " encoder_clicks=%d i_value=%.2f"
                % (self.oams_idx, self.current_spool, self.fps_value,
                   self.f1s_hes_value[0], self.f1s_hes_value[1],
                   self.f1s_hes_value[2], self.f1s_hes_value[3],
                   self.hub_hes_value[0], self.hub_hes_value[1],
                   self.hub_hes_value[2], self.hub_hes_value[3],
                   self.kp, self.ki, self.kd, self.encoder_clicks,
                   self.i_value))

    def get_webhook_status(self):
        f1s, hub = self.f1s_hes_value, self.hub_hes_value
        return {
            "current_spool": self.current_spool, "fps_value": self.fps_value,
            "f1s_hes_value_0": f1s[0], "f1s_hes_value_1": f1s[1],
            "f1s_hes_value_2": f1s[2], "f1s_hes_value_3": f1s[3],
            "hub_hes_value_0": hub[0], "hub_hes_value_1": hub[1],
            "hub_hes_value_2": hub[2], "hub_hes_value_3": hub[3],
            "kp": self.kp, "ki": self.ki, "kd": self.kd,
            "encoder_clicks": self.encoder_clicks, "i_value": self.i_value,
            "protocol_version": self.protocol_version,
            "gen_matched_protocol": self._use_gen_protocol,
        }

    # -------------------------------------------------------------- connection

    def handle_connect(self):
        try:
            self._resolve_protocol()
            self.oams_load_spool_cmd = self.mcu.lookup_command(
                "oams_cmd_load_spool spool=%c")
            self.oams_unload_spool_cmd = self.mcu.lookup_command(
                "oams_cmd_unload_spool")
            try:
                self.oams_load_spool_cancel_cmd = self.mcu.lookup_command(
                    "oams_cmd_load_spool_cancel")
            except Exception as e:
                logging.warning("OAMS: load-spool-cancel command unavailable"
                                " (update firmware): %s", e)
            self.oams_follower_cmd = self.mcu.lookup_command(
                "oams_cmd_follower enable=%c direction=%c")
            self.oams_calibrate_ptfe_length_cmd = self.mcu.lookup_command(
                "oams_cmd_calibrate_ptfe_length spool=%c")
            self.oams_calibrate_hub_hes_cmd = self.mcu.lookup_command(
                "oams_cmd_calibrate_hub_hes spool=%c")
            self.oams_pid_cmd = self.mcu.lookup_command(
                "oams_cmd_pid kp=%u ki=%u kd=%u target=%u")
            self.oams_set_led_error_cmd = self.mcu.lookup_command(
                "oams_set_led_error idx=%c value=%c")
            cmd_queue = self.mcu.alloc_command_queue()
            self.oams_spool_query_spool_cmd = self.mcu.lookup_query_command(
                "oams_cmd_query_spool", "oams_query_response_spool spool=%u",
                cq=cmd_queue)
            self._detect_gen_protocol()
            self.clear_errors()
        except Exception as e:
            logging.exception("OAMS: failed to initialize commands: %s", e)

    def _resolve_protocol(self):
        """Read the firmware-published protocol contract from the data
        dictionary. Everything is optional: absent keys keep the built-in
        defaults so old firmware (which publishes nothing) still works."""
        consts = {}
        get_constants = getattr(self.mcu, "get_constants", None)
        if get_constants is None:
            # Should not happen on a real Klipper MCU, but never assume.
            logging.warning("OAMS[%s]: MCU has no get_constants(); assuming"
                            " legacy protocol", self.oams_idx)
        else:
            try:
                consts = get_constants() or {}
            except Exception:
                logging.exception("OAMS[%s]: reading firmware constants failed;"
                                  " assuming legacy protocol", self.oams_idx)
        # Distinguish "no OAMS_* published at all" (legacy firmware) from "some
        # published" so a misconfigured/partial dictionary is diagnosable.
        oams_keys = sorted(k for k in consts if k.startswith("OAMS_"))
        logging.info("OAMS[%s]: firmware published %d OAMS_* constant(s)%s",
                     self.oams_idx, len(oams_keys),
                     "" if not oams_keys else ": " + ", ".join(oams_keys))

        self.protocol_version = consts.get("OAMS_PROTOCOL_VERSION")
        if self.protocol_version is None:
            logging.info("OAMS[%s]: firmware publishes no OAMS_PROTOCOL_VERSION;"
                         " using built-in protocol defaults (legacy mode)",
                         self.oams_idx)
        else:
            logging.info("OAMS[%s]: firmware protocol version %s",
                         self.oams_idx, self.protocol_version)
            if self.protocol_version < MIN_PROTOCOL_VERSION:
                logging.warning(
                    "OAMS[%s]: firmware protocol version %s is older than the"
                    " minimum this plugin expects (%s); behaviour may be"
                    " degraded", self.oams_idx, self.protocol_version,
                    MIN_PROTOCOL_VERSION)

        # Action enum: resolved dynamically (driver-local interpretation).
        resolved = {name: consts.get(name, default)
                    for name, default in _ACTION_ENUM_NAMES}
        self.status_loading = resolved["OAMS_STATUS_LOADING"]
        self.status_unloading = resolved["OAMS_STATUS_UNLOADING"]
        self.status_calibrating = resolved["OAMS_STATUS_CALIBRATING"]
        self.status_error = resolved["OAMS_STATUS_ERROR"]

        # Op codes / follower directions: these flow into the pure reducer,
        # which cannot read the dictionary, so we cannot substitute them — we
        # validate that the firmware agrees with the host's compiled-in values
        # and warn loudly on any divergence (a real contract break the version
        # gate is meant to catch).
        mismatches = ["%s: firmware=%s host=%s" % (name, consts[name], default)
                      for name, default in _VALIDATED_ENUM_NAMES
                      if name in consts and consts[name] != default]
        if mismatches:
            logging.error(
                "OAMS[%s]: firmware protocol enum mismatch (%s); the plugin's"
                " result-code handling may be wrong — update the plugin to a"
                " version matching this firmware", self.oams_idx,
                "; ".join(mismatches))

    def _detect_gen_protocol(self):
        """Feature-detect the generation-matched commands (same try/except
        pattern as oams_cmd_load_spool_cancel). When present, switch to the *2
        senders + oams_action_status2 (gen echoed on the wire); otherwise keep
        the legacy commands and the FIFO heuristic."""
        try:
            self.oams_load_spool2_cmd = self.mcu.lookup_command(
                "oams_cmd_load_spool2 spool=%c gen=%c")
            self.oams_unload_spool2_cmd = self.mcu.lookup_command(
                "oams_cmd_unload_spool2 gen=%c")
            self.oams_calibrate_ptfe_length2_cmd = self.mcu.lookup_command(
                "oams_cmd_calibrate_ptfe_length2 spool=%c gen=%c")
            self.oams_calibrate_hub_hes2_cmd = self.mcu.lookup_command(
                "oams_cmd_calibrate_hub_hes2 spool=%c gen=%c")
        except Exception:
            self._use_gen_protocol = False
            logging.info("OAMS[%s]: generation-matched commands unavailable;"
                         " using legacy FIFO completion matching", self.oams_idx)
            return
        # Only register the status2 handler once we know the firmware speaks it,
        # so old firmware never has a dangling response registration.
        self.mcu.register_serial_response(
            self._action_status2_received,
            "oams_action_status2 action=%c code=%c value=%u gen=%c")
        self._use_gen_protocol = True
        logging.info("OAMS[%s]: using generation-matched completion protocol",
                     self.oams_idx)

    def clear_errors(self):
        for i in range(4):
            self.set_led_error(i, 0)
        # CLEAR_ERRORS is the resync point: any replies still owed for
        # abandoned ops are no longer wanted.
        self._gen_queue.clear()
        self.current_spool = self.determine_current_spool()

    def set_led_error(self, idx, value):
        if self.oams_set_led_error_cmd is None:
            return
        logging.info("OAMS: setting LED %d to %d", idx, value)
        self.oams_set_led_error_cmd.send([idx, value])

    def determine_current_spool(self):
        if self.oams_spool_query_spool_cmd is None:
            return None
        params = self.oams_spool_query_spool_cmd.send()
        if params is not None and 0 <= params.get("spool", -1) <= 3:
            return params["spool"]
        return None

    # ---------------------------------------------------------------- commands

    def register_commands(self):
        oams_id = str(self.oams_idx)
        gcode = self.printer.lookup_object("gcode")
        for name, handler, desc in (
            ("OAMS_LOAD_SPOOL", self.cmd_OAMS_LOAD_SPOOL,
             self.cmd_OAMS_LOAD_SPOOL_help),
            ("OAMS_UNLOAD_SPOOL", self.cmd_OAMS_UNLOAD_SPOOL,
             self.cmd_OAMS_UNLOAD_SPOOL_help),
            ("OAMS_FOLLOWER", self.cmd_OAMS_FOLLOWER, self.cmd_OAMS_FOLLOWER_help),
            ("OAMS_CALIBRATE_PTFE_LENGTH", self.cmd_OAMS_CALIBRATE_PTFE_LENGTH,
             self.cmd_OAMS_CALIBRATE_PTFE_LENGTH_help),
            ("OAMS_CALIBRATE_HUB_HES", self.cmd_OAMS_CALIBRATE_HUB_HES,
             self.cmd_OAMS_CALIBRATE_HUB_HES_help),
            ("OAMS_PID_AUTOTUNE", self.cmd_OAMS_PID_AUTOTUNE,
             self.cmd_OAMS_PID_AUTOTUNE_help),
            ("OAMS_PID_SET", self.cmd_OAMS_PID_SET, self.cmd_OAMS_PID_SET_help),
            ("OAMS_CURRENT_PID_SET", self.cmd_OAMS_CURRENT_PID_SET,
             self.cmd_OAMS_CURRENT_PID_SET_help),
        ):
            gcode.register_mux_command(name, "OAMS", oams_id, handler, desc=desc)

    # ------------------------------------------------ standalone blocking wait

    def _wait_for_action(self, timeout=OAMS_ACTION_TIMEOUT):
        """Bounded wait used by the low-level OAMS_* commands when no runtime is
        driving completion. Times out instead of hanging on a lost reply.
        Returns True if the wait timed out (the firmware op may still be
        running and should be cancelled where possible)."""
        endtime = self.reactor.monotonic() + timeout
        while self.action_status is not None:
            if self.reactor.monotonic() >= endtime:
                logging.warning("OAMS[%s]: timed out waiting for firmware status",
                                self.oams_idx)
                self.action_status_code = OAMS_OP_CODE_ERROR_UNSPECIFIED
                self.action_status = None
                return True
            self.reactor.pause(self.reactor.monotonic() + POLL_INTERVAL)
        return False

    # --------------------------------------------------------------- PID setup

    cmd_OAMS_CURRENT_PID_SET_help = "Set the PID values for the rewind current sensor"

    def cmd_OAMS_CURRENT_PID_SET(self, gcmd):
        p, i, d = gcmd.get_float("P"), gcmd.get_float("I"), gcmd.get_float("D")
        t = gcmd.get_float("TARGET", self.current_target)
        self._require_cmd(self.oams_pid_cmd, "PID command")
        self.oams_pid_cmd.send([self.float_to_u32(p), self.float_to_u32(i),
                                self.float_to_u32(d), self.float_to_u32(t)])
        self.current_kp, self.current_ki, self.current_kd = p, i, d
        self.current_target = t
        gcmd.respond_info("Current PID values set to P=%f I=%f D=%f TARGET=%f"
                          % (p, i, d, t))

    cmd_OAMS_PID_SET_help = "Set the PID values for the OAMS hub motor"

    def cmd_OAMS_PID_SET(self, gcmd):
        p, i, d = gcmd.get_float("P"), gcmd.get_float("I"), gcmd.get_float("D")
        t = gcmd.get_float("TARGET", self.fps_target)
        self._require_cmd(self.oams_pid_cmd, "PID command")
        self.oams_pid_cmd.send([self.float_to_u32(p), self.float_to_u32(i),
                                self.float_to_u32(d), self.float_to_u32(t)])
        self.kp, self.ki, self.kd, self.fps_target = p, i, d, t
        gcmd.respond_info("PID values set to P=%f I=%f D=%f TARGET=%f"
                          % (p, i, d, t))

    # TODO: Implement this completely
    cmd_OAMS_PID_AUTOTUNE_help = "Run PID autotune"

    def cmd_OAMS_PID_AUTOTUNE(self, gcmd):
        target_flow = gcmd.get_float("TARGET_FLOW")
        target_temp = gcmd.get_float("TARGET_TEMP")
        extrusion_speed_per_min = 60 * target_flow / (pi * (1.75 / 2) ** 2)
        extrusion_length = extrusion_speed_per_min / 60 * 30
        gcode = self.printer.lookup_object("gcode")
        gcode.run_script_from_command("M104 S%f" % target_temp)
        gcode.run_script_from_command(
            "G1 E%f F%f" % (extrusion_length, extrusion_speed_per_min))

    # ----------------------------------------------------- firmware op senders

    def _require_cmd(self, cmd, name):
        if cmd is None:
            raise self.printer.command_error(
                "OAMS[%s]: %s unavailable (MCU not connected or firmware"
                " initialization failed; see klippy.log)" % (self.oams_idx, name))
        return cmd

    @staticmethod
    def _wire_gen(gen):
        # The gen rides as a single byte (%c). The reducer already keeps op_gen
        # in 0..255; the standalone fallback path passes gen=None -> 0 (unused).
        return 0 if gen is None else (gen & 0xFF)

    def _track_gen(self, gen):
        # Legacy protocol only: remember the gen so the next completion can be
        # tagged with it. Under the *2 protocol the firmware echoes the gen.
        if not self._use_gen_protocol:
            self._gen_queue.append(gen)

    def start_load_spool(self, spool_idx, gen=None):
        # Resolve the command FIRST so a missing-command failure raises before
        # any state is mutated (no phantom FIFO entry, no stuck sentinel).
        if self._use_gen_protocol:
            cmd = self._require_cmd(self.oams_load_spool2_cmd, "load command")
            args = [spool_idx, self._wire_gen(gen)]
        else:
            cmd = self._require_cmd(self.oams_load_spool_cmd, "load command")
            args = [spool_idx]
        self._pending_bay = spool_idx
        self._track_gen(gen)
        self.action_status_code = None
        self.action_status = OAMS_STATUS_LOADING
        cmd.send(args)

    def start_unload_spool(self, gen=None):
        if self._use_gen_protocol:
            cmd = self._require_cmd(self.oams_unload_spool2_cmd, "unload command")
            args = [self._wire_gen(gen)]
        else:
            cmd = self._require_cmd(self.oams_unload_spool_cmd, "unload command")
            args = []
        self._track_gen(gen)
        self.action_status_code = None
        self.action_status = OAMS_STATUS_UNLOADING
        cmd.send(args)

    def start_calibrate(self, kind, bay, gen=None):
        if self._use_gen_protocol:
            cmd = self._require_cmd(
                self.oams_calibrate_hub_hes2_cmd if kind == "hub_hes"
                else self.oams_calibrate_ptfe_length2_cmd,
                "calibrate command")
            args = [bay, self._wire_gen(gen)]
        else:
            cmd = self._require_cmd(
                self.oams_calibrate_hub_hes_cmd if kind == "hub_hes"
                else self.oams_calibrate_ptfe_length_cmd,
                "calibrate command")
            args = [bay]
        self._track_gen(gen)
        self.action_status_code = None
        self.action_status_value = None
        self.action_status = OAMS_STATUS_CALIBRATING
        cmd.send(args)

    def load_spool_cancel(self):
        if self.oams_load_spool_cancel_cmd is None:
            return "OAMS load spool cancel command not available on this firmware"
        self.oams_load_spool_cancel_cmd.send()
        return "OAMS load spool operation cancelled"

    def set_oams_follower(self, enable, direction):
        if self.oams_follower_cmd is None:
            return
        self.oams_follower_cmd.send([enable, direction])

    # --------------------------------------------- low-level OAMS_* commands

    def _result_message(self, code, verb):
        if code == OAMS_OP_CODE_SUCCESS:
            return "Spool %sed successfully" % verb
        if code == OAMS_OP_CODE_CANCEL:
            return "Spool %sing cancelled" % verb
        return "Spool %sing failed (%s)" % (verb, S.describe_code(code))

    cmd_OAMS_LOAD_SPOOL_help = "Load a specific bay on this OAMS"

    def cmd_OAMS_LOAD_SPOOL(self, gcmd):
        bay = gcmd.get_int("SPOOL", minval=0, maxval=3)
        if self.runtime is not None:
            # Route through the store so it is the single source of truth (and
            # the bay gets runout protection if it belongs to a group).
            result = self.runtime.request(
                self.fps_name, S.LoadBay(self.fps_name, (self.oams_idx, bay))).wait()
            if result.ok or result.code == OAMS_OP_CODE_CANCEL:
                gcmd.respond_info(result.message)
            else:
                raise gcmd.error(result.message)
            return
        # Standalone fallback (no manager bound): bounded blocking.
        self.start_load_spool(bay)
        if self._wait_for_action():
            # The firmware op is still running; tell it to stop feeding.
            self.load_spool_cancel()
        code = self.action_status_code
        if code in (OAMS_OP_CODE_SUCCESS, OAMS_OP_CODE_CANCEL):
            gcmd.respond_info(self._result_message(code, "load"))
        else:
            raise gcmd.error(self._result_message(code, "load"))

    cmd_OAMS_UNLOAD_SPOOL_help = "Unload the spool loaded on this OAMS"

    def cmd_OAMS_UNLOAD_SPOOL(self, gcmd):
        requested = gcmd.get_int("SPOOL", None)
        if self.runtime is not None:
            lane = self.runtime.get_state().lanes.get(self.fps_name)
            unit = lane.unit if lane is not None else None
            if unit is None or unit[0] != self.oams_idx:
                self.set_oams_follower(0, FOLLOWER_REVERSE)
                gcmd.respond_info("No spool loaded on this OAMS; nothing to"
                                  " unload (follower stopped)")
                return
            if requested is not None and requested != unit[1]:
                raise gcmd.error("Refusing to unload: spool %d is loaded on this"
                                 " OAMS, not %d" % (unit[1], requested))
            result = self.runtime.request(self.fps_name, S.Unload(self.fps_name)).wait()
            if result.ok:
                gcmd.respond_info(result.message)
            else:
                raise gcmd.error(result.message)  # store already stopped follower
            return
        # Standalone fallback (no manager bound).
        if self.current_spool is None:
            self.set_oams_follower(0, FOLLOWER_REVERSE)
            gcmd.respond_info("No spool is currently loaded on this OAMS; nothing"
                              " to unload (follower stopped)")
            return
        if requested is not None and requested != self.current_spool:
            raise gcmd.error("Refusing to unload: spool %d is loaded on this"
                             " OAMS, not %d" % (self.current_spool, requested))
        self.start_unload_spool()
        self._wait_for_action()
        code = self.action_status_code
        if code == OAMS_OP_CODE_SUCCESS:
            gcmd.respond_info(self._result_message(code, "unload"))
        else:
            self.set_oams_follower(0, FOLLOWER_REVERSE)
            raise gcmd.error(self._result_message(code, "unload"))

    cmd_OAMS_FOLLOWER_help = "Enable or disable the follower and set its direction"

    def cmd_OAMS_FOLLOWER(self, gcmd):
        enable = gcmd.get_int("ENABLE", minval=0, maxval=1)
        direction = gcmd.get_int("DIRECTION", minval=0, maxval=1)
        # When this unit is the lane's loaded unit, go through the store so
        # LaneState.following/direction stay truthful; otherwise drive the
        # hardware directly (the store does not track idle units).
        lane = None
        if self.runtime is not None:
            lane = self.runtime.get_state().lanes.get(self.fps_name)
        if lane is not None and lane.unit is not None \
                and lane.unit[0] == self.oams_idx:
            self.runtime.dispatch(S.Follow(self.fps_name, enable, direction))
        else:
            self.set_oams_follower(enable, direction)
        if enable == 0:
            gcmd.respond_info("Follower disabled")
        elif direction == FOLLOWER_FORWARD:
            gcmd.respond_info("Follower enabled in forward direction")
        else:
            gcmd.respond_info("Follower enabled in reverse direction")

    # ------------------------------------------------- calibration (async/store)

    def _calibrate(self, kind, bay):
        if self.runtime is not None:
            result = self.runtime.request(
                self.fps_name, S.Calibrate(self.fps_name, self.oams_idx, bay, kind)
            ).wait()
            return result.ok, result.value
        # Standalone fallback (no manager): bounded blocking.
        self.start_calibrate(kind, bay)
        self._wait_for_action()
        ok = self.action_status_code == OAMS_OP_CODE_SUCCESS
        return ok, self.action_status_value

    def _persist_option(self, option, value, gcmd):
        """Persist one option on this [oams ...] section. Routes through the
        manager's in-place file writeback (SAVE_CONFIG can't reach an included
        subfile like oams.cfg); falls back to configfile.set only if no manager
        is present (unsupported, but never silently lose the value)."""
        mgr = self.printer.lookup_object("oams_manager", None)
        if mgr is not None and hasattr(mgr, "persist_config_option"):
            mgr.persist_config_option(self.name, option, value)
            return "saved to %s" % mgr.openams_config_path
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.name, option, value)
        return "run SAVE_CONFIG to persist"

    cmd_OAMS_CALIBRATE_HUB_HES_help = "Calibrate the range of a single hub HES"

    def cmd_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        bay = gcmd.get_int("SPOOL", minval=0, maxval=3)
        ok, value = self._calibrate("hub_hes", bay)
        if not ok:
            raise gcmd.error("Calibration of HES %d failed" % bay)
        threshold = self.u32_to_float(value)
        self.hub_hes_on[bay] = threshold
        how = self._persist_option(
            "hub_hes_on", ",".join(map(str, self.hub_hes_on)), gcmd)
        gcmd.respond_info("Calibrated HES %d to %f threshold (%s)."
                          % (bay, threshold, how))

    cmd_OAMS_CALIBRATE_PTFE_LENGTH_help = "Calibrate the length of the PTFE tube"

    def cmd_OAMS_CALIBRATE_PTFE_LENGTH(self, gcmd):
        bay = gcmd.get_int("SPOOL", minval=0, maxval=3)
        ok, value = self._calibrate("ptfe", bay)
        if not ok:
            raise gcmd.error("Calibration of PTFE length failed")
        how = self._persist_option("ptfe_length", "%d" % (value,), gcmd)
        gcmd.respond_info("Calibrated PTFE length to %d (%s)." % (value, how))

    # ----------------------------------------------------- firmware callbacks

    def _oams_cmd_stats(self, params):
        # Serial reader thread. Publish each array as a fresh list (atomic ref
        # swap) so reactor-thread readers always see a consistent snapshot.
        self.fps_value = self.u32_to_float(params["fps_value"])
        self.f1s_hes_value = [params["f1s_hes_value_0"], params["f1s_hes_value_1"],
                              params["f1s_hes_value_2"], params["f1s_hes_value_3"]]
        self.hub_hes_value = [params["hub_hes_value_0"], params["hub_hes_value_1"],
                              params["hub_hes_value_2"], params["hub_hes_value_3"]]
        self.encoder_clicks = params["encoder_clicks"]

    def _oams_cmd_current_stats(self, params):
        self.i_value = self.u32_to_float(params["current_value"])

    def _action_status_received(self, params):
        # Serial reader thread -> marshal onto the reactor thread so the
        # completion bookkeeping (and any waiter) stays single-threaded.
        self.reactor.register_async_callback(
            lambda et, params=params: self._apply_action_status(params))

    def _action_status2_received(self, params):
        # Generation-matched variant: the firmware echoes the op gen, so it is
        # authoritative (no FIFO inference).
        self.reactor.register_async_callback(
            lambda et, params=params: self._apply_action_status(
                params, wire_gen=params["gen"]))

    def _apply_action_status(self, params, wire_gen=None):
        action = params["action"]
        code = params["code"]
        if action == self.status_calibrating:
            self.action_status_value = params["value"]
            self.action_status_code = code
        # Verified against firmware 2.0.25: code 5 (KLIPPER_CALL) only ever
        # co-occurs with action=CALIBRATING, so the `or` clause is redundant
        # there — kept for robustness against other firmware versions. The
        # follower paths (actions 2-5) only use codes BUSY/NO_SPOOL_IN_BAY.
        elif action in (self.status_loading, self.status_unloading,
                        self.status_error) or code == OAMS_OP_CODE_ERROR_KLIPPER_CALL:
            self.action_status_code = code
            # Keep the firmware mirror of current_spool in step with our commands.
            if action == self.status_loading and code == OAMS_OP_CODE_SUCCESS:
                self.current_spool = self._pending_bay
            elif action == self.status_unloading and code == OAMS_OP_CODE_SUCCESS:
                self.current_spool = None
        else:
            # Follower/coast/stop notifications and anything unrecognized are
            # NOT op completions: dispatching them would falsely complete (or
            # fail) an in-flight load/unload on this lane. Log and drop.
            logging.info("OAMS[%s]: ignoring non-completion action status"
                         " action=%d code=%d", self.oams_idx, action, code)
            return
        # Publish the completion sentinel last, then notify the store (if
        # bound). The gen is the firmware echo (wire_gen) under the *2 protocol,
        # or the oldest pending FIFO entry on legacy firmware. Either way the
        # reducer rejects a gen that does not match the in-flight op; an
        # unsolicited completion-class status (gen=None / empty queue) is
        # never accepted.
        self.action_status = None
        if wire_gen is not None:
            gen = wire_gen
        elif self._gen_queue:
            gen = self._gen_queue.popleft()
        else:
            gen = None
        if self.on_action_complete is not None:
            self.on_action_complete(self.action_status_code,
                                    self.action_status_value, gen)

    # -------------------------------------------------------------- utilities

    def float_to_u32(self, f):
        return struct.unpack("I", struct.pack("f", f))[0]

    def u32_to_float(self, i):
        return struct.unpack("f", struct.pack("I", i))[0]

    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_oams_buffer upper=%u lower=%u is_reversed=%u"
            % (self.float_to_u32(self.fps_upper_threshold),
               self.float_to_u32(self.fps_lower_threshold), self.fps_is_reversed))
        self.mcu.add_config_cmd(
            "config_oams_f1s_hes on1=%u on2=%u on3=%u on4=%u is_above=%u"
            % (self.float_to_u32(self.f1s_hes_on[0]),
               self.float_to_u32(self.f1s_hes_on[1]),
               self.float_to_u32(self.f1s_hes_on[2]),
               self.float_to_u32(self.f1s_hes_on[3]), self.f1s_hes_is_above))
        self.mcu.add_config_cmd(
            "config_oams_hub_hes on1=%u on2=%u on3=%u on4=%u is_above=%u"
            % (self.float_to_u32(self.hub_hes_on[0]),
               self.float_to_u32(self.hub_hes_on[1]),
               self.float_to_u32(self.hub_hes_on[2]),
               self.float_to_u32(self.hub_hes_on[3]), self.hub_hes_is_above))
        self.mcu.add_config_cmd(
            "config_oams_pid kp=%u ki=%u kd=%u target=%u"
            % (self.float_to_u32(self.kp), self.float_to_u32(self.ki),
               self.float_to_u32(self.kd), self.float_to_u32(self.fps_target)))
        self.mcu.add_config_cmd(
            "config_oams_ptfe length=%u" % (self.filament_path_length,))
        self.mcu.add_config_cmd(
            "config_oams_current_pid kp=%u ki=%u kd=%u target=%u"
            % (self.float_to_u32(self.current_kp),
               self.float_to_u32(self.current_ki),
               self.float_to_u32(self.current_kd),
               self.float_to_u32(self.current_target)))
        self.mcu.add_config_cmd("config_oams_logger idx=%u" % (self.oams_idx,))


def load_config_prefix(config):
    return OAMS(config)
