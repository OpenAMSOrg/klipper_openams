# OpenAMS Mainboard
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import struct
from math import pi

import mcu

# Firmware "action" identifiers reported by oams_action_status.
OAMS_STATUS_LOADING = 0
OAMS_STATUS_UNLOADING = 1
OAMS_STATUS_FORWARD_FOLLOWING = 2
OAMS_STATUS_REVERSE_FOLLOWING = 3
OAMS_STATUS_COASTING = 4
OAMS_STATUS_STOPPED = 5
OAMS_STATUS_CALIBRATING = 6
OAMS_STATUS_ERROR = 7

# Firmware operation result codes. NOTE: SUCCESS is 0 (falsy) -- always compare
# result codes with "== OAMS_OP_CODE_SUCCESS", never with "if code:".
OAMS_OP_CODE_SUCCESS = 0
OAMS_OP_CODE_ERROR_UNSPECIFIED = 1
OAMS_OP_CODE_ERROR_BUSY = 2
OAMS_OP_CODE_SPOOL_ALREADY_IN_BAY = 3
OAMS_OP_CODE_NO_SPOOL_IN_BAY = 4
OAMS_OP_CODE_ERROR_KLIPPER_CALL = 5
OAMS_OP_CODE_CANCEL = 6

# How often the blocking waiters re-check completion, and the upper bound on how
# long they will wait before giving up. The timeout is a safety net: without it a
# dropped/garbled firmware response would wedge the operation (and the print)
# forever.
POLL_INTERVAL = 0.1
OAMS_ACTION_TIMEOUT = 120.0

# Follower direction codes (firmware convention).
FOLLOWER_REVERSE = 0
FOLLOWER_FORWARD = 1


class OAMS:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Full config-section name (e.g. "oams oams1"); needed for configfile.set.
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()
        self.mcu = mcu.get_printer_mcu(self.printer, config.get("mcu", "mcu"))

        self.fps_upper_threshold = config.getfloat("fps_upper_threshold")
        self.fps_lower_threshold = config.getfloat("fps_lower_threshold")
        self.fps_is_reversed = config.getboolean("fps_is_reversed")

        self.f1s_hes_on = [
            float(x.strip()) for x in config.get("f1s_hes_on").split(",")
        ]
        self.f1s_hes_is_above = config.getboolean("f1s_hes_is_above")
        self.hub_hes_on = [
            float(x.strip()) for x in config.get("hub_hes_on").split(",")
        ]
        self.hub_hes_is_above = config.getboolean("hub_hes_is_above")
        # PTFE path length in firmware clicks. 0 is allowed: a fresh machine must
        # boot uncalibrated so OAMS_CALIBRATE_PTFE_LENGTH can be run. The runout
        # auto-reload guards against an uncalibrated (<= 0) length rather than
        # rejecting it here (see OAMSManager._service_runout).
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
            above=self.fps_lower_threshold, below=self.fps_upper_threshold,
        )
        self.current_target = config.getfloat(
            "current_target", 0.3, minval=0.1, maxval=0.4
        )

        # Index of the spool currently loaded through the hub (0..3), or None.
        self.current_spool = None

        # Firmware command handles, resolved at klippy:connect. Pre-set to None so
        # a missing/older-firmware command degrades gracefully instead of raising
        # AttributeError the first time it is referenced.
        self.oams_load_spool_cmd = None
        self.oams_unload_spool_cmd = None
        self.oams_load_spool_cancel_cmd = None
        self.oams_follower_cmd = None
        self.oams_calibrate_ptfe_length_cmd = None
        self.oams_calibrate_hub_hes_cmd = None
        self.oams_pid_cmd = None
        self.oams_set_led_error_cmd = None
        self.oams_spool_query_spool_cmd = None

        # Live telemetry.
        self.fps_value = 0.0
        self.i_value = 0.0
        self.encoder_clicks = 0
        self.f1s_hes_value = [0, 0, 0, 0]
        self.hub_hes_value = [0, 0, 0, 0]

        # Completion signalling for blocking operations (load/unload/calibrate).
        #
        # Threading model: firmware messages are dispatched on the serial reader
        # thread. The high-rate sensor handler updates whole-list references
        # atomically (safe to read from the reactor thread). The action-status
        # handler instead marshals onto the reactor thread via
        # register_async_callback, so action_status / _code / _value are only ever
        # written on the reactor thread -- the same thread the waiters poll on --
        # which removes the read/write race entirely. action_status is always
        # published last so a waiter that observes "done" sees a settled code.
        self.action_status = None
        self.action_status_code = None
        self.action_status_value = None

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

    # ------------------------------------------------------------------ status

    def get_status(self, eventtime):
        return {"current_spool": self.current_spool}

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
        f1s = self.f1s_hes_value
        hub = self.hub_hes_value
        return {
            "current_spool": self.current_spool,
            "fps_value": self.fps_value,
            "f1s_hes_value_0": f1s[0], "f1s_hes_value_1": f1s[1],
            "f1s_hes_value_2": f1s[2], "f1s_hes_value_3": f1s[3],
            "hub_hes_value_0": hub[0], "hub_hes_value_1": hub[1],
            "hub_hes_value_2": hub[2], "hub_hes_value_3": hub[3],
            "kp": self.kp, "ki": self.ki, "kd": self.kd,
            "encoder_clicks": self.encoder_clicks, "i_value": self.i_value,
        }

    # -------------------------------------------------------------- connection

    def handle_connect(self):
        try:
            self.oams_load_spool_cmd = self.mcu.lookup_command(
                "oams_cmd_load_spool spool=%c")
            self.oams_unload_spool_cmd = self.mcu.lookup_command(
                "oams_cmd_unload_spool")
            try:
                self.oams_load_spool_cancel_cmd = self.mcu.lookup_command(
                    "oams_cmd_load_spool_cancel")
            except Exception as e:
                logging.warning(
                    "OAMS: load-spool-cancel command unavailable (update"
                    " firmware to enable cancellation): %s", e)
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
            self.clear_errors()
        except Exception as e:
            logging.exception("OAMS: failed to initialize commands: %s", e)

    def clear_errors(self):
        for i in range(4):
            self.set_led_error(i, 0)
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
            ("OAMS_FOLLOWER", self.cmd_OAMS_FOLLOWER,
             self.cmd_OAMS_FOLLOWER_help),
            ("OAMS_CALIBRATE_PTFE_LENGTH", self.cmd_OAMS_CALIBRATE_PTFE_LENGTH,
             self.cmd_OAMS_CALIBRATE_PTFE_LENGTH_help),
            ("OAMS_CALIBRATE_HUB_HES", self.cmd_OAMS_CALIBRATE_HUB_HES,
             self.cmd_OAMS_CALIBRATE_HUB_HES_help),
            ("OAMS_PID_AUTOTUNE", self.cmd_OAMS_PID_AUTOTUNE,
             self.cmd_OAMS_PID_AUTOTUNE_help),
            ("OAMS_PID_SET", self.cmd_OAMS_PID_SET,
             self.cmd_OAMS_PID_SET_help),
            ("OAMS_CURRENT_PID_SET", self.cmd_OAMS_CURRENT_PID_SET,
             self.cmd_OAMS_CURRENT_PID_SET_help),
        ):
            gcode.register_mux_command(name, "OAMS", oams_id, handler, desc=desc)

    # ------------------------------------------------------- completion waiter

    def _wait_for_action(self, timeout=OAMS_ACTION_TIMEOUT):
        """Block the calling reactor greenlet until the in-flight firmware
        action completes, or until `timeout` seconds elapse. On timeout a failure
        code is synthesized so callers never hang on a lost/garbled response."""
        endtime = self.reactor.monotonic() + timeout
        while self.action_status is not None:
            if self.reactor.monotonic() >= endtime:
                logging.warning(
                    "OAMS[%s]: timed out waiting for firmware action status",
                    self.oams_idx)
                self.action_status_code = OAMS_OP_CODE_ERROR_UNSPECIFIED
                self.action_status = None
                break
            self.reactor.pause(self.reactor.monotonic() + POLL_INTERVAL)

    # --------------------------------------------------------------- PID setup

    cmd_OAMS_CURRENT_PID_SET_help = "Set the PID values for the current sensor"

    def cmd_OAMS_CURRENT_PID_SET(self, gcmd):
        p = gcmd.get_float("P")
        i = gcmd.get_float("I")
        d = gcmd.get_float("D")
        t = gcmd.get_float("TARGET", self.current_target)
        self.oams_pid_cmd.send([self.float_to_u32(p), self.float_to_u32(i),
                                self.float_to_u32(d), self.float_to_u32(t)])
        self.current_kp, self.current_ki, self.current_kd = p, i, d
        self.current_target = t
        gcmd.respond_info(
            "Current PID values set to P=%f I=%f D=%f TARGET=%f" % (p, i, d, t))

    cmd_OAMS_PID_SET_help = "Set the PID values for the OAMS"

    def cmd_OAMS_PID_SET(self, gcmd):
        p = gcmd.get_float("P")
        i = gcmd.get_float("I")
        d = gcmd.get_float("D")
        t = gcmd.get_float("TARGET", self.fps_target)
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
        # Convert the requested volumetric flow into a 30 second G1 E move.
        extrusion_speed_per_min = 60 * target_flow / (pi * (1.75 / 2) ** 2)
        extrusion_length = extrusion_speed_per_min / 60 * 30
        gcode = self.printer.lookup_object("gcode")
        gcode.run_script_from_command("M104 S%f" % target_temp)
        gcode.run_script_from_command(
            "G1 E%f F%f" % (extrusion_length, extrusion_speed_per_min))

    # ------------------------------------------------------------ calibration

    cmd_OAMS_CALIBRATE_HUB_HES_help = "Calibrate the range of a single hub HES"

    def cmd_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        spool_idx = gcmd.get_int("SPOOL", minval=0, maxval=3)
        self.action_status_code = None
        self.action_status_value = None
        self.action_status = OAMS_STATUS_CALIBRATING
        self.oams_calibrate_hub_hes_cmd.send([spool_idx])
        self._wait_for_action()
        if self.action_status_code != OAMS_OP_CODE_SUCCESS:
            raise gcmd.error("Calibration of HES %d failed" % spool_idx)
        value = self.u32_to_float(self.action_status_value)
        self.hub_hes_on[spool_idx] = value
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.name, "hub_hes_on", ",".join(map(str, self.hub_hes_on)))
        gcmd.respond_info(
            "Calibrated HES %d to %f threshold. Run SAVE_CONFIG (or update"
            " hub_hes_on) to persist it." % (spool_idx, value))

    cmd_OAMS_CALIBRATE_PTFE_LENGTH_help = "Calibrate the length of the PTFE tube"

    def cmd_OAMS_CALIBRATE_PTFE_LENGTH(self, gcmd):
        spool = gcmd.get_int("SPOOL", minval=0, maxval=3)
        self.action_status_code = None
        self.action_status_value = None
        self.action_status = OAMS_STATUS_CALIBRATING
        self.oams_calibrate_ptfe_length_cmd.send([spool])
        self._wait_for_action()
        if self.action_status_code != OAMS_OP_CODE_SUCCESS:
            raise gcmd.error("Calibration of PTFE length failed")
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.name, "ptfe_length", "%d" % (self.action_status_value,))
        gcmd.respond_info(
            "Calibrated PTFE length to %d. Run SAVE_CONFIG (or update"
            " ptfe_length) to persist it." % (self.action_status_value,))

    # ------------------------------------------------------------ load / unload

    def start_load_spool(self, spool_idx):
        """Send the load command and return immediately. Poll action_status
        until None, then call finish_load_spool() for the result."""
        self.action_status_code = None
        self.action_status = OAMS_STATUS_LOADING
        self.oams_load_spool_cmd.send([spool_idx])

    def finish_load_spool(self, spool_idx):
        """Interpret action_status_code once action_status has become None.
        Returns (op_code, message)."""
        code = self.action_status_code
        if code == OAMS_OP_CODE_SUCCESS:
            self.current_spool = spool_idx
            return code, "Spool loaded successfully"
        if code == OAMS_OP_CODE_CANCEL:
            # Loading was aborted partway; filament is NOT seated. Deliberately
            # leave current_spool unset so the spool is never treated as loaded.
            return code, "Spool loading cancelled"
        if code == OAMS_OP_CODE_ERROR_KLIPPER_CALL:
            return code, "Spool loading stopped by klipper monitor"
        if code == OAMS_OP_CODE_ERROR_BUSY:
            return code, "OAMS is busy"
        return code, "Spool loading failed (code %s)" % (code,)

    def load_spool(self, spool_idx):
        """Blocking load. Returns (op_code, message)."""
        self.start_load_spool(spool_idx)
        self._wait_for_action()
        return self.finish_load_spool(spool_idx)

    def load_spool_cancel(self):
        if self.oams_load_spool_cancel_cmd is None:
            return "OAMS load spool cancel command not available on this firmware"
        self.oams_load_spool_cancel_cmd.send()
        return "OAMS load spool operation cancelled"

    def start_unload_spool(self):
        self.action_status_code = None
        self.action_status = OAMS_STATUS_UNLOADING
        self.oams_unload_spool_cmd.send()

    def finish_unload_spool(self):
        """Interpret action_status_code after an unload. Returns (op_code, msg)."""
        code = self.action_status_code
        if code == OAMS_OP_CODE_SUCCESS:
            self.current_spool = None
            return code, "Spool unloaded successfully"
        if code == OAMS_OP_CODE_ERROR_KLIPPER_CALL:
            return code, "Spool unloading stopped by klipper monitor"
        if code == OAMS_OP_CODE_ERROR_BUSY:
            return code, "OAMS is busy"
        return code, "Spool unloading failed (code %s)" % (code,)

    def unload_spool(self):
        """Blocking unload. Returns (op_code, message)."""
        self.start_unload_spool()
        self._wait_for_action()
        return self.finish_unload_spool()

    cmd_OAMS_LOAD_SPOOL_help = "Load a new spool of filament"

    def cmd_OAMS_LOAD_SPOOL(self, gcmd):
        spool_idx = gcmd.get_int("SPOOL", minval=0, maxval=3)
        code, message = self.load_spool(spool_idx)
        if code in (OAMS_OP_CODE_SUCCESS, OAMS_OP_CODE_CANCEL):
            gcmd.respond_info(message)
        else:
            raise gcmd.error(message)

    cmd_OAMS_UNLOAD_SPOOL_help = "Unload a spool of filament"

    def cmd_OAMS_UNLOAD_SPOOL(self, gcmd):
        # Only unload the spool that is actually loaded. Issuing an unload while
        # nothing (or a different spool) is loaded would drive the firmware into
        # a rewind/follower state that never completes, leaving the follower
        # running indefinitely.
        requested = gcmd.get_int("SPOOL", None)
        if self.current_spool is None:
            self.set_oams_follower(0, FOLLOWER_REVERSE)
            gcmd.respond_info(
                "No spool is currently loaded on this OAMS; nothing to unload"
                " (follower stopped)")
            return
        if requested is not None and requested != self.current_spool:
            raise gcmd.error(
                "Refusing to unload: spool %d is loaded on this OAMS, not %d"
                % (self.current_spool, requested))
        code, message = self.unload_spool()
        if code == OAMS_OP_CODE_SUCCESS:
            gcmd.respond_info(message)
        else:
            # The unload did not complete cleanly; stop the follower so it does
            # not keep rewinding the spool.
            self.set_oams_follower(0, FOLLOWER_REVERSE)
            raise gcmd.error(message)

    # ------------------------------------------------------------- follower

    def set_oams_follower(self, enable, direction):
        if self.oams_follower_cmd is None:
            return
        self.oams_follower_cmd.send([enable, direction])

    cmd_OAMS_FOLLOWER_help = "Enable or disable follower and set its direction"

    def cmd_OAMS_FOLLOWER(self, gcmd):
        enable = gcmd.get_int("ENABLE", minval=0, maxval=1)
        direction = gcmd.get_int("DIRECTION", minval=0, maxval=1)
        self.set_oams_follower(enable, direction)
        if enable == 0:
            gcmd.respond_info("Follower disabled")
        elif direction == FOLLOWER_FORWARD:
            gcmd.respond_info("Follower enabled in forward direction")
        else:
            gcmd.respond_info("Follower enabled in reverse direction")

    # ----------------------------------------------------- firmware callbacks

    def _oams_cmd_stats(self, params):
        # Runs on the serial reader thread. Publish each array as a fresh list so
        # readers on the reactor thread always see a consistent snapshot via a
        # single (atomic) reference swap rather than a half-updated list.
        self.fps_value = self.u32_to_float(params["fps_value"])
        self.f1s_hes_value = [params["f1s_hes_value_0"], params["f1s_hes_value_1"],
                              params["f1s_hes_value_2"], params["f1s_hes_value_3"]]
        self.hub_hes_value = [params["hub_hes_value_0"], params["hub_hes_value_1"],
                              params["hub_hes_value_2"], params["hub_hes_value_3"]]
        self.encoder_clicks = params["encoder_clicks"]

    def _oams_cmd_current_stats(self, params):
        self.i_value = self.u32_to_float(params["current_value"])

    def _action_status_received(self, params):
        # Runs on the serial reader thread. Marshal onto the reactor thread so the
        # completion bookkeeping (and the waiters polling it) stay single-threaded.
        self.reactor.register_async_callback(
            lambda et, params=params: self._apply_action_status(params))

    def _apply_action_status(self, params):
        action = params["action"]
        code = params["code"]
        if action == OAMS_STATUS_CALIBRATING:
            self.action_status_value = params["value"]
            self.action_status_code = code
        elif action in (OAMS_STATUS_LOADING, OAMS_STATUS_UNLOADING,
                        OAMS_STATUS_ERROR) or code == OAMS_OP_CODE_ERROR_KLIPPER_CALL:
            self.action_status_code = code
        else:
            # Defensive default: an unrecognized action/code combination still
            # releases any waiter, so a stray or garbled message can never wedge
            # an operation forever.
            logging.error(
                "OAMS[%s]: unexpected action status code=%d action=%d",
                self.oams_idx, code, action)
            self.action_status_code = OAMS_OP_CODE_ERROR_UNSPECIFIED
        # Publish the completion sentinel last (see threading note in __init__).
        self.action_status = None

    # -------------------------------------------------------------- utilities

    def float_to_u32(self, f):
        return struct.unpack("I", struct.pack("f", f))[0]

    def u32_to_float(self, i):
        return struct.unpack("f", struct.pack("I", i))[0]

    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_oams_buffer upper=%u lower=%u is_reversed=%u"
            % (self.float_to_u32(self.fps_upper_threshold),
               self.float_to_u32(self.fps_lower_threshold),
               self.fps_is_reversed))
        self.mcu.add_config_cmd(
            "config_oams_f1s_hes on1=%u on2=%u on3=%u on4=%u is_above=%u"
            % (self.float_to_u32(self.f1s_hes_on[0]),
               self.float_to_u32(self.f1s_hes_on[1]),
               self.float_to_u32(self.f1s_hes_on[2]),
               self.float_to_u32(self.f1s_hes_on[3]),
               self.f1s_hes_is_above))
        self.mcu.add_config_cmd(
            "config_oams_hub_hes on1=%u on2=%u on3=%u on4=%u is_above=%u"
            % (self.float_to_u32(self.hub_hes_on[0]),
               self.float_to_u32(self.hub_hes_on[1]),
               self.float_to_u32(self.hub_hes_on[2]),
               self.float_to_u32(self.hub_hes_on[3]),
               self.hub_hes_is_above))
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
