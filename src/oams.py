# OpenAMS Mainboard
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import mcu
import struct
from math import pi

OAMS_STATUS_LOADING = 0
OAMS_STATUS_UNLOADING = 1
OAMS_STATUS_FORWARD_FOLLOWING = 2
OAMS_STATUS_REVERSE_FOLLOWING = 3
OAMS_STATUS_COASTING = 4
OAMS_STATUS_STOPPED = 5
OAMS_STATUS_CALIBRATING = 6
OAMS_STATUS_ERROR = 7

OAMS_OP_CODE_SUCCESS = 0
OAMS_OP_CODE_ERROR_UNSPECIFIED = 1
OAMS_OP_CODE_ERROR_BUSY = 2
OAMS_OP_CODE_SPOOL_ALREADY_IN_BAY  = 3
OAMS_OP_CODE_NO_SPOOL_IN_BAY = 4
OAMS_OP_CODE_ERROR_KLIPPER_CALL  = 5


class OAMS:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu = mcu.get_printer_mcu(self.printer, config.get("mcu", "mcu"))
        self.fps_upper_threshold = config.getfloat("fps_upper_threshold")
        self.fps_lower_threshold = config.getfloat("fps_lower_threshold")
        self.fps_is_reversed = config.getboolean("fps_is_reversed")
        self.i_value = 0.0
        self.encoder_clicks = 0
        
        self.f1s_hes_on = list(
            map(lambda x: float(x.strip()), config.get("f1s_hes_on").split(","))
        )
        self.f1s_hes_is_above = config.getboolean("f1s_hes_is_above")
        self.hub_hes_on = list(
            map(lambda x: float(x.strip()), config.get("hub_hes_on").split(","))
        )
        self.hub_hes_is_above = config.getboolean("hub_hes_is_above")
        self.filament_path_length = config.getfloat("ptfe_length")
        self.oams_idx = config.getint("oams_idx")

        self.kd = config.getfloat("kd", 0.0)
        self.ki = config.getfloat("ki", 0.0)
        self.kp = config.getfloat("kp", 6.0)

        self.current_kp = config.getfloat("current_kp", 0.375)
        self.current_ki = config.getfloat("current_ki", 0.0)
        self.current_kd = config.getfloat("current_kd", 0.0)

        self.fps_target = config.getfloat(
            "fps_target",
            0.5,
            minval=0.0,
            maxval=1.0,
            above=self.fps_lower_threshold,
            below=self.fps_upper_threshold,
        )
        self.current_target = config.getfloat(
            "current_target", 0.3, minval=0.1, maxval=0.4
        )
        self.current_spool = None
        self.mcu.register_response(self._oams_action_status, "oams_action_status")
        self.mcu.register_response(self._oams_cmd_stats, "oams_cmd_stats")
        self.mcu.register_response(self._oams_cmd_current_stats, "oams_cmd_current_status")
        self.mcu.register_config_callback(self._build_config)
        self.name = config.get_name()
        self.register_commands(self.name.split()[-1])
        # self.printer.add_object("oams", self)
        self.reactor = self.printer.get_reactor()
        self.action_status = None
        self.action_status_code = None
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.fps_value = 0
        self.f1s_hes_value = [0, 0, 0, 0]
        self.hub_hes_value = [0, 0, 0, 0]
        super().__init__()

    def get_status(self, eventtime):
        return {"current_spool": self.current_spool}
    
    def is_bay_ready(self, bay_index):
        return bool(self.f1s_hes_value[bay_index])
    
    def is_bay_loaded(self, bay_index):
        return bool(self.hub_hes_value[bay_index])

    def stats(self, eventtime):
        return (
            False,
            """
OAMS[%s]: current_spool=%s fps_value=%s f1s_hes_value_0=%d f1s_hes_value_1=%d f1s_hes_value_2=%d f1s_hes_value_3=%d hub_hes_value_0=%d hub_hes_value_1=%d hub_hes_value_2=%d hub_hes_value_3=%d kp=%d ki=%d kd=%d encoder_clicks=%d i_value=%.2f
"""
            % ( self.oams_idx,
                self.current_spool,
                self.fps_value,
                self.f1s_hes_value[0],
                self.f1s_hes_value[1],
                self.f1s_hes_value[2],
                self.f1s_hes_value[3],
                self.hub_hes_value[0],
                self.hub_hes_value[1],
                self.hub_hes_value[2],
                self.hub_hes_value[3],
                self.kp,
                self.ki,
                self.kd,
                self.encoder_clicks,
                self.i_value,
            ),
        )
    
    def get_webhook_status(self):
        return {
            "current_spool": self.current_spool,
            "fps_value": self.fps_value,
            "f1s_hes_value_0": self.f1s_hes_value[0],
            "f1s_hes_value_1": self.f1s_hes_value[1],
            "f1s_hes_value_2": self.f1s_hes_value[2],
            "f1s_hes_value_3": self.f1s_hes_value[3],
            "hub_hes_value_0": self.hub_hes_value[0],
            "hub_hes_value_1": self.hub_hes_value[1],
            "hub_hes_value_2": self.hub_hes_value[2],
            "hub_hes_value_3": self.hub_hes_value[3],
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
            "encoder_clicks": self.encoder_clicks,
            "i_value": self.i_value,
        }

    def handle_connect(self):
        try:
            self.oams_load_spool_cmd = self.mcu.lookup_command(
                "oams_cmd_load_spool spool=%c"
            )

            self.oams_unload_spool_cmd = self.mcu.lookup_command(
                "oams_cmd_unload_spool"
            )

            self.oams_follower_cmd = self.mcu.lookup_command(
                "oams_cmd_follower enable=%c direction=%c"
            )

            self.oams_calibrate_ptfe_length_cmd = self.mcu.lookup_command(
                "oams_cmd_calibrate_ptfe_length spool=%c"
            )

            self.oams_calibrate_hub_hes_cmd = self.mcu.lookup_command(
                "oams_cmd_calibrate_hub_hes spool=%c"
            )

            self.oams_pid_cmd = self.mcu.lookup_command(
                "oams_cmd_pid kp=%u ki=%u kd=%u target=%u"
            )
            # TODO: change this to reset the state of the AMS and determine it again instead
            #       of directly doing it via klipper, let the firmware handle it
            self.oams_set_led_error_cmd = self.mcu.lookup_command(
                "oams_set_led_error idx=%c value=%c"
            )

            cmd_queue = self.mcu.alloc_command_queue()

            self.oams_spool_query_spool_cmd = self.mcu.lookup_query_command(
                "oams_cmd_query_spool",
                "oams_query_response_spool spool=%u",
                cq=cmd_queue,
            )
            
            self.clear_errors()
            
        except Exception as e:
            logging.error("Failed to initialize OAMS commands: %s", e)

    def get_spool_status(self, bay_index):
        return self.f1s_hes_value[bay_index]
            
    def clear_errors(self):
        for i in range(4):
            self.set_led_error(i, 0)
        self.current_spool = self.determine_current_spool()
        self.determine_current_spool()
            
    def set_led_error(self, idx, value):
        logging.info("Setting LED %d to %d", idx, value)
        self.oams_set_led_error_cmd.send([idx, value])
        # TODO: need to restore the actual value of the LED when resetting the error
        
            
    def determine_current_spool(self):
        params = self.oams_spool_query_spool_cmd.send()
        if params is not None and "spool" in params:
            if params["spool"] >= 0 and params["spool"] <= 3:
                return params["spool"]
        return None
        

    def register_commands(self, name):
        id = str(self.oams_idx)
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "OAMS_LOAD_SPOOL",
            "OAMS",
            id,
            self.cmd_OAMS_LOAD_SPOOL,
            desc=self.cmd_OAMS_LOAD_SPOOL_help,
        )
        gcode.register_mux_command(
            "OAMS_UNLOAD_SPOOL",
            "OAMS",
            id,
            self.cmd_OAMS_UNLOAD_SPOOL,
            self.cmd_OAMS_UNLOAD_SPOOL_help,
        )

        gcode.register_mux_command(
            "OAMS_FOLLOWER",
            "OAMS",
            id,
            self.cmd_OAMS_FOLLOWER,
            self.cmd_OAMS_FOLLOWER_help,
        )

        gcode.register_mux_command(
            "OAMS_CALIBRATE_PTFE_LENGTH",
            "OAMS",
            id,
            self.cmd_OAMS_CALIBRATE_PTFE_LENGTH,
            self.cmd_OAMS_CALIBRATE_PTFE_LENGTH_help,
        )

        gcode.register_mux_command(
            "OAMS_CALIBRATE_HUB_HES",
            "OAMS",
            id,
            self.cmd_OAMS_CALIBRATE_HUB_HES,
            self.cmd_OAMS_CALIBRATE_HUB_HES_help,
        )

        gcode.register_mux_command(
            "OAMS_PID_AUTOTUNE",
            "OAMS",
            id,
            self.cmd_OAMS_PID_AUTOTUNE,
            self.cmd_OAMS_PID_AUTOTUNE_help,
        )

        gcode.register_mux_command(
            "OAMS_PID_SET",
            "OAMS",
            id,
            self.cmd_OAMS_PID_SET,
            self.cmd_OAMS_PID_SET_help,
        )

        gcode.register_mux_command(
            "OAMS_CURRENT_PID_SET",
            "OAMS",
            id,
            self.cmd_OAMS_CURRENT_PID_SET,
            self.cmd_OAMS_CURRENT_PID_SET_help,
        )

    cmd_OAMS_CURRENT_PID_SET_help = "Set the PID values for the current sensor"

    def cmd_OAMS_CURRENT_PID_SET(self, gcmd):
        p = gcmd.get_float("P", None)
        i = gcmd.get_float("I", None)
        d = gcmd.get_float("D", None)
        t = gcmd.get_float("TARGET", None)
        if p is None:
            raise gcmd.error("P value is required")
        if i is None:
            raise gcmd.error("I value is required")
        if d is None:
            raise gcmd.error("D value is required")
        if t is None:
            t = self.current_target
        kp = self.float_to_u32(p)
        ki = self.float_to_u32(i)
        kd = self.float_to_u32(d)
        kt = self.float_to_u32(t)
        self.oams_pid_cmd.send([kp, ki, kd, kt])
        self.current_kp = p
        self.current_ki = i
        self.current_kd = d
        self.current_target = t
        gcmd.respond_info(
            "Current PID values set to P=%f I=%f D=%f TARGET=%f" % (p, i, d, t)
        )

    cmd_OAMS_PID_SET_help = "Set the PID values for the OAMS"

    def cmd_OAMS_PID_SET(self, gcmd):
        p = gcmd.get_float("P", None)
        i = gcmd.get_float("I", None)
        d = gcmd.get_float("D", None)
        t = gcmd.get_float("TARGET", None)
        if p is None:
            raise gcmd.error("P value is required")
        if i is None:
            raise gcmd.error("I value is required")
        if d is None:
            raise gcmd.error("D value is required")
        if t is None:
            t = self.fps_target
        kp = self.float_to_u32(p)
        ki = self.float_to_u32(i)
        kd = self.float_to_u32(d)
        kt = self.float_to_u32(t)
        self.oams_pid_cmd.send([kp, ki, kd, kt])
        self.kp = p
        self.ki = i
        self.kd = d
        self.fps_target = t
        gcmd.respond_info("PID values set to P=%f I=%f D=%f TARGET=%f" % (p, i, d, t))

    # TODO: Implement this completely
    cmd_OAMS_PID_AUTOTUNE_help = "Run PID autotune"

    def cmd_OAMS_PID_AUTOTUNE(self, gcmd):
        target_flow = gcmd.get_float("TARGET_FLOW", None)
        target_temp = gcmd.get_float("TARGET_TEMP", None)

        if target_flow is None:
            raise gcmd.error("TARGET flowrate in mm^3/s is required")
        if target_temp is None:
            raise gcmd.error("TARGET temperature in degrees C is required")

        # Given a flowrate we will calculate 30 seconds of a G1 E command
        extrusion_speed_per_min = (
            60 * target_flow / (pi * (1.75 / 2) ** 2)
        )  # this is the G1 F parameter
        extrusion_length = (
            extrusion_speed_per_min / 60 * 30
        )  # this is the G1 E parameter

        gcode = self.printer.lookup_object("gcode")

        # turn on extruder heater and wait for it to stabilize
        gcode.send("M104 S%f" % target_temp)
        gcode.send("G1 E%f F%f" % (extrusion_length, extrusion_speed_per_min))

    cmd_OAMS_CALIBRATE_HUB_HES_help = "Calibrate the range of a single hub HES"

    def cmd_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        self.action_status = OAMS_STATUS_CALIBRATING
        spool_idx = gcmd.get_int("SPOOL", None)
        if spool_idx is None:
            raise gcmd.error("SPOOL index is required")
        if spool_idx < 0 or spool_idx > 3:
            raise gcmd.error("Invalid SPOOL index")
        self.oams_calibrate_hub_hes_cmd.send([spool_idx])
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMS_OP_CODE_SUCCESS:
            value = self.u32_to_float(self.action_status_value)
            gcmd.respond_info("Calibrated HES %d to %f threshold" % (spool_idx, value))
            configfile = self.printer.lookup_object("configfile")
            self.hub_hes_on[spool_idx] = value
            values = ",".join(map(str, self.hub_hes_on))
            configfile.set(self.name, "hub_hes_on", "%s" % (values,))
            gcmd.respond_info("Done calibrating HES; please note this value, and update parameter hub_hes_on for index %d in the configuration" % (spool_idx,))
        else:
            gcmd.error("Calibration of HES %d failed" % spool_idx)

    cmd_OAMS_CALIBRATE_PTFE_LENGTH_help = "Calibrate the length of the PTFE tube"

    def cmd_OAMS_CALIBRATE_PTFE_LENGTH(self, gcmd):
        self.action_status = OAMS_STATUS_CALIBRATING
        spool = gcmd.get_int("SPOOL", None)
        if spool is None:
            raise gcmd.error("SPOOL index is required")
        self.oams_calibrate_ptfe_length_cmd.send([spool])
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMS_OP_CODE_SUCCESS:
            gcmd.respond_info("Calibrated PTFE length to %d" % self.action_status_value)
            configfile = self.printer.lookup_object("configfile")
            configfile.set(self.name, "ptfe_length", "%d" % (self.action_status_value,))
            gcmd.respond_info("Done calibrating clicks, please note this value and update parameter ptfe_length in the configuration")
        else:
            gcmd.error("Calibration of PTFE length failed")

    def load_spool(self, spool_idx):
        self.action_status = OAMS_STATUS_LOADING
        self.oams_load_spool_cmd.send([spool_idx])
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMS_OP_CODE_SUCCESS:
            self.current_spool = spool_idx
            return True, "Spool loaded successfully"
        elif self.action_status_code == OAMS_OP_CODE_ERROR_KLIPPER_CALL:
            return False, "Spool loading stopped by klipper monitor"
        elif self.action_status_code == OAMS_OP_CODE_ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS with code %d" % self.action_status_code

    cmd_OAMS_LOAD_SPOOL_help = "Load a new spool of filament"

    def cmd_OAMS_LOAD_SPOOL(self, gcmd):
        self.action_status = OAMS_STATUS_LOADING
        self.oams_spool_query_spool_cmd.send()
        spool_idx = gcmd.get_int("SPOOL", None)
        if spool_idx is None:
            raise gcmd.error("SPOOL index is required")
        if spool_idx < 0 or spool_idx > 3:
            raise gcmd.error("Invalid SPOOL index")
        
        success, message = self.load_spool(spool_idx)
        
        if success:
            gcmd.respond_info(message)
        else:
            gcmd.error(message)
            
    def unload_spool(self):
        self.action_status = OAMS_STATUS_UNLOADING
        self.oams_unload_spool_cmd.send()
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMS_OP_CODE_SUCCESS:
            self.current_spool = None
            return True, "Spool unloaded successfully"
        elif self.action_status_code == OAMS_OP_CODE_ERROR_KLIPPER_CALL:
            return False, "Spool unloading stopped by klipper monitor"
        elif self.action_status_code == OAMS_OP_CODE_ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS"

    cmd_OAMS_UNLOAD_SPOOL_help = "Unload a spool of filament"

    def cmd_OAMS_UNLOAD_SPOOL(self, gcmd):
        success, message = self.unload_spool()
        if success:
            gcmd.respond_info(message)
        else:
            gcmd.error(message)

    def set_oams_follower(self, enable, direction):
        self.oams_follower_cmd.send([enable, direction])

    cmd_OAMS_FOLLOWER_help = "Enable or disable follower and set its direction"

    def cmd_OAMS_FOLLOWER(self, gcmd):
        enable = gcmd.get_int("ENABLE", None)
        if enable is None:
            raise gcmd.error("ENABLE is required")
        direction = gcmd.get_int("DIRECTION", None)
        if direction is None:
            raise gcmd.error("DIRECTION is required")
        self.oams_follower_cmd.send([enable, direction])
        if enable == 1 and direction == 0:
            gcmd.respond_info("Follower enable in reverse direction")
        elif enable == 1 and direction == 1:
            gcmd.respond_info("Follower enable in forward direction")
        elif enable == 0:
            gcmd.respond_info("Follower disabled")

    def _oams_cmd_stats(self, params):
        self.fps_value = self.u32_to_float(params["fps_value"])
        self.f1s_hes_value[0] = params["f1s_hes_value_0"]
        self.f1s_hes_value[1] = params["f1s_hes_value_1"]
        self.f1s_hes_value[2] = params["f1s_hes_value_2"]
        self.f1s_hes_value[3] = params["f1s_hes_value_3"]
        self.hub_hes_value[0] = params["hub_hes_value_0"]
        self.hub_hes_value[1] = params["hub_hes_value_1"]
        self.hub_hes_value[2] = params["hub_hes_value_2"]
        self.hub_hes_value[3] = params["hub_hes_value_3"]
        self.encoder_clicks = params["encoder_clicks"]
        
    def _oams_cmd_current_stats(self, params):
        self.i_value = self.u32_to_float(params["current_value"])

    def get_current(self):
        return self.i_value

    def _oams_action_status(self, params):
        logging.info("oams status received")
        if params["action"] == OAMS_STATUS_LOADING:
            self.action_status = None
            self.action_status_code = params["code"]
        elif params["action"] == OAMS_STATUS_UNLOADING:
            self.action_status = None
            self.action_status_code = params["code"]
        elif params["action"] == OAMS_STATUS_CALIBRATING:
            self.action_status = None
            self.action_status_code = params["code"]
            self.action_status_value = params["value"]
        elif params["action"] == OAMS_STATUS_ERROR:
            self.action_status = None
            self.action_status_code = params["code"]
        elif params["code"] == OAMS_OP_CODE_ERROR_KLIPPER_CALL:
            self.action_status = None
            self.action_status_code = params["code"]
        else:
            logging.error(
                "Spurious response from AMS with code %d and action %d",
                params["code"],
                params["action"],
            )

    def float_to_u32(self, f):
        return struct.unpack("I", struct.pack("f", f))[0]

    def u32_to_float(self, i):
        return struct.unpack("f", struct.pack("I", i))[0]

    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_oams_buffer upper=%u lower=%u is_reversed=%u"
            % (
                self.float_to_u32(self.fps_upper_threshold),
                self.float_to_u32(self.fps_lower_threshold),
                self.fps_is_reversed,
            )
        )

        self.mcu.add_config_cmd(
            "config_oams_f1s_hes on1=%u on2=%u on3=%u on4=%u is_above=%u"
            % (
                self.float_to_u32(self.f1s_hes_on[0]),
                self.float_to_u32(self.f1s_hes_on[1]),
                self.float_to_u32(self.f1s_hes_on[2]),
                self.float_to_u32(self.f1s_hes_on[3]),
                self.f1s_hes_is_above,
            )
        )

        self.mcu.add_config_cmd(
            "config_oams_hub_hes on1=%u on2=%u on3=%u on4=%u is_above=%u"
            % (
                self.float_to_u32(self.hub_hes_on[0]),
                self.float_to_u32(self.hub_hes_on[1]),
                self.float_to_u32(self.hub_hes_on[2]),
                self.float_to_u32(self.hub_hes_on[3]),
                self.hub_hes_is_above,
            )
        )

        self.mcu.add_config_cmd(
            "config_oams_pid kp=%u ki=%u kd=%u target=%u"
            % (
                self.float_to_u32(self.kp),
                self.float_to_u32(self.ki),
                self.float_to_u32(self.kd),
                self.float_to_u32(self.fps_target),
            )
        )

        self.mcu.add_config_cmd(
            "config_oams_ptfe length=%u" % (self.filament_path_length)
        )

        self.mcu.add_config_cmd(
            "config_oams_current_pid kp=%u ki=%u kd=%u target=%u"
            % (
                self.float_to_u32(self.current_kp),
                self.float_to_u32(self.current_ki),
                self.float_to_u32(self.current_kd),
                self.float_to_u32(self.current_target),
            )
        )

        self.mcu.add_config_cmd("config_oams_logger idx=%u" % (self.oams_idx))


def load_config_prefix(config):
    return OAMS(config)
