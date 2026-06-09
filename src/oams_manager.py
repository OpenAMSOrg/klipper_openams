# OpenAMS Manager
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from collections import deque

from .oams import (
    OAMS_OP_CODE_SUCCESS,
    OAMS_OP_CODE_ERROR_UNSPECIFIED,
    OAMS_ACTION_TIMEOUT,
    POLL_INTERVAL,
    FOLLOWER_FORWARD,
    FOLLOWER_REVERSE,
)

# How often the monitor timer wakes up.
MONITOR_INTERVAL = 1.0

# Distance (mm of extrusion) to keep feeding after a hub runout is detected,
# before letting the follower coast so the tail clears the hub.
PAUSE_DISTANCE = 60.0
# The configured PTFE path length is divided by this factor to decide when
# enough of the old filament has been consumed to start loading the next spool.
# It bakes in the ratio between the firmware "clicks" path length and the
# extruder mm travelled, plus a safety margin so the new tip arrives just before
# the toolhead runs dry.
FILAMENT_PATH_LENGTH_FACTOR = 1.14

# Stall detection while loading/unloading.
ENCODER_SAMPLES = 2
MIN_ENCODER_DIFF = 1
MONITOR_LOADING_SPEED_AFTER = 2.0    # seconds before sampling begins
MONITOR_UNLOADING_SPEED_AFTER = 2.0  # seconds before sampling begins

# Top-level operating states.
STATE_UNLOADED = "UNLOADED"
STATE_LOADED = "LOADED"
STATE_LOADING = "LOADING"
STATE_UNLOADING = "UNLOADING"
STATE_PAUSED = "PAUSED"

# Runout sub-state machine (only relevant while STATE_LOADED and printing).
RUNOUT_IDLE = "idle"
RUNOUT_PAUSING = "pausing"      # feeding PAUSE_DISTANCE before coasting
RUNOUT_COASTING = "coasting"    # follower coasting, consuming the old tail
RUNOUT_LOADING = "loading"      # next spool load in flight (non-blocking)


class OAMSState:
    """Lean snapshot of what the manager is doing right now."""
    def __init__(self):
        self.name = STATE_UNLOADED
        self.since = 0.0
        # The (oam, bay_index) tuple the current operation concerns, or None.
        self.current_unit = None
        self.following = False
        self.direction = FOLLOWER_FORWARD


class OAMSManager:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        self.filament_groups = {}
        self.oams = {}
        self._initialize_oams()
        self._initialize_filament_groups()

        # "What is loaded" is tracked in exactly one place at the manager level:
        # current_group (logical group name) + current_unit ((oam, bay) tuple).
        # The authoritative ground truth on startup/clear is always the hub HES
        # hardware sensor (see determine_current_loaded_group).
        self.current_group = None
        self.current_unit = None
        self.current_state = OAMSState()

        self.reload_before_toolhead_distance = config.getfloat(
            "reload_before_toolhead_distance", 0.0)

        # Stall-detection samples; reset at the start of each load/unload.
        self.encoder_samples = deque(maxlen=ENCODER_SAMPLES)

        # Single monitor timer (registered in start_monitors); never leaks.
        self.monitor_timer = None
        self._reset_runout()

        # Cached object references (resolved at klippy:ready).
        self.extruder = None
        self.idle_timeout = None

        self._load_cancel_requested = False
        self.ready = False

        self.fps = self.printer.lookup_object("fps")
        self.webhooks = self.printer.lookup_object("webhooks")
        self.webhooks.register_endpoint("openams/status", self._webhook_status)
        self.webhooks.register_endpoint("openams/cancel_load",
                                        self._webhook_cancel_load)

        self.register_commands()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    # ------------------------------------------------------------ bootstrap

    def _initialize_oams(self):
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam

    def _initialize_filament_groups(self):
        seen = set()
        for name, group in self.printer.lookup_objects(module="filament_group"):
            group_name = name.split()[-1]
            if group_name in seen:
                raise self.config.error(
                    "Duplicate filament_group '%s' found. Filament group names"
                    " must be unique." % group_name)
            seen.add(group_name)
            logging.info("OAMS: adding group %s", group_name)
            self.filament_groups[group_name] = group

    def handle_ready(self):
        self.extruder = self.printer.lookup_object("extruder")
        self.idle_timeout = self.printer.lookup_object("idle_timeout")
        self.determine_state()
        self.start_monitors()
        self.ready = True

    # --------------------------------------------------------------- status

    def get_status(self, eventtime):
        return {"current_group": self.current_group}

    def _webhook_status(self, request):
        status = {
            "ready": self.ready,
            "current_group": self.current_group,
            "units": len(self.oams),
            "fps_value": self.fps.get_value(),
            "filament_groups": {},
        }
        for group_name, group in self.filament_groups.items():
            status["filament_groups"][group_name] = {
                "bays": len(group.bays),
                "spools": ["oams%d-%d" % (oam.oams_idx, bay)
                           for (oam, bay) in group.bays],
            }
        request.send({"status": {"openams": status}})

    def _webhook_cancel_load(self, request):
        if self.current_state.name == STATE_LOADING:
            self._load_cancel_requested = True
            request.send({"result": "cancel requested"})
        else:
            request.send({"result": "no load in progress"})

    # ---------------------------------------------------- shared state helpers

    def _enter_state(self, name, unit):
        self.current_state.name = name
        self.current_state.since = self.reactor.monotonic()
        self.current_state.current_unit = unit
        # Fresh stall-detection window for the new operation.
        self.encoder_samples.clear()

    def _reset_runout(self):
        self.runout_phase = RUNOUT_IDLE
        self._runout_pause_origin = None
        self._runout_coast_origin = None
        self._reload_oam = None
        self._reload_bay = None
        self._reload_deadline = None

    def _clear_loaded_state(self):
        self.current_group = None
        self.current_unit = None

    def determine_current_loaded_group(self):
        for group_name, group in self.filament_groups.items():
            for (oam, bay) in group.bays:
                if oam.is_bay_loaded(bay):
                    return group_name, oam, bay
        return None, None, None

    def determine_state(self):
        """Resync manager state from the hub HES hardware (the source of truth)."""
        self._reset_runout()
        group_name, oam, bay = self.determine_current_loaded_group()
        if oam is not None:
            self.current_group = group_name
            self.current_unit = (oam, bay)
            self._enter_state(STATE_LOADED, self.current_unit)
        else:
            self._clear_loaded_state()
            self._enter_state(STATE_UNLOADED, None)
            # Nothing is loaded, so no follower should run. The firmware keeps a
            # follower active across a host restart, so an aborted unload could
            # otherwise leave one rewinding forever ("restart didn't help").
            self._stop_all_followers()

    def is_printer_loaded(self):
        return self._loaded_oam() is not None

    def _loaded_oam(self):
        for _, oam in self.oams.items():
            if oam.current_spool is not None:
                return oam
        return None

    def _stop_all_followers(self):
        for _, oam in self.oams.items():
            try:
                oam.set_oams_follower(0, FOLLOWER_REVERSE)
            except Exception:
                logging.exception("OAMS: could not stop follower on %s", oam.name)

    def _is_printing(self, eventtime):
        if self.idle_timeout is None:
            return False
        return self.idle_timeout.get_status(eventtime)["state"] == "Printing"

    def _pause_print(self, reason):
        logging.info("OAMS: pausing print: %s", reason)
        try:
            gcode = self.printer.lookup_object("gcode")
            gcode.run_script("M118 OAMS: %s" % reason)
            gcode.run_script("M117 OAMS paused")
            gcode.run_script("PAUSE")
        except Exception:
            # Never let a failing PAUSE escape a reactor timer callback.
            logging.exception("OAMS: failed to issue PAUSE")

    # ------------------------------------------------------------- monitors

    def start_monitors(self):
        self.stop_monitors()
        self.monitor_timer = self.reactor.register_timer(
            self._monitor, self.reactor.NOW)
        logging.info("OAMS: monitor started")

    def stop_monitors(self):
        if self.monitor_timer is not None:
            self.reactor.unregister_timer(self.monitor_timer)
            self.monitor_timer = None

    def _monitor(self, eventtime):
        # One always-on timer drives all monitoring. It never returns NEVER, so
        # it is never re-registered and cannot leak or fork into parallel chains.
        try:
            name = self.current_state.name
            if name == STATE_UNLOADING:
                self._check_speed(eventtime, MONITOR_UNLOADING_SPEED_AFTER,
                                  "unloading")
            elif name == STATE_LOADING:
                self._check_speed(eventtime, MONITOR_LOADING_SPEED_AFTER,
                                  "loading")
            elif name == STATE_LOADED:
                self._service_runout(eventtime)
        except Exception:
            logging.exception("OAMS: monitor tick failed")
        return eventtime + MONITOR_INTERVAL

    def _check_speed(self, eventtime, after, what):
        unit = self.current_state.current_unit
        if unit is None:
            return
        if self.reactor.monotonic() - self.current_state.since <= after:
            return
        oam, bay = unit
        self.encoder_samples.append(oam.encoder_clicks)
        if len(self.encoder_samples) < ENCODER_SAMPLES:
            return
        diff = abs(self.encoder_samples[-1] - self.encoder_samples[0])
        logging.info("OAMS[%d] %s monitor: encoder diff %d",
                     oam.oams_idx, what, diff)
        if diff < MIN_ENCODER_DIFF:
            oam.set_led_error(bay, 1)
            self._pause_print("%s speed too low" % what)
            self.current_state.name = STATE_PAUSED

    # ----------------------------------------------------- runout/auto-reload

    def _service_runout(self, eventtime):
        phase = self.runout_phase

        if phase == RUNOUT_IDLE:
            if (self._is_printing(eventtime) and self.current_unit is not None
                    and not self._unit_loaded(self.current_unit)):
                logging.info("OAMS: runout detected on group %s; feeding %.0fmm"
                             " before coasting", self.current_group, PAUSE_DISTANCE)
                self._runout_pause_origin = self.extruder.last_position
                self.runout_phase = RUNOUT_PAUSING
            return

        # All later phases need a loaded unit; bail safely if it vanished (e.g. a
        # manual unload happened mid-runout).
        if self.current_unit is None:
            self._reset_runout()
            return

        if phase == RUNOUT_PAUSING:
            travelled = self.extruder.last_position - self._runout_pause_origin
            if travelled >= PAUSE_DISTANCE:
                logging.info("OAMS: pause complete, coasting the follower")
                self.current_unit[0].set_oams_follower(0, FOLLOWER_FORWARD)
                self._runout_coast_origin = self.extruder.last_position
                self.runout_phase = RUNOUT_COASTING

        elif phase == RUNOUT_COASTING:
            path_length = self.current_unit[0].filament_path_length
            if path_length <= 0:
                # ptfe_length is uncalibrated, so the handoff distance is
                # unknown. Pause for the user rather than loading at the wrong
                # time. Run OAMS_CALIBRATE_PTFE_LENGTH to enable auto-reload.
                self._pause_print(
                    "ptfe_length is not calibrated (0); cannot auto-load the"
                    " next spool. Run OAMS_CALIBRATE_PTFE_LENGTH.")
                self._clear_loaded_state()
                self._reset_runout()
                return
            consumed = self.extruder.last_position - self._runout_coast_origin
            path_limit = path_length / FILAMENT_PATH_LENGTH_FACTOR
            if (consumed + PAUSE_DISTANCE + self.reload_before_toolhead_distance
                    > path_limit):
                self._begin_reload(eventtime)

        elif phase == RUNOUT_LOADING:
            self._service_reload(eventtime)

    def _unit_loaded(self, unit):
        oam, bay = unit
        return oam.is_bay_loaded(bay)

    def _begin_reload(self, eventtime):
        group = self.filament_groups.get(self.current_group)
        ranout = self.current_unit
        if group is not None:
            for (oam, bay) in group.bays:
                if (oam, bay) == ranout:
                    continue  # never try to reload the spool that just ran out
                if oam.is_bay_ready(bay):
                    logging.info("OAMS: loading next spool oams%d bay %d",
                                 oam.oams_idx, bay)
                    oam.start_load_spool(bay)
                    self._reload_oam = oam
                    self._reload_bay = bay
                    self._reload_deadline = eventtime + OAMS_ACTION_TIMEOUT
                    self.runout_phase = RUNOUT_LOADING
                    return
        self._pause_print("filament runout on group %s and no spare spool"
                          " available" % self.current_group)
        self._clear_loaded_state()
        self._reset_runout()

    def _service_reload(self, eventtime):
        oam, bay = self._reload_oam, self._reload_bay
        if oam.action_status is None:
            code, message = oam.finish_load_spool(bay)
            if code == OAMS_OP_CODE_SUCCESS:
                logging.info("OAMS: next spool loaded oams%d bay %d",
                             oam.oams_idx, bay)
                self.current_unit = (oam, bay)
                self.current_state.current_unit = self.current_unit
                self._reset_runout()  # back to LOADED/idle
            else:
                self._pause_print("failed to load next spool: %s" % message)
                self._clear_loaded_state()
                self._reset_runout()
        elif eventtime >= self._reload_deadline:
            logging.warning("OAMS: timed out loading next spool")
            # Release the firmware op so the OAMS is not stuck "loading".
            oam.action_status = None
            oam.action_status_code = OAMS_OP_CODE_ERROR_UNSPECIFIED
            self._pause_print("timed out loading next spool")
            self._clear_loaded_state()
            self._reset_runout()

    # -------------------------------------------------------------- commands

    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        for name, handler, desc in (
            ("OAMSM_UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT,
             self.cmd_UNLOAD_FILAMENT_help),
            ("OAMSM_LOAD_FILAMENT", self.cmd_LOAD_FILAMENT,
             self.cmd_LOAD_FILAMENT_help),
            ("OAMSM_FOLLOWER", self.cmd_FOLLOWER, self.cmd_FOLLOWER_help),
            ("OAMSM_CURRENT_LOADED_GROUP", self.cmd_CURRENT_LOADED_GROUP,
             self.cmd_CURRENT_LOADED_GROUP_help),
            ("OAMSM_CLEAR_ERRORS", self.cmd_CLEAR_ERRORS,
             self.cmd_CLEAR_ERRORS_help),
            ("OAMSM_LOAD_FILAMENT_CANCEL", self.cmd_LOAD_FILAMENT_CANCEL,
             self.cmd_LOAD_FILAMENT_CANCEL_help),
        ):
            gcode.register_command(name, handler, desc=desc)

    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"

    def cmd_CLEAR_ERRORS(self, gcmd):
        self.stop_monitors()
        self.encoder_samples.clear()
        for _, oam in self.oams.items():
            oam.clear_errors()
        self.determine_state()
        self.start_monitors()

    cmd_CURRENT_LOADED_GROUP_help = "Get the current loaded group"

    def cmd_CURRENT_LOADED_GROUP(self, gcmd):
        group_name, _, _ = self.determine_current_loaded_group()
        gcmd.respond_info(group_name if group_name is not None
                          else "No group is currently loaded")

    cmd_FOLLOWER_help = "Enable the follower on whichever OAMS is loaded"

    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int("ENABLE", minval=0, maxval=1)
        direction = gcmd.get_int("DIRECTION", minval=0, maxval=1)
        oam = self._loaded_oam()
        if oam is None:
            gcmd.respond_info("No spool is currently loaded")
            return
        oam.set_oams_follower(enable, direction)
        self.current_state.following = bool(enable)
        self.current_state.direction = direction

    cmd_LOAD_FILAMENT_CANCEL_help = "Cancel the current load filament operation"

    def cmd_LOAD_FILAMENT_CANCEL(self, gcmd):
        unit = self.current_state.current_unit
        if self.current_state.name == STATE_LOADING and unit is not None:
            gcmd.respond_info(unit[0].load_spool_cancel())
        else:
            gcmd.respond_info("No load filament operation is currently in progress")

    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"

    def cmd_UNLOAD_FILAMENT(self, gcmd):
        self._reset_runout()
        oam = self._loaded_oam()
        if oam is None:
            # SAFE_UNLOAD_FILAMENT enables the follower in the rewind direction
            # before calling this command; stop it so it cannot keep rewinding.
            self._stop_all_followers()
            self._clear_loaded_state()
            self._enter_state(STATE_UNLOADED, None)
            gcmd.respond_info("No spool is loaded in any of the OAMS")
            return
        self._enter_state(STATE_UNLOADING, (oam, oam.current_spool))
        code, message = oam.unload_spool()
        if code == OAMS_OP_CODE_SUCCESS:
            self._clear_loaded_state()
            self._enter_state(STATE_UNLOADED, None)
        else:
            # Unload did not complete; stop the follower so it cannot keep
            # rewinding, and keep reporting the spool as still loaded.
            oam.set_oams_follower(0, FOLLOWER_REVERSE)
            self._enter_state(STATE_LOADED, (oam, oam.current_spool))
            gcmd.respond_info(message)

    cmd_LOAD_FILAMENT_help = "Load a spool from a specific group"

    def cmd_LOAD_FILAMENT(self, gcmd):
        self._reset_runout()
        if self.is_printer_loaded():
            gcmd.respond_info("Printer is already loaded with a spool")
            return
        group_name = gcmd.get("GROUP")
        group = self.filament_groups.get(group_name)
        if group is None:
            raise gcmd.error("Group %s does not exist" % group_name)
        for (oam, bay) in group.bays:
            if not oam.is_bay_ready(bay):
                continue
            self._enter_state(STATE_LOADING, (oam, bay))
            self._load_cancel_requested = False
            oam.start_load_spool(bay)
            self._wait_for_load_or_cancel(oam)
            code, message = oam.finish_load_spool(bay)
            logging.info("OAMS[%d] load result: code=%s msg=%s",
                         oam.oams_idx, code, message)
            if code == OAMS_OP_CODE_SUCCESS:
                self.current_group = group_name
                self.current_unit = (oam, bay)
                self._enter_state(STATE_LOADED, self.current_unit)
                gcmd.respond_info(message)
            else:
                # CANCEL or error: nothing is seated, so do not record a load.
                self._clear_loaded_state()
                self._enter_state(STATE_UNLOADED, None)
                gcmd.respond_info(message)
            return
        gcmd.respond_info("No spool available for group %s" % group_name)

    def _wait_for_load_or_cancel(self, oam):
        """Block until the load completes, a cancel is requested (via the
        webhook), or the timeout fires. The webhook callback runs from a reactor
        timer during reactor.pause(), so the flag is observed between polls."""
        endtime = self.reactor.monotonic() + OAMS_ACTION_TIMEOUT
        while oam.action_status is not None:
            now = self.reactor.monotonic()
            if now >= endtime:
                oam.action_status = None
                oam.action_status_code = OAMS_OP_CODE_ERROR_UNSPECIFIED
                break
            if self._load_cancel_requested:
                self._load_cancel_requested = False
                oam.load_spool_cancel()
            self.reactor.pause(now + POLL_INTERVAL)


def load_config(config):
    return OAMSManager(config)
