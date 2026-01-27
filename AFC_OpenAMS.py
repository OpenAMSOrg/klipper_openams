# Armored Turtle Automated Filament Changer  
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# OPTIMIZATIONS APPLIED:
# 1. Object Caching: Cache frequently accessed objects (gcode, extruder, lane, OAMS index)
#    to eliminate redundant printer.lookup_object() calls
# 2. Event-Driven Architecture: All sensor updates via AMSHardwareService event subscriptions
#    instead of polling, eliminating unnecessary sensor reads
# 3. Sensor Helper Caching: Virtual sensor helpers cached to avoid repeated lookups
# 4. Registry Integration: Uses LaneRegistry for O(1) lane lookups across units

"""AMS integration helpers for Armored Turtle AFC."""

from __future__ import annotations

import json
import os
import re
import traceback
from textwrap import dedent
from datetime import datetime
from types import MethodType
from typing import Any, Dict, List, Optional, Set, Tuple

from configparser import Error as ConfigError

from extras.AFC_prep import afcPrep
try: from extras.AFC_utils import ERROR_STR
except: raise ConfigError("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLane, AFCLaneState
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))
try: from extras.AFC_utils import add_filament_switch
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_utils", trace=traceback.format_exc()))
try: import extras.AFC_extruder as _afc_extruder_mod
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_extruder", trace=traceback.format_exc()))
try: from extras.AFC_respond import AFCprompt
except: raise ConfigError(ERROR_STR.format(import_lib="AFC_respond", trace=traceback.format_exc()))

try:
    from extras.openams_integration import (
        AMSHardwareService,
        AMSRunoutCoordinator,
        LaneRegistry,
        AMSEventBus,
        normalize_extruder_name,
        OPENAMS_VERSION,
    )
except Exception:
    AMSHardwareService = None
    AMSRunoutCoordinator = None
    LaneRegistry = None
    AMSEventBus = None
    normalize_extruder_name = None
    OPENAMS_VERSION = "0.0.3"  # Fallback if import fails

_ORIGINAL_LANE_PRE_SENSOR = getattr(AFCLane, "get_toolhead_pre_sensor_state", None)
_ORIGINAL_PERFORM_INFINITE_RUNOUT = getattr(AFCLane, "_perform_infinite_runout", None)
_ORIGINAL_PREP_CAPTURE_TD1 = getattr(AFCLane, "_prep_capture_td1", None)
_ORIGINAL_GET_TD1_DATA = getattr(AFCLane, "get_td1_data", None)
_ORIGINAL_TD1_PREP = getattr(afcPrep, "_td1_prep", None)
_ORIGINAL_LANE_UNLOAD = None  # Will be set during patching
_ORIGINAL_BUFFER_SET_MULTIPLIER = None  # Will be set during patching
_ORIGINAL_BUFFER_GET_STATUS = None  # Will be set during patching
_ORIGINAL_BUFFER_EXTRUDER_POS_UPDATE = None  # Will be set during patching

class _VirtualRunoutHelper:
    """Minimal runout helper used by AMS-managed virtual sensors."""

    def __init__(self, printer, name, runout_cb=None, enable_runout=False):
        self.printer = printer
        self._reactor = printer.get_reactor()
        self.name = name
        self.runout_callback = runout_cb
        self.sensor_enabled = bool(enable_runout)
        self.filament_present = False
        self.insert_gcode = None
        self.runout_gcode = None
        self.event_delay = 0.0
        self.min_event_systime = self._reactor.NEVER

    def note_filament_present(self, eventtime=None, is_filament_present=False, **_kwargs):
        if eventtime is None:
            eventtime = self._reactor.monotonic()

        new_state = bool(is_filament_present)
        if new_state == self.filament_present:
            return

        self.filament_present = new_state

        if (not new_state and self.sensor_enabled and callable(self.runout_callback)):
            try:
                self.runout_callback(eventtime)
            except TypeError:
                self.runout_callback(eventtime=eventtime)

    def get_status(self, _eventtime=None):
        return {
            "filament_detected": bool(self.filament_present),
            "enabled": bool(self.sensor_enabled),
        }

class _VirtualFilamentSensor:
    """Lightweight filament sensor placeholder for AMS virtual pins."""

    QUERY_HELP = "Query the status of the Filament Sensor"
    SET_HELP = "Sets the filament sensor on/off"

    def __init__(self, printer, name, show_in_gui=True, runout_cb=None, enable_runout=False):
        self.printer = printer
        self.name = name
        self._object_name = f"filament_switch_sensor {name}"
        self.runout_helper = _VirtualRunoutHelper(printer, name, runout_cb=runout_cb, enable_runout=enable_runout)

        objects = getattr(printer, "objects", None)
        if isinstance(objects, dict):
            objects.setdefault(self._object_name, self)
            if not show_in_gui:
                hidden_key = "_" + self._object_name
                objects[hidden_key] = objects.pop(self._object_name)

        # Use lookup with None to prevent errors if gcode not yet loaded
        gcode = printer.lookup_object("gcode", None)
        if gcode is None:
            return
        try:
            gcode.register_mux_command("QUERY_FILAMENT_SENSOR", "SENSOR", name, self.cmd_QUERY_FILAMENT_SENSOR, desc=self.QUERY_HELP)
        except Exception:
            pass

        try:
            gcode.register_mux_command("SET_FILAMENT_SENSOR", "SENSOR", name, self.cmd_SET_FILAMENT_SENSOR, desc=self.SET_HELP)
        except Exception:
            pass

    def get_status(self, eventtime):
        return self.runout_helper.get_status(eventtime)

    def cmd_QUERY_FILAMENT_SENSOR(self, gcmd):
        status = self.runout_helper.get_status(None)
        if status["filament_detected"]:
            msg = f"Filament Sensor {self.name}: filament detected"
        else:
            msg = f"Filament Sensor {self.name}: filament not detected"
        gcmd.respond_info(msg)

    def cmd_SET_FILAMENT_SENSOR(self, gcmd):
        self.runout_helper.sensor_enabled = bool(gcmd.get_int("ENABLE", 1))

def _normalize_extruder_name(name: Optional[str]) -> Optional[str]:
    """Return a case-insensitive token for comparing extruder aliases."""
    if callable(normalize_extruder_name):
        try:
            return normalize_extruder_name(name)
        except Exception:
            pass

    if not name or not isinstance(name, str):
        return None

    normalized = name.strip()
    if not normalized:
        return None

    lowered = normalized.lower()
    if lowered.startswith("ams_"):
        lowered = lowered[4:]

    return lowered or None

def _normalize_ams_pin_value(pin_value) -> Optional[str]:
    """Return the cleaned AMS_* token stripped of comments and modifiers."""
    if not isinstance(pin_value, str):
        return None

    cleaned = pin_value.strip()
    if not cleaned:
        return None

    for comment_char in ("#", ";"):
        idx = cleaned.find(comment_char)
        if idx != -1:
            cleaned = cleaned[:idx].strip()
    if not cleaned:
        return None

    while cleaned and cleaned[0] in "!^":
        cleaned = cleaned[1:]

    return cleaned or None

def _patch_extruder_for_virtual_ams() -> None:
    """Patch AFC extruders so AMS_* tool pins avoid config-time errors."""
    extruder_cls = getattr(_afc_extruder_mod, "AFCExtruder", None)
    if extruder_cls is None or getattr(extruder_cls, "_ams_virtual_tool_patched", False):
        return

    base_init = extruder_cls.__init__

    class _ProxyConfig:
        def __init__(self, original):
            self._original = original

        def get(self, key, *args, **kwargs):
            if key == "pin_tool_start":
                return "buffer"
            return self._original.get(key, *args, **kwargs)

        def __getattr__(self, item):
            return getattr(self._original, item)

    def _patched_init(self, config):
        try:
            pin_value = config.get("pin_tool_start", None)
        except Exception:
            pin_value = None

        normalized = _normalize_ams_pin_value(pin_value)
        proxy_config = config

        if normalized:
            if normalized.upper().startswith("AMS_"):
                proxy_config = _ProxyConfig(config)
            else:
                normalized = None

        base_init(self, proxy_config)

        if not normalized:
            return

        show_sensor = getattr(self, "enable_sensors_in_gui", True)
        enable_runout = getattr(self, "enable_runout", False)
        runout_cb = getattr(self, "handle_start_runout", None)

        setattr(self, "_ams_virtual_tool_name", normalized)

        virtual = _VirtualFilamentSensor(self.printer, normalized, show_in_gui=show_sensor, runout_cb=runout_cb, enable_runout=enable_runout)

        self.tool_start = pin_value
        self.fila_tool_start = virtual
        self.tool_start_state = bool(virtual.runout_helper.filament_present)

    extruder_cls.__init__ = _patched_init
    extruder_cls._ams_virtual_tool_patched = True

class afcAMS(afcUnit):
    """AFC unit subclass that synchronises state with OpenAMS"""

    _sync_command_registered = False
    _sync_instances: Dict[str, "afcAMS"] = {}
    _hydrated_extruders: Set[str] = set()

    def __init__(self, config):
        super().__init__(config)
        self.type = "OpenAMS"
        self.logger = self.afc.logger

        # AMS units don't have physical buffers - force buffer_obj to None
        # This prevents buffer monitoring/fault detection from running on AMS lanes
        # even if user accidentally configured a buffer parameter
        self.buffer_obj = None

        # Ensure LED attributes are set (inherited from AFC_unit but may not be set if AFC base is missing)
        # These are needed by AFC_lane.py handle_unit_connect (lines 391-393)
        if not hasattr(self, 'led_tool_loaded'):
            self.led_tool_loaded = config.get('led_tool_loaded', '0,0,1,0')
        if not hasattr(self, 'led_tool_loaded_idle'):
            self.led_tool_loaded_idle = config.get('led_tool_loaded_idle', '0.4,0.4,0,0')
        if not hasattr(self, 'led_tool_unloaded'):
            self.led_tool_unloaded = config.get('led_tool_unloaded', '1,0,0,0')

        self.oams_name = config.get("oams", "oams1")

        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Lane registry integration
        self.registry = None
        if LaneRegistry is not None:
            try:
                self.registry = LaneRegistry.for_printer(self.printer)
            except Exception:
                self.logger.error("Failed to initialize LaneRegistry")

        # Event bus subscription for spool changes
        self.event_bus = None
        if AMSEventBus is not None:
            try:
                self.event_bus = AMSEventBus.get_instance()
                self.event_bus.subscribe("spool_loaded", self._handle_spool_loaded_event, priority=10)
                self.event_bus.subscribe("spool_unloaded", self._handle_spool_unloaded_event, priority=10)
            except Exception:
                self.logger.error("Failed to subscribe to AMS events")

        # NOTE: This class uses AFC native state exclusively (no duplicate tracking):
        # - extruder.lane_loaded - which lane is loaded to toolhead
        # - lane.load_state - filament at F1S sensor
        # - lane.loaded_to_hub - filament at hub sensor
        # - lane.tool_loaded - filament loaded to extruder
        # Previous versions maintained local caches, now removed for single source of truth

        self._saved_unit_cache: Optional[Dict[str, Any]] = None
        self._saved_unit_mtime: Optional[float] = None

        self._virtual_tool_sensor = None
        # Keep _last_hub_hes_values for HES calibration (not an AFC responsibility)
        self._last_hub_hes_values: Optional[List[float]] = None

        # OPTIMIZATION: Cache frequently accessed objects
        self._cached_sensor_helper = None
        self._cached_gcode = None
        self._cached_extruder_objects: Dict[str, Any] = {}
        self._cached_lane_objects: Dict[str, Any] = {}
        self._cached_oams_index: Optional[int] = None

        # Track pending TD-1 capture timers (delayed after spool insertion)
        self._pending_spool_loaded_timers: Dict[str, Any] = {}

        self.oams = None
        self.hardware_service = None

        if AMSRunoutCoordinator is not None:
            self.hardware_service = AMSRunoutCoordinator.register_afc_unit(self)
        elif AMSHardwareService is not None:
            self.hardware_service = AMSHardwareService.for_printer(self.printer, self.oams_name, self.logger)

        self._register_sync_dispatcher()
        self._patch_td1_capture()
        self._patch_td1_cali_fail_prompt()
        self._patch_td1_prep()

        self.gcode.register_mux_command("AFC_OAMS_CALIBRATE_HUB_HES", "UNIT", self.name, self.cmd_AFC_OAMS_CALIBRATE_HUB_HES, desc="calibrate the OpenAMS HUB HES value for a specific lane")
        self.gcode.register_mux_command("AFC_OAMS_CALIBRATE_HUB_HES_ALL", "UNIT", self.name, self.cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL, desc="calibrate the OpenAMS HUB HES value for every loaded lane")
        self.gcode.register_mux_command("AFC_OAMS_CALIBRATE_PTFE", "UNIT", self.name, self.cmd_AFC_OAMS_CALIBRATE_PTFE, desc="calibrate the OpenAMS PTFE length for a specific lane")
        self.gcode.register_mux_command("UNIT_PTFE_CALIBRATION", "UNIT", self.name, self.cmd_UNIT_PTFE_CALIBRATION, desc="show OpenAMS PTFE calibration menu")

    def _is_openams_unit(self):
        """Check if this unit has OpenAMS hardware available."""
        return self.oams is not None

    def _patch_td1_capture(self):
        if getattr(AFCLane, "_ams_td1_capture_patched", False):
            return

        def _patched_prep_capture_td1(lane_self):
            if (
                getattr(lane_self.unit_obj, "type", None) == "OpenAMS"
                and hasattr(lane_self.unit_obj, "prep_capture_td1")
            ):
                lane_self.unit_obj.prep_capture_td1(lane_self)
                return
            if _ORIGINAL_PREP_CAPTURE_TD1 is not None:
                return _ORIGINAL_PREP_CAPTURE_TD1(lane_self)
            return None

        def _patched_get_td1_data(lane_self):
            if (
                getattr(lane_self.unit_obj, "type", None) == "OpenAMS"
                and hasattr(lane_self.unit_obj, "capture_td1_data")
            ):
                return lane_self.unit_obj.capture_td1_data(lane_self)
            if _ORIGINAL_GET_TD1_DATA is not None:
                return _ORIGINAL_GET_TD1_DATA(lane_self)
            return False, "TD-1 capture not available"

        AFCLane._prep_capture_td1 = _patched_prep_capture_td1
        AFCLane.get_td1_data = _patched_get_td1_data
        AFCLane._ams_td1_capture_patched = True

    def _patch_td1_prep(self):
        if getattr(afcPrep, "_openams_td1_prep_patched", False):
            return

        def _openams_td1_prep(prep_self, overrall_status):
            capture_td1_data = prep_self.get_td1_data and prep_self.afc.td1_present
            any_td1_error = False
            if prep_self.afc.td1_present:
                prep_self.logger.info("Found TD-1 device connected to printer")
                any_td1_error, _ = prep_self.afc.function.check_for_td1_error()

            current_lane = prep_self.afc.function.get_current_lane_obj()
            if current_lane is not None:
                current_lane.unit_obj.select_lane(current_lane)
                if capture_td1_data:
                    prep_self.logger.info("Cannot capture TD-1 data during PREP since toolhead is loaded")
            elif capture_td1_data:
                if not overrall_status:
                    prep_self.logger.info("Cannot capture TD-1 data, not all of PREP succeeded")
                else:
                    if any_td1_error:
                        prep_self.logger.error("Error with a TD-1 device, not collecting data during prep")
                    else:
                        prep_self.logger.info("Capturing TD-1 data for all loaded lanes")
                        for lane in prep_self.afc.lanes.values():
                            prep_ready = lane.prep_state
                            if getattr(lane.unit_obj, "type", None) == "OpenAMS":
                                prep_ready = lane.load_state
                            if lane.td1_device_id and lane.load_state and prep_ready:
                                return_status, _ = lane.get_td1_data()
                                if not return_status:
                                    break
                        prep_self.logger.info("Done capturing TD-1 data")

        if _ORIGINAL_TD1_PREP is None:
            return

        afcPrep._td1_prep = _openams_td1_prep
        afcPrep._openams_td1_prep_patched = True

    def _patch_td1_cali_fail_prompt(self):
        afc_function = getattr(self.afc, "function", None)
        if afc_function is None:
            return
        if getattr(afc_function, "_openams_cali_fail_patched", False):
            return

        original_cmd = getattr(afc_function, "cmd_AFC_CALI_FAIL", None)
        if original_cmd is None:
            return

        def _openams_cmd_AFC_CALI_FAIL(afc_func_self, gcmd):
            cali = gcmd.get("FAIL", None)
            title = gcmd.get("TITLE", "AFC Calibration Failed")
            reset_lane = bool(gcmd.get_int("RESET", 1))

            lane = None
            if cali is not None:
                lane = afc_func_self.afc.lanes.get(str(cali))

            if (
                reset_lane
                and lane is not None
                and getattr(lane.unit_obj, "type", None) == "OpenAMS"
                and "TD-1" in title
            ):
                fail_message = gcmd.get("MSG", "")
                prompt = AFCprompt(gcmd, afc_func_self.logger)
                buttons = []
                footer = []
                text = f"{title} for {cali}. "
                fps_id = None
                if lane is not None:
                    fps_id = lane.unit_obj._get_fps_id_for_lane(lane.name)
                if fps_id:
                    text += "First: reset lane, Second: review messages in console and take necessary action and re-run calibration."
                    buttons.append(
                        (
                            "Reset lane",
                            f"OAMSM_UNLOAD_FILAMENT FPS={fps_id}",
                            "primary",
                        )
                    )
                if fail_message:
                    text += f" Fail message: {fail_message}"
                footer.append(("EXIT", "prompt_end", "info"))
                prompt.create_custom_p(title, text, buttons, True, None, footer)
                return

            return original_cmd(gcmd)

        afc_function.cmd_AFC_CALI_FAIL = MethodType(
            _openams_cmd_AFC_CALI_FAIL,
            afc_function,
        )
        afc_function._openams_cali_fail_patched = True

    def _get_fps_id_for_lane(self, lane_name: str) -> Optional[str]:
        oams_manager = self.printer.lookup_object("oams_manager", None)
        if oams_manager is None:
            return None
        fps_name = oams_manager.get_fps_for_afc_lane(lane_name)
        if not fps_name:
            return None
        return fps_name.split(" ", 1)[1] if fps_name.startswith("fps ") else fps_name

    def _format_openams_calibration_command(self, base_command, lane):
        if base_command not in {"OAMS_CALIBRATE_HUB_HES", "OAMS_CALIBRATE_PTFE_LENGTH"}:
            return super()._format_openams_calibration_command(base_command, lane)

        oams_index = self._get_openams_index()
        spool_index = self._get_openams_spool_index(lane)

        if oams_index is None or spool_index is None:
            lane_name = getattr(lane, "name", lane)
            self.logger.warning(f"Unable to format OpenAMS calibration command for lane {lane_name} on unit {self.name}")
            return None

        if base_command == "OAMS_CALIBRATE_HUB_HES":
            return f"AFC_OAMS_CALIBRATE_HUB_HES UNIT={self.name} SPOOL={spool_index}"

        return f"AFC_OAMS_CALIBRATE_PTFE UNIT={self.name} SPOOL={spool_index}"

    def cmd_UNIT_CALIBRATION(self, gcmd):
        """Override base calibration menu to show OpenAMS-specific options."""
        if not self._is_openams_unit():
            super().cmd_UNIT_CALIBRATION(gcmd)
            return

        prompt = AFCprompt(gcmd, self.logger)
        title = f"{self.name} Calibration"
        text = "Select OpenAMS calibration type"
        buttons = []

        # HUB HES calibration button
        buttons.append(("Calibrate HUB HES", f"UNIT_LANE_CALIBRATION UNIT={self.name}", "primary"))

        # PTFE calibration button
        buttons.append(("Calibrate PTFE Length", f"UNIT_PTFE_CALIBRATION UNIT={self.name}", "secondary"))

        any_lane_has_td1_ids = any(lane.td1_device_id for lane in self.lanes.values())
        if self.afc.td1_defined and (self.td1_device_id or any_lane_has_td1_ids):
            buttons.append(("Calibrate TD-1 Length", f"AFC_UNIT_TD_ONE_CALIBRATION UNIT={self.name}", "secondary"))

        # Back button
        back = [("Back", "AFC_CALIBRATION", "info")]

        prompt.create_custom_p(title, text, None, True, [buttons], back)

    def cmd_UNIT_PTFE_CALIBRATION(self, gcmd):
        """Show PTFE calibration menu with buttons for each loaded lane."""
        if not self._is_openams_unit():
            gcmd.respond_info("PTFE calibration is only available for OpenAMS units.")
            return

        # Check if any lane on THIS UNIT is loaded to toolhead
        for lane in self.lanes.values():
            if getattr(lane, "tool_loaded", False):
                gcmd.respond_info(f"Cannot run OpenAMS calibration while {lane.name} is loaded to the toolhead. Please unload the tool and try again.")
                return

        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"{self.name} PTFE Length Calibration"
        text = (
            "Select a loaded lane from {} to calibrate PTFE length using OpenAMS. "
            "Command: OAMS_CALIBRATE_PTFE_LENGTH"
        ).format(self.name)

        for lane in self.lanes.values():
            if not getattr(lane, "load_state", False):
                continue

            button_command = self._format_openams_calibration_command(
                "OAMS_CALIBRATE_PTFE_LENGTH", lane
            )
            if button_command is None:
                continue

            button_label = f"{lane}"
            button_style = "primary" if index % 2 == 0 else "secondary"
            group_buttons.append((button_label, button_command, button_style))

            index += 1
            if index % 2 == 0:
                buttons.append(list(group_buttons))
                group_buttons = []

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"

        back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]

        prompt.create_custom_p(title, text, None, True, buttons, back)

    def cmd_UNIT_LANE_CALIBRATION(self, gcmd):
        """Override base prompt to expose an all-lane HUB HES calibration action."""
        if not self._is_openams_unit():
            super().cmd_UNIT_LANE_CALIBRATION(gcmd)
            return

        # Check if any lane on THIS UNIT is loaded to toolhead
        for lane in self.lanes.values():
            if getattr(lane, "tool_loaded", False):
                gcmd.respond_info(f"Cannot run OpenAMS calibration while {lane.name} is loaded to the toolhead. Please unload the tool and try again.")
                return

        prompt = AFCprompt(gcmd, self.logger)
        buttons = []
        group_buttons = []
        index = 0
        title = f"{self.name} Lane Calibration"
        text = (
            "Select a loaded lane from {} to calibrate HUB HES using OpenAMS. "
            "Command: OAMS_CALIBRATE_HUB_HES"
        ).format(self.name)

        for lane in self.lanes.values():
            if not getattr(lane, "load_state", False):
                continue

            button_command = self._format_openams_calibration_command(
                "OAMS_CALIBRATE_HUB_HES", lane
            )
            if button_command is None:
                continue

            button_label = f"{lane}"
            button_style = "primary" if index % 2 == 0 else "secondary"
            group_buttons.append((button_label, button_command, button_style))

            index += 1
            if index % 2 == 0:
                buttons.append(list(group_buttons))
                group_buttons = []

        if group_buttons:
            buttons.append(list(group_buttons))

        total_buttons = sum(len(group) for group in buttons)
        if total_buttons == 0:
            text = "No lanes are loaded, please load before calibration"

        all_lanes = None
        if total_buttons > 1:
            all_lanes = [
                (
                    "Calibrate All HUB HES",
                    f"AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT={self.name}",
                    "default",
                )
            ]

        back = [("Back", f"UNIT_CALIBRATION UNIT={self.name}", "info")]

        prompt.create_custom_p(title, text, all_lanes, True, buttons, back)

    def handle_connect(self):
        """Initialise the AMS unit and configure custom logos."""
        super().handle_connect()
        # OPTIMIZATION: Pre-warm object caches for faster runtime access
        if self._cached_gcode is None:
            try:
                self._cached_gcode = self.printer.lookup_object("gcode")
            except Exception:
                pass

        # Pre-cache OAMS index
        if self.oams is not None and self._cached_oams_index is None:
            self._cached_oams_index = getattr(self.oams, "oams_idx", None)

        self._ensure_virtual_tool_sensor()

        # CRITICAL: Ensure all AMS lanes have buffer_obj = None
        # AMS units don't have physical buffers - this prevents buffer monitoring
        # from running even if users accidentally configure buffers at lane level
        for lane in self.lanes.values():
            if lane.buffer_obj is not None:
                self.logger.warning(
                    f"Lane {lane.name} had buffer '{lane.buffer_obj.name}' configured, "
                    f"but OpenAMS units don't have physical buffers. Removing buffer assignment."
                )
            lane.buffer_obj = None

        #  Register each lane with the shared registry
        for lane in self.lanes.values():
            lane.prep_state = False
            lane.load_state = False
            lane.status = AFCLaneState.NONE
            lane.ams_share_prep_load = getattr(lane, "load", None) is None

            idx = getattr(lane, "index", 0) - 1
            if idx >= 0 and self.registry is not None:
                lane_name = getattr(lane, "name", None)
                unit_name = self.oams_name or self.name
                extruder_name = getattr(lane, "extruder_name", None) or getattr(self, "extruder", None)

                if lane_name and extruder_name:
                    try:
                        self.registry.register_lane(
                            lane_name=lane_name,
                            unit_name=unit_name,
                            spool_index=idx,
                            extruder=extruder_name,
                            fps_name=None,
                            hub_name=getattr(lane, "hub", None),
                            led_index=getattr(lane, "led_index", None),
                            custom_load_cmd=getattr(lane, "custom_load_cmd", None),
                            custom_unload_cmd=getattr(lane, "custom_unload_cmd", None),
                        )
                    except Exception:
                        self.logger.error(f"Failed to register lane {lane_name} with registry")
        first_leg = ("<span class=warning--text>|</span>"
                    "<span class=error--text>_</span>")
        second_leg = f"{first_leg}<span class=warning--text>|</span>"
        self.logo = dedent("""\
            <span class=success--text>R  _____     ____
            E /      \\  |  </span><span class=info--text>o</span><span class=success--text> |
            A |       |/ ___/
            D |_________/
            Y {first}{second} {first}{second}
              {name}
            </span>
            """).format(first=first_leg, second=second_leg, name=self.name)

        self.logo_error = dedent("""\
            <span class=error--text>E  _ _   _ _
            R |_|_|_|_|_|
            R |         \\____
            O |              \\
            R |          |\\ <span class=secondary--text>X</span> |
            ! \\_________/ |___|
              {name}
            </span>
            """).format(name=self.name)

    def _ensure_virtual_tool_sensor(self) -> bool:
        """Resolve or create the virtual tool-start sensor for AMS extruders."""
        if self._virtual_tool_sensor is not None:
            return True

        extruder = getattr(self, "extruder_obj", None)
        if extruder is None:
            return False

        tool_pin = getattr(extruder, "tool_start", None)
        normalized = _normalize_ams_pin_value(tool_pin)
        if normalized is None:
            normalized = getattr(extruder, "_ams_virtual_tool_name", None)

        if not normalized or normalized.lower() in {"buffer", "none", "unknown"}:
            return False

        original_pin = tool_pin
        if not normalized.upper().startswith("AMS_"):
            return False

        sensor = getattr(extruder, "fila_tool_start", None)
        if sensor is None:
            sensor = self.printer.lookup_object(f"filament_switch_sensor {normalized}", None)

        if sensor is None:
            # Use lookup with None to prevent errors if pins not yet loaded
            pins = self.printer.lookup_object("pins", None)
            if pins is None:
                return False
            if not getattr(self.afc, "_virtual_ams_chip_registered", False):
                try:
                    pins.register_chip("afc_virtual_ams", self.afc)
                except Exception:
                    return False
                else:
                    self.afc._virtual_ams_chip_registered = True

            enable_gui = getattr(extruder, "enable_sensors_in_gui", True)
            runout_cb = getattr(extruder, "handle_start_runout", None)
            enable_runout = getattr(extruder, "enable_runout", False)
            debounce = getattr(extruder, "debounce_delay", 0.0)

            try:
                created = add_filament_switch(normalized, f"afc_virtual_ams:{normalized}", self.printer, enable_gui, runout_cb, enable_runout, debounce)
            except TypeError:
                try:
                    created = add_filament_switch(normalized, f"afc_virtual_ams:{normalized}", self.printer, enable_gui)
                except Exception:
                    return False
            except Exception:
                return False

            sensor = created[0] if isinstance(created, tuple) else created

        helper = getattr(sensor, "runout_helper", None)
        if helper is None:
            return False

        helper.runout_callback = None
        helper.sensor_enabled = False

        filament_present = getattr(helper, "filament_present", None)

        if getattr(extruder, "fila_tool_start", None) is None:
            extruder.fila_tool_start = sensor

        extruder.tool_start = original_pin
        self._virtual_tool_sensor = sensor
        
        # OPTIMIZATION: Cache the sensor helper
        self._cached_sensor_helper = helper

        alias_token = None
        try:
            alias_token = f"{extruder.name}_tool_start"
        except Exception:
            alias_token = None

        if alias_token:
            alias_object = f"filament_switch_sensor {alias_token}"
            objects = getattr(self.printer, "objects", None)
            if isinstance(objects, dict):
                previous = objects.get(alias_object)
                if previous is None or previous is sensor:
                    objects[alias_object] = sensor

            # OPTIMIZATION: Use cached gcode object
            gcode = self._cached_gcode
            if gcode is None:
                try:
                    gcode = self.printer.lookup_object("gcode")
                    self._cached_gcode = gcode
                except Exception:
                    gcode = None

            if gcode is not None:
                for command, handler, desc in (
                    ("QUERY_FILAMENT_SENSOR", sensor.cmd_QUERY_FILAMENT_SENSOR, sensor.QUERY_HELP),
                    ("SET_FILAMENT_SENSOR", sensor.cmd_SET_FILAMENT_SENSOR, sensor.SET_HELP),
                ):
                    try:
                        gcode.register_mux_command(command, "SENSOR", alias_token, handler, desc=desc)
                    except Exception:
                        pass

        return True

    def _lane_matches_extruder(self, lane) -> bool:
        """Return True if the lane is mapped to this AMS unit's extruder."""
        extruder_name = getattr(self, "extruder", None)
        unit_extruder_obj = getattr(self, "extruder_obj", None)
        if not extruder_name:
            return False

        lane_extruder = getattr(lane, "extruder_name", None)
        if lane_extruder is None:
            lane_extruder_obj = getattr(lane, "extruder_obj", None)
            lane_extruder = getattr(lane_extruder_obj, "name", None)
        else:
            lane_extruder_obj = getattr(lane, "extruder_obj", None)

        if unit_extruder_obj is not None and lane_extruder_obj is unit_extruder_obj:
            return True

        if lane_extruder == extruder_name:
            return True

        normalized_lane = _normalize_extruder_name(lane_extruder)
        normalized_unit = _normalize_extruder_name(extruder_name)

        if normalized_lane and normalized_unit and normalized_lane == normalized_unit:
            return True

        return False

    def _lane_reports_tool_filament(self, lane, sync_only: bool = False) -> Optional[bool]:
        """Return the best-known tool filament state for a lane.

        Args:
            lane: The lane to check
            sync_only: If True, only trust extruder.lane_loaded (for post-reboot sync).
                      If False, fall through to lane state for in-progress loads.

        Checks if the EXTRUDER thinks this lane is loaded, not just if the
        lane thinks it's loaded (to avoid stale state after reboot).
        """
        if lane is None:
            return None

        lane_name = getattr(lane, "name", None)
        lane_unit = getattr(lane, "unit", None)

        # Only apply AMS virtual sensor logic to lanes that belong to any OpenAMS unit
        if lane_unit:
            sync_units = self.__class__._sync_instances
            if lane_unit not in sync_units:
                return None
            if lane_unit != getattr(self, "name", None):
                return None

        # Check if the extruder thinks THIS lane is loaded (authoritative)
        extruder = getattr(lane, "extruder_obj", None)
        if extruder is not None:
            lane_loaded = getattr(extruder, "lane_loaded", None)
            if lane_loaded == lane_name:
                # Extruder confirms this lane is loaded
                return True
            elif lane_loaded is not None and lane_loaded != lane_name:
                # Extruder has a different lane loaded
                return False
            elif lane_loaded is None:
                # Extruder lost track (e.g., during BT_PREP system test). Rehydrate from saved state, then lane state
                lane_has_filament = bool(getattr(lane, "tool_loaded", False) and getattr(lane, "load_state", False))
                if not lane_has_filament:
                    # Fall back to live OAMS sensors for this lane to avoid toolchanger prep clearing LEDs
                    try:
                        sensor_snapshot = self._get_oams_sensor_snapshot({lane_name: lane}, require_hub=True)
                        lane_has_filament = bool(sensor_snapshot.get(lane_name, False) and getattr(lane, "tool_loaded", False))
                    except Exception:
                        lane_has_filament = False

                saved_lane_loaded = self._get_saved_extruder_lane_loaded(getattr(extruder, "name", None))
                canonical_saved = self._canonical_lane_name(saved_lane_loaded)
                canonical_lane = self._canonical_lane_name(lane_name)

                if lane_has_filament and canonical_saved is not None:
                    if canonical_saved == canonical_lane:
                        try:
                            extruder.lane_loaded = lane_name
                            self.logger.debug(f"Restored extruder.lane_loaded to {lane_name} from AFC.var.unit during sync")
                            return True
                        except Exception:
                            self.logger.debug("Failed to restore extruder.lane_loaded from AFC.var.unit during sync")
                    else:
                        return False

                if lane_has_filament:
                    try:
                        extruder.lane_loaded = lane_name
                        self.logger.debug(f"Restored extruder.lane_loaded to {lane_name} based on lane state during sync")
                        return True
                    except Exception:
                        self.logger.debug("Failed to restore extruder.lane_loaded during sync")
                if sync_only:
                    # Post-reboot sync: extruder says nothing loaded, trust it over stale lane state
                    return False
            # else: lane_loaded is None but sync_only=False, fall through to check lane state
            # This allows in-progress loads to work (lane.load_state changes before lane_loaded)

        # Fallback: check lane's own state
        # During normal operation: reflects in-progress loads (hub sensor detects filament)
        # During sync_only: should not reach here (extruder is not None for AMS lanes)
        load_state = getattr(lane, "load_state", None)
        if load_state is not None:
            # Only trust load_state when lane.tool_loaded is also True.
            # Prevents stale saved load_state from re-marking lanes as loaded at startup.
            if getattr(lane, "tool_loaded", False):
                return bool(load_state)
            return False if sync_only else None

        if getattr(lane, "tool_loaded", False):
            return True

        return None

    def _set_virtual_tool_sensor_state(self, filament_present: bool, eventtime: float, lane_name: Optional[str] = None, *, force: bool = False, lane_obj=None) -> None:
        """Update the cached virtual sensor and extruder state (OPTIMIZED)."""
        if not self._ensure_virtual_tool_sensor():
            return

        new_state = bool(filament_present)

        canonical_lane = self._canonical_lane_name(lane_name)
        if canonical_lane is None and lane_obj is not None:
            canonical_lane = self._canonical_lane_name(getattr(lane_obj, "name", None))

        if new_state and not force:
            if lane_obj and getattr(lane_obj, "tool_loaded", False) is False:
                return

        #  Use cached sensor helper
        helper = self._cached_sensor_helper
        if helper is None:
            sensor = self._virtual_tool_sensor
            helper = getattr(sensor, "runout_helper", None)
            if helper is None:
                return
            self._cached_sensor_helper = helper

        try:
            helper.note_filament_present(eventtime, filament_present)
        except TypeError:
            helper.note_filament_present(is_filament_present=filament_present)

        sensor = self._virtual_tool_sensor
        setattr(sensor, "filament_present", filament_present)

        extruder = getattr(self, "extruder_obj", None)
        if extruder is not None:
            extruder.tool_start_state = filament_present

    def lane_tool_loaded(self, lane):
        """Update the virtual tool sensor when a lane loads into the tool."""
        super().lane_tool_loaded(lane)

        # When a new lane loads to toolhead, clear tool_loaded on any OTHER lanes from this unit
        # that are on the SAME FPS/extruder (each FPS can have its own lane loaded)
        # This handles cross-Extruder runout where AFC switches from OpenAMS lane to different Extruder/FPS lane
        lane_extruder = getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None
        if lane_extruder:
            for other_lane in self.lanes.values():
                if other_lane.name == lane.name:
                    continue
                if not getattr(other_lane, 'tool_loaded', False):
                    continue
                # Check if other lane is on same extruder/FPS
                other_extruder = getattr(other_lane.extruder_obj, "name", None) if hasattr(other_lane, "extruder_obj") else None

                if other_extruder == lane_extruder:
                    # PHASE 1 REFACTOR: Use AFC native set_tool_unloaded() instead of direct assignment
                    # This ensures proper state cleanup and callbacks
                    other_lane.set_tool_unloaded()
                    if hasattr(other_lane, '_oams_runout_detected'):
                        other_lane._oams_runout_detected = False
                    self.logger.debug(f"Cleared tool_loaded for {other_lane.name} on same FPS (new lane {lane.name} loaded)")

        # Trigger OAMS state detection to sync FPS state with sensor readings
        # This ensures FPS state is updated when manually setting lane as loaded
        # NOTE: Use determine_state() for normal operations, not sync_state_with_afc()
        # sync_state_with_afc() is only for error recovery (SET_LANE_LOADED, OAMSM_CLEAR_ERRORS)
        if self.oams is not None:
            try:
                oams_manager = self.printer.lookup_object("oams_manager", None)
                if oams_manager is not None:
                    oams_manager.determine_state()
            except Exception as e:
                self.logger.error(f"Failed to trigger OAMS state detection for {getattr(lane, 'name', None)}: {e}")

        if not self._lane_matches_extruder(lane):
            return

        # Wait for all moves to complete to prevent "Timer too close" errors
        try:
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.wait_moves()
            # Add a small delay to allow the MCU to catch up
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        except Exception:
            pass

        eventtime = self.reactor.monotonic()
        lane_name = getattr(lane, "name", None)
        self._set_virtual_tool_sensor_state(True, eventtime, lane_name, force=True, lane_obj=lane)

    def lane_tool_unloaded(self, lane):
        """Update the virtual tool sensor when a lane unloads from the tool."""
        super().lane_tool_unloaded(lane)

        # PHASE 1 REFACTOR: Removed redundant lane.tool_loaded = False
        # Parent's lane_tool_unloaded() already calls lane.set_tool_unloaded() which sets tool_loaded = False

        # Clear runout flag if set
        if hasattr(lane, '_oams_runout_detected'):
            lane._oams_runout_detected = False

        # NOTE: Don't call sync_state_with_afc() here - it interferes with normal unload
        # sync_state_with_afc() is only for error recovery (SET_LANE_LOADED, OAMSM_CLEAR_ERRORS)
        # Normal unload operations update state through AFC's standard callbacks

        if not self._lane_matches_extruder(lane):
            return

        # Wait for all moves to complete to prevent "Timer too close" errors
        try:
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.wait_moves()
            # Add a small delay to allow the MCU to catch up
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        except Exception:
            pass

        eventtime = self.reactor.monotonic()
        lane_name = getattr(lane, "name", None)
        self._set_virtual_tool_sensor_state(False, eventtime, lane_name, lane_obj=lane)

    def _mirror_lane_to_virtual_sensor(self, lane, eventtime: float) -> None:
        """Mirror a lane's load state into the AMS virtual tool sensor."""
        if not self._lane_matches_extruder(lane):
            return

        desired_state = self._lane_reports_tool_filament(lane)
        if desired_state is None:
            return

        if desired_state == getattr(lane, "tool_loaded", False):
            return

        lane_name = getattr(lane, "name", None)
        self._set_virtual_tool_sensor_state(desired_state, eventtime, lane_name, lane_obj=lane)

    def _sync_virtual_tool_sensor(self, eventtime: float, lane_name: Optional[str] = None, force: bool = False) -> None:
        """Align the AMS virtual tool sensor with the mapped lane state.

        Args:
            eventtime: Current event time
            lane_name: Specific lane to check, or None to check all lanes
            force: If True, update sensor even if state hasn't changed (for post-reboot sync)
        """
        if not self._ensure_virtual_tool_sensor():
            return

        desired_state: Optional[bool] = None
        desired_lane: Optional[str] = None
        desired_lane_obj = None

        if lane_name:
            lane = self.lanes.get(lane_name)
            if lane is not None and self._lane_matches_extruder(lane):
                result = self._lane_reports_tool_filament(lane, sync_only=force)
                if result is not None:
                    desired_state = result
                    desired_lane = getattr(lane, "name", None)
                    desired_lane_obj = lane

        if desired_state is None:
            pending_false = None
            matching_lanes = 0
            for lane in self.lanes.values():
                if not self._lane_matches_extruder(lane):
                    continue

                matching_lanes += 1
                result = self._lane_reports_tool_filament(lane, sync_only=force)
                if result is None:
                    continue

                lane_id = getattr(lane, "name", None)
                if result:
                    desired_state = True
                    desired_lane = lane_id
                    desired_lane_obj = lane
                    break

                if pending_false is None:
                    pending_false = (False, lane_id, lane)

            if desired_state is None and pending_false is not None:
                desired_state, desired_lane, desired_lane_obj = pending_false

        # Skip update only if state matches AND not forcing
        if desired_state is None or (not force and desired_state == getattr(desired_lane_obj, "tool_loaded", False) if desired_lane_obj else False):
            return

        self._set_virtual_tool_sensor_state(desired_state, eventtime, desired_lane, lane_obj=desired_lane_obj)

    def _unit_matches(self, unit_value: Optional[str]) -> bool:
        """Return True when a mux UNIT value targets this AMS instance."""
        if not unit_value:
            return True

        normalized = unit_value.strip().strip('"').strip("'")
        if not normalized:
            return True

        if normalized == self.name:
            return True

        lowered = normalized.lower()
        if lowered == self.name.lower():
            return True

        parts = normalized.replace("_", " ").replace("-", " ").split()
        return any(part.lower() == self.name.lower() for part in parts)

    def _normalize_lane_alias(self, alias: Optional[str]) -> Optional[str]:
        """Return a trimmed lane alias token for comparison."""
        if not alias or not isinstance(alias, str):
            return None

        normalized = alias.strip()
        if not normalized:
            return None

        if " " in normalized:
            normalized = normalized.split()[-1]

        return normalized

    def _resolve_lane_alias(self, identifier: Optional[str]) -> Optional[str]:
        """Map common aliases (fps names, case variants) to lane objects."""
        if not identifier:
            return None

        lookup = identifier.strip()
        if not lookup:
            return None

        lane = self.lanes.get(lookup)
        if lane is not None:
            return lane.name

        lowered = lookup.lower()
        normalized_lookup = self._normalize_lane_alias(lookup)
        for lane in self.lanes.values():
            if lane.name.lower() == lowered:
                return lane.name

            lane_map = getattr(lane, "map", None)
            if isinstance(lane_map, str) and lane_map.lower() == lowered:
                return lane.name

            canonical_map = self._normalize_lane_alias(lane_map)
            if (canonical_map is not None and normalized_lookup is not None and canonical_map.lower() == normalized_lookup.lower()):
                return lane.name

        return None

    def _canonical_lane_name(self, lane_name: Optional[str]) -> Optional[str]:
        """Return a consistent lane identifier for latch/feed tracking."""
        if lane_name is None:
            return None

        lookup = lane_name.strip() if isinstance(lane_name, str) else str(lane_name).strip()
        if not lookup:
            return None

        resolved = self._resolve_lane_alias(lookup)
        if resolved:
            return resolved

        return lookup

    def _get_extruder_object(self, extruder_name: Optional[str]):
        # OPTIMIZATION: Cache extruder object lookups
        if not extruder_name:
            return None

        # Check cache first
        cached = self._cached_extruder_objects.get(extruder_name)
        if cached is not None:
            return cached

        key = f"AFC_extruder {extruder_name}"
        lookup = getattr(self.printer, "lookup_object", None)
        extruder = None
        if callable(lookup):
            try:
                extruder = lookup(key, None)
            except Exception:
                extruder = None

        if extruder is None:
            objects = getattr(self.printer, "objects", None)
            if isinstance(objects, dict):
                extruder = objects.get(key)

        # Cache result (even if None to avoid repeated lookups)
        if extruder is not None:
            self._cached_extruder_objects[extruder_name] = extruder

        return extruder

    def _current_lane_for_extruder(self, extruder_name: Optional[str]) -> Optional[str]:
        extruder = self._get_extruder_object(extruder_name)
        lane_name = getattr(extruder, "lane_loaded", None) if extruder else None
        return self._canonical_lane_name(lane_name)

    def _get_lane_object(self, lane_name: Optional[str]):
        # OPTIMIZATION: Cache lane object lookups
        canonical = self._canonical_lane_name(lane_name)
        if canonical is None:
            return None

        # Check local lanes dict first
        lane = self.lanes.get(canonical)
        if lane is not None:
            return lane

        # Check cache
        cached = self._cached_lane_objects.get(canonical)
        if cached is not None:
            return cached

        key = f"AFC_lane {canonical}"
        lookup = getattr(self.printer, "lookup_object", None)
        if callable(lookup):
            try:
                lane = lookup(key, None)
            except Exception:
                lane = None
        else:
            lane = None

        if lane is None:
            objects = getattr(self.printer, "objects", None)
            if isinstance(objects, dict):
                lane = objects.get(key)

        # Cache result if found
        if lane is not None:
            self._cached_lane_objects[canonical] = lane

        return lane

    def _saved_unit_file_path(self) -> Optional[str]:
        afc = getattr(self, "afc", None)
        base_path = getattr(afc, "VarFile", None)
        if not base_path:
            return None

        return os.path.expanduser(str(base_path) + ".unit")

    def _load_saved_unit_snapshot(self) -> Optional[Dict[str, Any]]:
        filename = self._saved_unit_file_path()
        if not filename:
            return None

        try:
            mtime = os.path.getmtime(filename)
        except OSError:
            self._saved_unit_cache = None
            self._saved_unit_mtime = None
            return None

        if self._saved_unit_cache is not None and self._saved_unit_mtime == mtime:
            return self._saved_unit_cache

        try:
            with open(filename, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception:
            self.logger.debug(f"Failed to read saved AFC unit data from {filename}")
            self._saved_unit_cache = None
        else:
            self._saved_unit_cache = data if isinstance(data, dict) else None

        self._saved_unit_mtime = mtime
        return self._saved_unit_cache

    def _get_unit_snapshot(self) -> Optional[Dict[str, Any]]:
        """Get AFC.var.unit snapshot from file.

        AFC doesn't have a .var object attribute - it saves to VarFile + '.unit' file.
        Always load from the saved file which is kept up-to-date by AFC's save_vars().
        """
        return self._load_saved_unit_snapshot()

    def _find_lane_snapshot(self, lane_name: Optional[str], unit_name: Optional[str] = None) -> Optional[Dict[str, Any]]:
        canonical = self._canonical_lane_name(lane_name)
        if canonical is None:
            return None

        snapshot = self._get_unit_snapshot()
        if not isinstance(snapshot, dict):
            return None

        unit_key = unit_name
        if isinstance(unit_key, str) and ":" in unit_key:
            unit_key = unit_key.split(":", 1)[0]

        if unit_key:
            unit_data = snapshot.get(unit_key)
            if isinstance(unit_data, dict):
                lane_data = unit_data.get(canonical)
                if isinstance(lane_data, dict):
                    return lane_data

        for unit_key, unit_data in snapshot.items():
            if unit_key in ("system", "Tools") or not isinstance(unit_data, dict):
                continue
            lane_data = unit_data.get(canonical)
            if isinstance(lane_data, dict):
                return lane_data

        return None

    def _get_saved_lane_field(self, lane_name: Optional[str], field: str) -> Optional[Any]:
        """Fetch a field for a lane from the AFC.var.unit snapshot."""

        unit_key = None
        lane_obj = self._get_lane_object(lane_name)
        if lane_obj is not None:
            unit_key = getattr(lane_obj, "unit", None)
        if unit_key is None:
            unit_key = getattr(self, "name", None)
        lane_data = self._find_lane_snapshot(lane_name, unit_name=unit_key)
        if not isinstance(lane_data, dict):
            return None

        return lane_data.get(field)

    def _get_saved_extruder_lane_loaded(self, extruder_name: Optional[str]) -> Optional[str]:
        """Return the persisted lane_loaded value for an extruder from AFC.var.unit."""
        if not extruder_name:
            return None

        snapshot = self._get_unit_snapshot()
        system = snapshot.get("system") if isinstance(snapshot, dict) else None
        if not isinstance(system, dict):
            return None

        extruders = system.get("extruders")
        if not isinstance(extruders, dict):
            return None

        data = extruders.get(extruder_name)
        if not isinstance(data, dict):
            return None

        return data.get("lane_loaded")

    def _get_openams_unit_names(self) -> Set[str]:
        """Return the set of unit names that correspond to OpenAMS units."""
        units: Set[str] = set(self.__class__._sync_instances.keys())

        afc = getattr(self, "afc", None)
        afc_units = getattr(afc, "units", {}) if afc else {}
        for unit_name, unit_obj in afc_units.items():
            if isinstance(unit_obj, afcAMS) or getattr(unit_obj, "type", "") == "OpenAMS":
                units.add(getattr(unit_obj, "name", unit_name))

        return {unit for unit in units if unit}

    def _get_oams_sensor_snapshot(self, lanes: Dict[str, Any], *, require_hub: bool = False) -> Dict[str, bool]:
        """Return a lane->filament-present map using live OAMS sensor data.

        Args:
            lanes: Lane objects keyed by lane name.
            require_hub: If True, only report filament present when the hub sensor is high.
                         Otherwise, hub or f1s sensors may assert filament presence.
        """
        snapshot: Dict[str, bool] = {}
        try:
            oams_manager = self.printer.lookup_object("oams_manager", None)
        except Exception:
            oams_manager = None

        if oams_manager is None:
            return snapshot

        oams_map = getattr(oams_manager, "oams", {}) if oams_manager else {}
        for lane_obj in lanes.values():
            lane_name = getattr(lane_obj, "name", None)
            lane_index = getattr(lane_obj, "index", None)
            lane_unit = getattr(lane_obj, "unit", None)
            if lane_name is None or lane_index is None or lane_unit is None:
                continue

            base_unit_name = lane_unit.split(":", 1)[0]
            unit_obj = getattr(lane_obj, "unit_obj", None)
            if unit_obj is None:
                afc = getattr(self, "afc", None)
                units = getattr(afc, "units", {}) if afc else {}
                unit_obj = units.get(base_unit_name)

            if unit_obj is None or (not isinstance(unit_obj, afcAMS) and getattr(unit_obj, "type", "") != "OpenAMS"):
                continue

            oams_name = getattr(unit_obj, "oams_name", None)
            if not oams_name:
                continue

            oam = oams_map.get(oams_name) or oams_map.get(f"oams {oams_name}")
            if oam is None:
                continue

            bay_index = int(lane_index) - 1
            if bay_index < 0:
                continue

            f1s_values = getattr(oam, "f1s_hes_value", None)
            hub_values = getattr(oam, "hub_hes_value", None)

            hub_present = False
            f1s_present = False
            try:
                if hub_values and bay_index < len(hub_values):
                    hub_present = bool(hub_values[bay_index])
                if f1s_values and bay_index < len(f1s_values):
                    f1s_present = bool(f1s_values[bay_index])
            except Exception:
                pass

            has_filament = bool(hub_present or (f1s_present and not require_hub))
            snapshot[lane_name] = has_filament

        return snapshot

    def _hydrate_from_saved_state(self) -> None:
        """Restore extruder.lane_loaded from saved state when sensors confirm filament at toolhead."""
        if getattr(self, "_hydrated_from_saved", False):
            return

        afc = getattr(self, "afc", None)
        if afc is None:
            return

        lanes = getattr(afc, "lanes", {})
        tools = getattr(afc, "tools", {})

        openams_units = self._get_openams_unit_names()
        if not openams_units:
            return

        sensor_snapshot = self._get_oams_sensor_snapshot(lanes, require_hub=True)

        for extruder_name, extruder_obj in tools.items():
            if extruder_name in self.__class__._hydrated_extruders:
                continue

            saved_lane = self._get_saved_extruder_lane_loaded(extruder_name)
            current_lane = getattr(extruder_obj, "lane_loaded", None)

            lane_obj = None
            canonical_lane = None
            lane_unit = None

            for candidate in (current_lane, saved_lane):
                canonical = self._canonical_lane_name(candidate)
                if not canonical:
                    continue

                lane_obj = lanes.get(candidate) or lanes.get(canonical)
                if lane_obj is not None:
                    lane_unit = getattr(lane_obj, "unit", None) or self._get_saved_lane_field(canonical, "unit")
                    canonical_lane = getattr(lane_obj, "name", None) or canonical
                    break

                lane_unit = self._get_saved_lane_field(canonical, "unit")
                if lane_unit:
                    canonical_lane = canonical
                    break

            if not canonical_lane:
                continue

            if lane_unit and lane_unit not in openams_units:
                continue

            unit_obj = getattr(lane_obj, "unit_obj", None)
            if unit_obj is not None and not isinstance(unit_obj, afcAMS):
                continue

            hub_reports_filament = bool(sensor_snapshot.get(canonical_lane, False))
            lane_filament_present = False
            if lane_obj is not None:
                lane_filament_present = bool(getattr(lane_obj, "tool_loaded", False) and getattr(lane_obj, "load_state", False) and hub_reports_filament)
            else:
                saved_tool_loaded = bool(self._get_saved_lane_field(canonical_lane, "tool_loaded"))
                saved_load_state = bool(self._get_saved_lane_field(canonical_lane, "load_state"))
                lane_filament_present = bool(hub_reports_filament and saved_tool_loaded and saved_load_state)

            # Only hydrate when sensors (virtual or hardware) indicate filament at toolhead
            if not lane_filament_present:
                continue

            try:
                extruder_obj.lane_loaded = canonical_lane
                self.__class__._hydrated_extruders.add(extruder_name)
                self.logger.debug(f"Hydrated {extruder_name}.lane_loaded from saved state: {canonical_lane}")
            except Exception:
                self.logger.debug(f"Failed to hydrate {extruder_name} lane from saved state")

        self._hydrated_from_saved = True

    def _get_saved_lane_runout_target(self, lane_name: Optional[str]) -> Optional[str]:
        """Return the saved runout target for a lane from AFC.var.unit, if available."""

        saved_value = self._get_saved_lane_field(lane_name, "runout_lane")
        return saved_value

    def record_load(self, extruder: Optional[str] = None, lane_name: Optional[str] = None) -> Optional[str]:
        extruder_name = extruder or getattr(self, "extruder", None)
        canonical = self._canonical_lane_name(lane_name)

        return canonical

    def get_last_loaded_lane(self, extruder: Optional[str] = None) -> Optional[str]:
        extruder_name = extruder or getattr(self, "extruder", None)
        if extruder_name is None:
            return None

        extruder_obj = self.afc.extruders.get(extruder_name) if hasattr(self, "afc") else None
        return getattr(extruder_obj, "lane_loaded", None) if extruder_obj else None

    cmd_SYNC_TOOL_SENSOR_help = "Synchronise the AMS virtual tool-start sensor with the assigned lane."
    def cmd_SYNC_TOOL_SENSOR(self, gcmd):
        cls = self.__class__

        unit_value = gcmd.get("UNIT", None)
        if not unit_value:
            unit_value = cls._extract_raw_param(gcmd.get_commandline(), "UNIT")

        lane_name = gcmd.get("LANE", None)
        if lane_name is None:
            lane_name = gcmd.get("FPS", None)

        if lane_name is None:
            commandline = gcmd.get_commandline()
            lane_name = cls._extract_raw_param(commandline, "LANE")
            if lane_name is None:
                lane_name = cls._extract_raw_param(commandline, "FPS")

        for instance in cls._sync_instances.values():
            if not instance._unit_matches(unit_value):
                continue

            resolved_lane = instance._resolve_lane_alias(lane_name)
            eventtime = instance.reactor.monotonic()
            instance._sync_virtual_tool_sensor(eventtime, resolved_lane)

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """Validate AMS lane state without attempting any motion."""
        msg = ""
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                # BoxTurtle-style logic: lane isn't truly empty until hub is clear too
                # Check hub sensor before declaring empty (handles runout buffer scenarios)
                hub_loaded = cur_lane.hub_obj and cur_lane.hub_obj.state
                if not hub_loaded:
                    # Truly empty - both spool (F1S) and hub are clear
                    self.lane_not_ready(cur_lane)
                    msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
                else:
                    # Hub still has filament (runout in progress, repositioning, or residual)
                    self.lane_fault(cur_lane)
                    msg += '<span class=warning--text>Filament in hub</span>'
                    succeeded = False
            else:
                self.lane_fault(cur_lane)
                msg += '<span class=error--text> NOT READY</span>'
                cur_lane.do_enable(False)
                msg = '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                succeeded = False
        else:
            self.lane_loaded(cur_lane)
            msg += '<span class=success--text>LOCKED</span>'
            if not cur_lane.load_state:
                msg += '<span class=error--text> NOT LOADED</span>'
                self.lane_not_ready(cur_lane)
                succeeded = False
            else:
                cur_lane.status = AFCLaneState.LOADED
                msg += '<span class=success--text> AND LOADED</span>'
                self.lane_illuminate_spool(cur_lane)

                if cur_lane.tool_loaded:
                    tool_ready = (cur_lane.get_toolhead_pre_sensor_state() or cur_lane.extruder_obj.tool_start == "buffer" or cur_lane.extruder_obj.tool_end_state)
                    if tool_ready and cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                        cur_lane.sync_to_extruder()
                        on_shuttle = ""
                        try:
                            if cur_lane.extruder_obj.tool_obj and cur_lane.extruder_obj.tc_unit_name:
                                on_shuttle = " and toolhead on shuttle" if cur_lane.extruder_obj.on_shuttle() else ""
                        except Exception:
                            pass

                        msg += f"<span class=primary--text> in ToolHead{on_shuttle}</span>"
                        if cur_lane.extruder_obj.tool_start == "buffer":
                            msg += '<span class=warning--text> Ram sensor enabled, confirm tool is loaded</span>'
                        if self.afc.function.get_current_lane() == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            cur_lane.unit_obj.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED
                        else:
                            cur_lane.unit_obj.lane_tool_loaded_idle(cur_lane)
                    elif tool_ready:
                        msg += '<span class=error--text> error in ToolHead. Lane identified as loaded but not identified as loaded in extruder</span>'
                        succeeded = False

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info('{lane_name} tool cmd: {tcmd:3} {msg}'.format(lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg))
        cur_lane.set_afc_prep_done()

        # Trigger lane sync after PREP completes for this lane
        # The delay ensures hardware sensors have stabilized and all lanes have been tested
        try:
            oams_manager = self.printer.lookup_object("oams_manager", None)
            if oams_manager and hasattr(oams_manager, '_sync_all_fps_lanes_after_prep'):
                # 200ms delay allows all lanes to complete and hardware to stabilize
                self.afc.reactor.register_callback(
                    lambda et: oams_manager._sync_all_fps_lanes_after_prep(),
                    self.afc.reactor.monotonic() + 0.2
                )
        except Exception as e:
            self.logger.error(f"Failed to schedule post-PREP sync: {e}")

        return succeeded

    def calibrate_bowden(self, cur_lane, dis, tol):
        """OpenAMS units use different calibration commands."""
        msg = (
            "OpenAMS units do not support standard AFC bowden calibration. "
            "Use OpenAMS-specific calibration commands instead:\n"
            "  - AFC_OAMS_CALIBRATE_HUB_HES UNIT={} SPOOL=<spool_index>\n"
            "  - AFC_OAMS_CALIBRATE_PTFE UNIT={} SPOOL=<spool_index>\n"
            "  - AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT={}"
        ).format(self.name, self.name, self.name)
        self.logger.info(msg)
        return False, msg, 0

    def _unload_after_td1(self, cur_lane, spool_index, fps_id):
        """
        Unload filament after TD-1 operation by reversing follower and spool motor until hub clears.
        """
        # Send unload command first to retract spool motor
        try:
            self.oams.oams_unload_spool_cmd.send()
        except Exception:
            self.logger.error(f"Failed to send unload command for {cur_lane.name}")

        # Also reverse follower to help pull filament back
        try:
            self.oams.set_oams_follower(1, 0)  # Enable reverse
        except Exception:
            self.logger.error(f"Failed to enable reverse follower for {cur_lane.name}")

        # Wait for hub sensor to clear (timeout after 45 seconds)
        # Increased timeout since spool motor retraction can take longer
        hub_clear_timeout = self.afc.reactor.monotonic() + 45.0
        hub_cleared = False

        while self.afc.reactor.monotonic() < hub_clear_timeout:
            try:
                hub_loaded = bool(self.oams.hub_hes_value[spool_index])
            except Exception:
                hub_loaded = True  # Assume loaded if we can't read

            if not hub_loaded:
                hub_cleared = True
                self.logger.debug(f"Hub sensor cleared for {cur_lane.name}")
                break
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.3)

        # Disable follower after unload
        try:
            self.oams.set_oams_follower(0, 0)
        except Exception:
            self.logger.error(f"Failed to disable follower after unload for {cur_lane.name}")

        if not hub_cleared:
            self.logger.warning(f"Hub sensor did not clear after TD-1 unload for {cur_lane.name}")
        else:
            self.logger.info(f"TD-1 unload completed for {cur_lane.name}")

    def calibrate_td1(self, cur_lane, dis, tol):
        """
        Calibration function for automatically determining td1_bowden_length.
        Uses OAMS load/unload commands and TD-1 data detection to bracket distance.
        """
        if cur_lane.td1_device_id is None:
            msg = f"Cannot calibrate TD-1 for {cur_lane.name}, td1_device_id is a required "
            msg += "field in AFC_hub or per AFC_lane"
            return False, msg, 0

        if self.oams is None:
            msg = "OpenAMS hardware not available; cannot calibrate TD-1."
            self.logger.error(msg)
            return False, msg, 0

        spool_index = self._get_openams_spool_index(cur_lane)
        if spool_index is None:
            msg = f"Unable to resolve spool index for {cur_lane.name}"
            return False, msg, 0

        # Verify TD-1 is still connected before trying to get data
        valid, msg = self.afc.function.check_for_td1_id(cur_lane.td1_device_id)
        if not valid:
            msg = f"TD-1 device(SN: {cur_lane.td1_device_id}) not detected anymore, "
            msg += "please check before continuing to calibrate TD-1 bowden length"
            return valid, msg, 0

        self.logger.raw(f"Calibrating bowden length to TD-1 device with {cur_lane.name}")
        gcode = self.gcode
        fps_id = self._get_fps_id_for_lane(cur_lane.name)
        if fps_id is None:
            msg = f"Unable to resolve FPS for {cur_lane.name}"
            self.logger.error(msg)
            return False, msg, 0

        # Load the spool before starting TD-1 calibration
        # The load command is needed to move filament from spool bay to hub motor
        try:
            self.oams.oams_load_spool_cmd.send([spool_index])
        except Exception:
            self.logger.error(f"Failed to start spool load for TD-1 calibration on {cur_lane.name}")
            return False, "Failed to start spool load", 0

        # Wait for hub to load and STOP immediately when hub sensor triggers
        # IMPORTANT: Always use OAMS hardware sensor, not hub_obj (which doesn't update in real-time)
        hub_timeout = self.afc.reactor.monotonic() + 10.0
        hub_detected = False
        check_count = 0

        self.logger.debug(f"TD-1 calibration: using OAMS hardware sensor hub_hes_value[{spool_index}] for {cur_lane.name}")

        while self.afc.reactor.monotonic() < hub_timeout:
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)
            check_count += 1

            hub_loaded = None
            try:
                hub_loaded = bool(self.oams.hub_hes_value[spool_index])
                # Log every 10th check to avoid spam
                if check_count % 10 == 0:
                    self.logger.debug(f"TD-1 calibration check #{check_count}: hub_hes_value[{spool_index}] = {self.oams.hub_hes_value[spool_index]}")
            except Exception as e:
                if check_count % 10 == 0:
                    self.logger.debug(f"TD-1 calibration check #{check_count}: Exception reading hub_hes_value: {e}")
                hub_loaded = None

            if hub_loaded:
                hub_detected = True
                self.logger.info(f"Hub sensor triggered for {cur_lane.name} after {check_count} checks (~{check_count * 0.1:.1f}s)")
                break

        if not hub_detected:
            # Abort the load operation and stop the follower
            try:
                self.oams.abort_current_action(wait=True, code=0)
            except Exception:
                self.logger.error(f"Failed to abort load action for {cur_lane.name}")
            try:
                self.oams.set_oams_follower(0, 0)
            except Exception:
                self.logger.error(f"Failed to disable follower for {cur_lane.name}")
            msg = f"Hub sensor did not trigger during TD-1 calibration for {cur_lane.name}"
            self.logger.error(msg)
            return False, msg, 0

        # Hub loaded successfully - abort the load operation and take manual control
        # This stops the FPS-based load and lets us control follower manually by distance
        self.logger.debug(f"Hub loaded, aborting load operation and taking manual control for {cur_lane.name}")
        try:
            self.oams.abort_current_action(wait=True, code=0)
        except Exception:
            self.logger.error(f"Failed to abort load action for {cur_lane.name}")
        try:
            self.oams.set_oams_follower(0, 0)
        except Exception:
            self.logger.error(f"Failed to stop follower for {cur_lane.name}")

        # Give it a moment for the follower to stop
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.5)

        try:
            encoder_before = int(self.oams.encoder_clicks)
        except Exception:
            encoder_before = None

        # Now cycle follower manually: off 1s, on 1s, repeat
        # This allows TD-1 to detect filament during the movement cycles
        compare_time = datetime.now()
        td1_timeout = self.afc.reactor.monotonic() + 180.0
        td1_detected = False
        cycle_duration = 1.0

        self.logger.debug(f"Starting manual follower cycling for TD-1 detection on {cur_lane.name}")
        while self.afc.reactor.monotonic() < td1_timeout:
            # Disable follower for 1 second
            try:
                self.oams.set_oams_follower(0, 0)
            except Exception:
                self.logger.error(f"Failed to disable follower during TD-1 cycling for {cur_lane.name}")
            self.afc.reactor.pause(self.afc.reactor.monotonic() + cycle_duration)

            # Enable follower forward for 1 second
            try:
                self.oams.set_oams_follower(1, 1)
            except Exception:
                self.logger.error(f"Failed to enable follower during TD-1 cycling for {cur_lane.name}")
            self.afc.reactor.pause(self.afc.reactor.monotonic() + cycle_duration)

            # Check for TD-1 data after each cycle
            if self.get_td1_data(cur_lane, compare_time):
                td1_detected = True
                self.logger.debug(f"TD-1 data detected for {cur_lane.name}")
                break
            compare_time = datetime.now()

        # Disable follower after cycling
        try:
            self.oams.set_oams_follower(0, 0)
        except Exception:
            self.logger.error(f"Failed to disable follower after TD-1 cycling for {cur_lane.name}")

        if not td1_detected:
            # Filament reached hub but not TD-1 - unload it
            self._unload_after_td1(cur_lane, spool_index, fps_id)
            msg = f"TD-1 failed to detect filament for {cur_lane.name}"
            return False, msg, 0

        try:
            encoder_after = int(self.oams.encoder_clicks)
        except Exception:
            encoder_after = None

        if encoder_before is None or encoder_after is None:
            msg = "Unable to read encoder clicks for TD-1 calibration"
            self.logger.error(msg)
            return False, msg, 0

        encoder_delta = abs(encoder_after - encoder_before)
        cal_msg = f"\n td1_bowden_length: New: {encoder_delta} Old: {cur_lane.td1_bowden_length}"

        cur_lane.td1_bowden_length = encoder_delta
        fullname = cur_lane.fullname
        self.afc.function.ConfigRewrite(fullname, "td1_bowden_length", encoder_delta, cal_msg)
        cur_lane.do_enable(False)
        cur_lane.unit_obj.return_to_home()
        self.afc.save_vars()

        # Unload filament after successful TD-1 calibration
        self._unload_after_td1(cur_lane, spool_index, fps_id)

        return True, "td1_bowden_length calibration successful", encoder_delta

    def _capture_td1_with_oams(
        self,
        cur_lane,
        *,
        require_loaded: bool,
        require_enabled: bool,
    ) -> Tuple[bool, str]:
        if require_enabled and not cur_lane.td1_when_loaded:
            return False, "TD-1 capture disabled"
        if cur_lane.td1_device_id is None:
            return False, "TD-1 device ID not configured"

        # Check if toolhead actually has filament loaded (not just hub)
        # tool_loaded indicates filament is loaded all the way to the toolhead
        # This is different from the lane being "current" which just means it's in the hub
        if getattr(cur_lane, "tool_loaded", False):
            self.logger.info(
                f"Cannot get TD-1 data for {cur_lane.name}, toolhead is loaded"
            )
            return False, "Toolhead is loaded"

        # Fix hub state logic - only check if hub exists and is loaded
        hub_state = False
        if cur_lane.hub_obj is not None:
            hub_state = bool(cur_lane.hub_obj.state)

        if cur_lane.td1_bowden_length is None:
            self.logger.info(
                f"td1_bowden_length not set for {cur_lane.name}, skipping TD-1 capture"
            )
            return False, "td1_bowden_length not set"
        if require_loaded and not (cur_lane.load_state or cur_lane.prep_state):
            return False, "Lane is not loaded"
        if self.oams is None:
            self.logger.error("OpenAMS hardware not available; skipping TD-1 capture")
            return False, "OpenAMS hardware not available"

        spool_index = self._get_openams_spool_index(cur_lane)
        if spool_index is None:
            self.logger.error(f"Unable to resolve spool index for {cur_lane.name}")
            return False, "Unable to resolve spool index"

        # Check for conflicts with other OpenAMS units
        hub_values = getattr(self.oams, "hub_hes_value", None)
        current_hub_loaded = False
        other_hub_loaded = False
        try:
            if hub_values and spool_index < len(hub_values):
                current_hub_loaded = bool(hub_values[spool_index])
                other_hub_loaded = any(
                    bool(value) for idx, value in enumerate(hub_values) if idx != spool_index
                )
        except Exception:
            current_hub_loaded = False
            other_hub_loaded = False

        if other_hub_loaded:
            self.logger.info(
                "Skipping TD-1 capture for %s because another OpenAMS hub is already loaded",
                cur_lane.name,
            )
            return False, "Another OpenAMS hub already loaded"

        if hub_state and not current_hub_loaded:
            self.logger.info(
                f"Cannot get TD-1 data for {cur_lane.name}, hub shows filament in path"
            )
            return False, "Hub shows filament in path"

        # Load the spool before starting TD-1 capture
        # The load command is needed to move filament from spool bay to follower (hub) motor
        try:
            self.oams.oams_load_spool_cmd.send([spool_index])
        except Exception:
            self.logger.error(f"Failed to start spool load for TD-1 capture on {cur_lane.name}")
            return False, "Failed to start spool load"

        fps_id = self._get_fps_id_for_lane(cur_lane.name)
        if fps_id is None:
            self.logger.error(f"Unable to resolve FPS for {cur_lane.name}")
            try:
                self.oams.set_oams_follower(0, 0)
            except Exception:
                pass
            try:
                self.oams.oams_unload_spool_cmd.send()
            except Exception:
                pass
            return False, "Unable to resolve FPS"

        # Wait for hub to load (should happen within a few seconds)
        # Use OAMS hardware sensor, not hub_obj
        hub_timeout = self.afc.reactor.monotonic() + 10.0
        hub_detected = False
        self.logger.debug(f"TD-1 capture: waiting for hub sensor on {cur_lane.name}")

        while self.afc.reactor.monotonic() < hub_timeout:
            try:
                hub_detected = bool(self.oams.hub_hes_value[spool_index])
            except Exception:
                hub_detected = False
            if hub_detected:
                self.logger.info(f"Hub sensor triggered for TD-1 capture on {cur_lane.name}")
                break
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        if not hub_detected:
            # Abort the load operation and stop the follower
            try:
                self.oams.abort_current_action(wait=True, code=0)
            except Exception:
                pass
            try:
                self.oams.set_oams_follower(0, 0)
            except Exception:
                pass
            self.logger.error(
                f"Hub sensor did not trigger during TD-1 capture for {cur_lane.name}"
            )
            return False, "Hub sensor did not trigger"

        # Hub detected - abort load operation and take manual control
        # This stops FPS-based loading and lets us feed exact distance to TD-1
        self.logger.debug(f"Hub detected, aborting load and taking manual control for {cur_lane.name}")
        try:
            self.oams.abort_current_action(wait=True, code=0)
        except Exception:
            self.logger.error(f"Failed to abort load action for {cur_lane.name}")

        try:
            encoder_before = int(self.oams.encoder_clicks)
        except Exception:
            encoder_before = None

        if encoder_before is None:
            try:
                self.oams.set_oams_follower(0, 0)
            except Exception:
                pass
            self.logger.error(
                f"Unable to read encoder before TD-1 capture for {cur_lane.name}"
            )
            return False, "Unable to read encoder before capture"

        # Feed filament by td1_bowden_length + 5 clicks to get past TD-1 sensor
        # Enable follower manually to feed the exact distance
        target_clicks = max(0, int(cur_lane.td1_bowden_length) + 5)
        compare_time = datetime.now()
        td1_timeout = self.afc.reactor.monotonic() + 30.0

        self.logger.debug(f"TD-1 capture: feeding {target_clicks} encoder clicks to TD-1 on {cur_lane.name}")
        try:
            self.oams.set_oams_follower(1, 1)  # Enable follower forward
        except Exception:
            self.logger.error(f"Failed to enable follower for TD-1 capture on {cur_lane.name}")
            return False, "Failed to enable follower"
        while self.afc.reactor.monotonic() < td1_timeout:
            try:
                encoder_now = int(self.oams.encoder_clicks)
            except Exception:
                encoder_now = encoder_before

            clicks_moved = abs(encoder_now - encoder_before)
            if clicks_moved >= target_clicks:
                self.logger.debug(f"TD-1 capture: reached {clicks_moved} clicks on {cur_lane.name}")
                break
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

        # Stop the load - disable follower
        try:
            self.oams.set_oams_follower(0, 0)
        except Exception:
            self.logger.error(f"Failed to disable follower after TD-1 capture for {cur_lane.name}")

        # Wait for TD-1 to read data
        # TD-1 needs full 3.5 seconds to properly read data before unload
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 3.5)
        self.get_td1_data(cur_lane, compare_time)

        # Unload filament after successful TD-1 capture
        self._unload_after_td1(cur_lane, spool_index, fps_id)

        return True, "TD-1 data captured"

    def prep_capture_td1(self, cur_lane):
        self._capture_td1_with_oams(
            cur_lane,
            require_loaded=False,
            require_enabled=True,
        )

    def capture_td1_data(self, cur_lane):
        return self._capture_td1_with_oams(
            cur_lane,
            require_loaded=True,
            require_enabled=False,
        )

    def calibrate_hub(self, cur_lane, tol):
        """OpenAMS units use different calibration commands."""
        msg = (
            "OpenAMS units do not support standard AFC hub calibration. "
            "Use OpenAMS-specific calibration commands instead:\n"
            "  - AFC_OAMS_CALIBRATE_HUB_HES UNIT={} SPOOL=<spool_index>\n"
            "  - AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT={}"
        ).format(self.name, self.name)
        self.logger.info(msg)
        return False, msg, 0

    def _sync_afc_from_hardware_at_startup(self):
        """Align AFC state with OpenAMS hardware sensors.

        This reconciles AFC.var.unit and AFC live state with actual hardware sensors.
        Can be called at startup or on-demand (e.g., OAMSM_CLEAR_ERRORS).

        Priority hierarchy:
        1. Tool sensor (F1S/virtual) = Highest priority (actual filament in toolhead)
        2. Hub sensor = Secondary (filament present in AMS)
        3. AFC.var.unit = Lowest priority (might be stale)
        """
        if not hasattr(self, 'afc') or self.afc is None:
            self.logger.debug("State sync: AFC not available, skipping")
            return

        if not hasattr(self, 'lanes') or not self.lanes:
            self.logger.debug("State sync: No lanes configured, skipping")
            return

        self.logger.info(f"State sync: Reconciling AFC state with {self.name} hardware sensors")

        synced_count = 0
        cleared_count = 0
        skipped_count = 0
        conflict_count = 0

        # Track which extruders have lanes claiming to be loaded
        extruder_loaded_lanes = {}  # {extruder_name: [lane_names]}

        for lane in self.lanes.values():
            try:
                lane_name = getattr(lane, 'name', None)
                if not lane_name:
                    skipped_count += 1
                    continue

                extruder_obj = getattr(lane, 'extruder_obj', None)
                if extruder_obj is None:
                    skipped_count += 1
                    continue

                extruder_name = getattr(extruder_obj, 'name', 'unknown')

                # Read ACTUAL hardware state
                # Tool sensor is the ground truth - if it shows loaded, filament is in toolhead
                tool_loaded = False
                try:
                    tool_loaded = self._lane_reports_tool_filament(lane, sync_only=False)
                except Exception:
                    pass

                # Hub sensor shows filament in AMS (but not necessarily in toolhead)
                # CRITICAL: Read ACTUAL hardware sensor, not AFC state!
                hub_loaded = False
                if self.oams is not None:
                    try:
                        spool_index = self._get_openams_spool_index(lane)
                        if spool_index is not None:
                            hub_loaded = bool(self.oams.hub_hes_value[spool_index])
                    except Exception:
                        pass

                # Read what AFC THINKS
                afc_lane_loaded = getattr(extruder_obj, 'lane_loaded', None)
                afc_thinks_this_lane = (afc_lane_loaded == lane_name)

                # Track lanes that have tool filament for conflict detection
                if tool_loaded:
                    if extruder_name not in extruder_loaded_lanes:
                        extruder_loaded_lanes[extruder_name] = []
                    extruder_loaded_lanes[extruder_name].append(lane_name)

                # RECONCILE: Tool sensor wins (it's the authoritative truth)
                if tool_loaded and not afc_thinks_this_lane:
                    # Hardware shows loaded but AFC doesn't know
                    # Only update if we're sure (no conflicts)
                    if extruder_name in extruder_loaded_lanes and len(extruder_loaded_lanes[extruder_name]) > 1:
                        # Multiple lanes show loaded for same extruder - don't auto-fix
                        self.logger.error(
                            f"State sync: Multiple lanes show tool filament for {extruder_name}: "
                            f"{extruder_loaded_lanes[extruder_name]}. Manual intervention required."
                        )
                        conflict_count += 1
                        continue

                    # Check if AFC thinks a non-AMS lane is loaded - don't override non-AMS lanes!
                    if afc_lane_loaded is not None:
                        # Look up what AFC thinks is loaded and check if it's an AMS lane
                        loaded_lane_obj = self.afc.lanes.get(afc_lane_loaded) if hasattr(self.afc, 'lanes') else None
                        if loaded_lane_obj is not None:
                            loaded_unit_obj = getattr(loaded_lane_obj, 'unit_obj', None)
                            loaded_unit_type = getattr(loaded_unit_obj, 'type', None) if loaded_unit_obj else None

                            if loaded_unit_type != "OpenAMS":
                                # AFC thinks a non-AMS lane (Box Turtle, etc.) is loaded
                                # Don't auto-correct based on AMS sensors!
                                self.logger.debug(
                                    f"State sync: AMS sensors show {lane_name} on {extruder_name}, "
                                    f"but AFC thinks non-AMS lane '{afc_lane_loaded}' (type={loaded_unit_type}) is loaded. "
                                    f"Skipping auto-correction to preserve non-AMS state."
                                )
                                skipped_count += 1
                                continue

                    self.logger.info(
                        f"State sync: Sensors show {lane_name} loaded to {extruder_name}, "
                        f"but AFC thinks '{afc_lane_loaded}' is loaded. Updating AFC to match hardware."
                    )
                    try:
                        extruder_obj.lane_loaded = lane_name
                        synced_count += 1
                        # Also ensure AFC saves this state
                        if hasattr(self.afc, 'save_vars'):
                            self.afc.save_vars()
                    except Exception as e:
                        self.logger.error(f"State sync: Failed to update AFC for {lane_name}: {e}")

                elif not tool_loaded and afc_thinks_this_lane:
                    # AFC thinks loaded but no filament detected
                    if not hub_loaded:
                        # Really empty - clear stale AFC state
                        self.logger.info(
                            f"State sync: No filament detected for {lane_name}, but AFC thinks it's loaded. "
                            f"Clearing stale AFC state."
                        )
                        try:
                            extruder_obj.lane_loaded = None
                            cleared_count += 1
                            # Save the cleared state
                            if hasattr(self.afc, 'save_vars'):
                                self.afc.save_vars()
                        except Exception as e:
                            self.logger.error(f"State sync: Failed to clear AFC state for {lane_name}: {e}")
                    else:
                        # Filament in hub but not tool - this is a mid-load state, don't change it
                        self.logger.debug(
                            f"State sync: {lane_name} has filament in hub but not tool. "
                            f"Leaving AFC state as-is (mid-load state)."
                        )
                        skipped_count += 1

                elif tool_loaded and afc_thinks_this_lane:
                    # Hardware and AFC agree - all good
                    self.logger.debug(f"State sync: {lane_name} state matches between hardware and AFC")

                elif not tool_loaded and not afc_thinks_this_lane:
                    # Both agree it's not loaded - all good
                    self.logger.debug(f"State sync: {lane_name} correctly shows as unloaded in both hardware and AFC")

            except Exception as e:
                self.logger.error(f"State sync: Failed to process lane {getattr(lane, 'name', 'unknown')}: {e}")
                skipped_count += 1

        # Summary log
        self.logger.info(
            f"State sync complete for {self.name}: "
            f"{synced_count} lanes synced to AFC, "
            f"{cleared_count} stale states cleared, "
            f"{skipped_count} skipped, "
            f"{conflict_count} conflicts requiring manual resolution"
        )

        if conflict_count > 0:
            self.logger.error(
                f"State sync: {conflict_count} conflicts detected. "
                f"Multiple lanes show loaded for the same extruder. "
                f"Use TOOL_UNLOAD and load the correct lane manually."
            )

    def handle_ready(self):
        """Resolve the OpenAMS object once Klippy is ready."""
        # CRITICAL: Re-enforce buffer_obj = None on all AMS lanes FIRST
        # This runs during klippy:ready AFTER AFC_lane._handle_ready() has initialized lanes
        # AFC_lane._handle_ready() may assign buffers from unit/extruder, we must override to None
        # MUST run BEFORE any early returns to ensure buffers are always cleared
        for lane in self.lanes.values():
            if lane.buffer_obj is not None:
                buffer_name = getattr(lane.buffer_obj, 'name', 'unknown')
                self.logger.warning(
                    f"Lane {lane.name} was assigned buffer '{buffer_name}' during initialization, "
                    f"but OpenAMS units don't have physical buffers. Removing buffer assignment."
                )
                lane.buffer_obj = None

        # First check if ANY OpenAMS hardware exists in the system
        if not _has_openams_hardware(self.printer):
            self.logger.info(
                "No OpenAMS hardware found in configuration. "
                f"Skipping OpenAMS integration for AFC unit '{self.name}'. "
                "This is normal if you are using Box Turtle or other non-AMS hardware."
            )
            # Skip all OpenAMS initialization if no hardware is present
            return

        if self.hardware_service is not None:
            self.oams = self.hardware_service.resolve_controller()
        else:
            self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
            if AMSHardwareService is not None and self.oams is not None:
                try:
                    self.hardware_service = AMSHardwareService.for_printer(self.printer, self.oams_name, self.logger)
                    self.hardware_service.attach_controller(self.oams)
                except Exception:
                    self.logger.error(f"Failed to attach AMSHardwareService for {self.oams_name}")
        # Check if OAMS hardware was found for THIS specific unit
        if self.oams is None:
            self.logger.warning(
                "OpenAMS hardware '[oams "
                f"{self.oams_name}]' not found for AFC unit '{self.name}'. "
                "If you are using Box Turtle or other non-AMS hardware, "
                f"remove the '[afc_openams {self.name}]' section from your config."
            )
            # Don't start polling if no OAMS hardware
            return

        # Subscribe to hardware sensor events (requires AMSHardwareService)
        if self.hardware_service is None:
            self.logger.error(
                f"AMSHardwareService not available for {self.name} - OpenAMS requires openams_integration.py. "
                "Sensor updates will not work!"
            )
            return

        # Subscribe to sensor change events
        self.event_bus.subscribe("f1s_changed", self._on_f1s_changed, priority=5)
        self.event_bus.subscribe("hub_changed", self._on_hub_changed, priority=5)

        # Start unified polling in AMSHardwareService
        try:
            self.hardware_service.start_polling()
        except Exception:
            self.logger.error(f"Failed to start unified polling for {self.oams_name}")
        # Hook into AFC's LANE_UNLOAD for cross-extruder runouts
        self._wrap_afc_lane_unload()
        self._wrap_afc_unset_lane_loaded()
        self._patch_afc_sequences()

        # Sync AFC state with hardware sensors at startup
        # This should run after all initialization is complete and sensors are stable
        # Delay slightly to ensure sensors have had time to stabilize
        self.reactor.register_callback(lambda et: self._sync_afc_from_hardware_at_startup())

    def _wrap_afc_lane_unload(self):
        """Wrap AFC's LANE_UNLOAD to handle cross-extruder runout scenarios."""
        if not hasattr(self, 'afc') or self.afc is None:
            return

        # Store original method if not already wrapped
        if not hasattr(self.afc, '_original_LANE_UNLOAD'):
            self.afc._original_LANE_UNLOAD = self.afc.LANE_UNLOAD

        # Create wrapper
        def wrapped_lane_unload(gcmd_or_lane):
            # LANE_UNLOAD can be called two ways:
            # 1. As gcode command: LANE_UNLOAD LANE=lane7 (gcmd has .get())
            # 2. Direct call: self.LANE_UNLOAD(cur_lane) (AFCLane object)
            lane = None

            # Check if it's a GCodeCommand (has 'get' method) or AFCLane object
            if hasattr(gcmd_or_lane, 'get'):
                # Gcode command - extract lane name
                lane_name = gcmd_or_lane.get('LANE', None)
                if lane_name and lane_name in self.afc.lanes:
                    lane = self.afc.lanes[lane_name]
            elif hasattr(gcmd_or_lane, 'name'):
                # Direct lane object passed
                lane = gcmd_or_lane

            # Check if this is an OpenAMS lane with cross-extruder runout flag
            if lane is not None:
                if getattr(lane, 'unit_obj', None) is not None and getattr(lane.unit_obj, 'type', None) == "OpenAMS":
                    is_cross_extruder = getattr(lane, '_oams_cross_extruder_runout', False)
                    if is_cross_extruder:
                        lane_name = getattr(lane, 'name', 'unknown')
                        self.logger.debug("Cross-Extruder LANE_UNLOAD for {} - clearing extruder.lane_loaded to bypass check".format(lane_name))
                        try:
                            # Get the extruder and current active extruder
                            lane_extruder = getattr(lane, 'extruder_obj', None)
                            current_extruder = self.afc.function.get_current_extruder()

                            # If lane is loaded in a different extruder than current, clear it
                            if lane_extruder is not None and current_extruder != getattr(lane_extruder, 'name', None):
                                self.logger.debug("Cross-Extruder: Lane {} on extruder {}, current is {} - clearing lane_loaded".format(
                                    lane_name,
                                    getattr(lane_extruder, 'name', 'unknown'),
                                    current_extruder
                                ))
                                lane_extruder.lane_loaded = None
                                self.logger.debug("Cross-Extruder: Cleared extruder.lane_loaded for cross-extruder unload")

                                # Also clear FPS state in OAMS manager
                                try:
                                    oams_mgr = self.printer.lookup_object("oams_manager", None)
                                    if oams_mgr is not None:
                                        fps_name = oams_mgr.get_fps_for_afc_lane(lane_name)
                                        if fps_name:
                                            fps_state = oams_mgr.current_state.fps_state.get(fps_name)
                                            if fps_state is not None:
                                                spool_index = fps_state.current_spool_idx

                                                # Clear FPS state (matching _unload_filament_for_fps logic)
                                                fps_state.state = 0  # FPSLoadState.UNLOADED
                                                fps_state.following = False
                                                fps_state.direction = 0
                                                fps_state.clog_restore_follower = False
                                                fps_state.clog_restore_direction = 1
                                                fps_state.since = self.reactor.monotonic()
                                                fps_state.current_lane = None
                                                fps_state.current_spool_idx = None
                                                if hasattr(fps_state, 'reset_stuck_spool_state'):
                                                    fps_state.reset_stuck_spool_state()

                                                self.logger.debug("Cross-Extruder: Cleared FPS state for {} (was spool {})".format(fps_name, spool_index))

                                                # Notify AFC via AMSRunoutCoordinator
                                                try:
                                                    from .openams_integration import AMSRunoutCoordinator
                                                    AMSRunoutCoordinator.notify_lane_tool_state(
                                                        self.printer,
                                                        self.oams_name,
                                                        lane_name,
                                                        loaded=False,
                                                        spool_index=spool_index,
                                                        eventtime=fps_state.since
                                                    )
                                                    self.logger.debug("Cross-Extruder: Notified AFC that lane {} unloaded".format(lane_name))
                                                except Exception:
                                                    self.logger.error("Failed to notify AFC about lane {} unload".format(lane_name))

                                                # Also manually set the virtual tool sensor to False for AMS virtual extruders
                                                # This ensures virtual sensor (e.g., AMS_Extruder4) shows correct state
                                                try:
                                                    if lane_extruder is not None:
                                                        extruder_name = getattr(lane_extruder, 'name', None)
                                                        if extruder_name and extruder_name.upper().startswith('AMS_'):
                                                            sensor_name = extruder_name.replace('ams_', '').replace('AMS_', '')
                                                            sensor = self.printer.lookup_object("filament_switch_sensor {}".format(sensor_name), None)
                                                            if sensor and hasattr(sensor, 'runout_helper'):
                                                                sensor.runout_helper.note_filament_present(self.reactor.monotonic(), is_filament_present=False)
                                                                self.logger.debug("Cross-Extruder: Set virtual sensor {} to False after cross-extruder runout".format(sensor_name))
                                                except Exception:
                                                    self.logger.error("Failed to update virtual sensor for lane {} during cross-extruder runout".format(lane_name))
                                            else:
                                                self.logger.warning("Cross-Extruder: Could not find FPS state for {}".format(fps_name))
                                        else:
                                            self.logger.warning("Cross-Extruder: Could not find FPS name for lane {}".format(lane_name))
                                    else:
                                        self.logger.warning("Cross-Extruder: OAMS manager not found, FPS state not cleared")
                                except Exception:
                                    self.logger.error("Failed to clear FPS state for lane {}".format(lane_name))

                            # Clear the flag
                            lane._oams_cross_extruder_runout = False
                        except Exception as e:
                            self.logger.error("Failed to handle cross-extruder LANE_UNLOAD for {}: {}".format(lane_name, str(e)))

            # Call original method
            return self.afc._original_LANE_UNLOAD(gcmd_or_lane)

        # Apply wrapper
        self.afc.LANE_UNLOAD = wrapped_lane_unload
        self.logger.debug("Wrapped AFC.LANE_UNLOAD for cross-extruder runout handling")

    def _wrap_afc_unset_lane_loaded(self):
        """Wrap AFC's unset_lane_loaded to notify OAMS manager for OpenAMS lanes."""
        if not hasattr(self, 'afc') or self.afc is None:
            return

        afc_function = getattr(self.afc, "function", None)
        if afc_function is None:
            return

        if not hasattr(afc_function, "unset_lane_loaded"):
            return

        if not hasattr(afc_function, "_oams_unset_lane_loaded_original"):
            afc_function._oams_unset_lane_loaded_original = afc_function.unset_lane_loaded

        def unset_lane_loaded_wrapper():
            cur_lane_loaded = afc_function.get_current_lane_obj()
            lane_name = getattr(cur_lane_loaded, "name", None) if cur_lane_loaded else None
            unit_obj = getattr(cur_lane_loaded, "unit_obj", None) if cur_lane_loaded else None
            is_openams = unit_obj is not None and getattr(unit_obj, "type", "") == "OpenAMS"

            result = afc_function._oams_unset_lane_loaded_original()

            if is_openams and lane_name:
                try:
                    oams_manager = self.printer.lookup_object("oams_manager", None)
                    extruder_name = getattr(cur_lane_loaded.extruder_obj, "name", None) if cur_lane_loaded else None
                    if oams_manager is not None:
                        oams_manager.on_afc_lane_unloaded(lane_name, extruder_name=extruder_name)
                except Exception:
                    self.logger.error("Failed to notify OAMS manager during unset_lane_loaded")
            return result

        afc_function.unset_lane_loaded = unset_lane_loaded_wrapper
        self.logger.debug("Wrapped AFC.function.unset_lane_loaded for OpenAMS state sync")

    def _disable_buffers_for_openams_load(self, afc, cur_lane) -> None:
        """Disable all lane buffers before an OpenAMS load to avoid stray buffer faults."""
        try:
            for lane_name, lane_obj in afc.lanes.items():
                if lane_obj is None:
                    continue
                try:
                    lane_obj.disable_buffer()
                except Exception:
                    self.logger.debug(f"Failed to disable buffer for lane {lane_name}")
        except Exception:
            self.logger.error(f"Failed to disable buffers before loading lane {cur_lane.name}")

    def _patch_afc_sequences(self) -> None:
        """Patch AFC load/unload sequences to delegate OpenAMS lanes."""
        if not hasattr(self, "afc") or self.afc is None:
            return

        afc = self.afc
        if getattr(afc, "_oams_sequences_patched", False):
            return

        if not hasattr(afc, "load_sequence") or not hasattr(afc, "unload_sequence"):
            return

        afc._oams_load_sequence_original = afc.load_sequence
        afc._oams_unload_sequence_original = afc.unload_sequence
        def load_sequence_wrapper(afc_self, cur_lane, cur_hub, cur_extruder):
            unit_obj = getattr(cur_lane, "unit_obj", None)
            is_openams = unit_obj is not None and getattr(unit_obj, "type", "") == "OpenAMS"
            if not is_openams:
                return afc_self._oams_load_sequence_original(cur_lane, cur_hub, cur_extruder)

            # Check if this lane is already loaded to toolhead
            # If so, skip load and just sync state
            if cur_lane.get_toolhead_pre_sensor_state() and hasattr(cur_lane, 'tool_loaded') and cur_lane.tool_loaded:
                afc_self.logger.debug(f"Lane {cur_lane.name} already loaded to toolhead, skipping load")
                cur_lane.set_tool_loaded()
                afc_self.save_vars()
                return True

            self._disable_buffers_for_openams_load(afc_self, cur_lane)

            if afc_self._check_extruder_temp(cur_lane):
                afc_self.afcDeltaTime.log_with_time("Done heating toolhead")

            try:
                afc_self.logger.debug(
                    f"OpenAMS load: delegating to OAMSM_LOAD_FILAMENT for lane {cur_lane.name}"
                )
                oams_manager = afc_self.printer.lookup_object("oams_manager", None)
                if oams_manager is not None:
                    success, message = oams_manager._load_filament_for_lane(cur_lane.name)
                    if not success:
                        message = message or f"OpenAMS load failed for {cur_lane.name}"
                        afc_self.error.handle_lane_failure(cur_lane, message)
                        return False
                else:
                    afc_self.gcode.run_script_from_command(
                        "OAMSM_LOAD_FILAMENT LANE={}".format(cur_lane.name)
                    )
            except Exception as e:
                message = "OpenAMS load failed for {}: {}".format(cur_lane.name, str(e))
                afc_self.error.handle_lane_failure(cur_lane, message)
                return False

            if not cur_lane.get_toolhead_pre_sensor_state():
                message = (
                    "OpenAMS load did not trigger pre extruder gear toolhead sensor, CHECK FILAMENT PATH\n"
                    "||=====||====||==>--||\nTRG   LOAD   HUB   TOOL"
                )
                message += "\nTo resolve set lane loaded with `SET_LANE_LOADED LANE={}` macro.".format(cur_lane.name)
                if afc_self.function.in_print():
                    message += "\nOnce filament is fully loaded click resume to continue printing"
                afc_self.error.handle_lane_failure(cur_lane, message)
                return False

            cur_lane.set_tool_loaded()
            afc_self.save_vars()
            return True

        def unload_sequence_wrapper(afc_self, cur_lane, cur_hub, cur_extruder):
            unit_obj = getattr(cur_lane, "unit_obj", None)
            is_openams = unit_obj is not None and getattr(unit_obj, "type", "") == "OpenAMS"
            if not is_openams:
                return afc_self._oams_unload_sequence_original(cur_lane, cur_hub, cur_extruder)

            cur_lane.status = AFCLaneState.TOOL_UNLOADING

            if afc_self._check_extruder_temp(cur_lane):
                afc_self.afcDeltaTime.log_with_time("Done heating toolhead")

            afc_self.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull", wait_tool=False)
            afc_self.function.log_toolhead_pos("TOOL_UNLOAD quick pull: ")
            cur_lane.unit_obj.lane_unloading(cur_lane)
            cur_lane.sync_to_extruder()
            cur_lane.do_enable(True)
            cur_lane.select_lane()

            if afc_self.tool_cut:
                # Stats moved from AFC_stats to AFC_extruder (AFCExtruderStats)
                cur_lane.extruder_obj.estats.increase_cut_total()
                afc_self.gcode.run_script_from_command(afc_self.tool_cut_cmd)
                afc_self.afcDeltaTime.log_with_time("TOOL_UNLOAD: After cut")
                afc_self.function.log_toolhead_pos()

                if afc_self.park:
                    afc_self.gcode.run_script_from_command(afc_self.park_cmd)
                    afc_self.afcDeltaTime.log_with_time("TOOL_UNLOAD: After park")
                    afc_self.function.log_toolhead_pos()

            if afc_self.form_tip:
                if afc_self.park:
                    afc_self.gcode.run_script_from_command(afc_self.park_cmd)
                    afc_self.afcDeltaTime.log_with_time("TOOL_UNLOAD: After form tip park")
                    afc_self.function.log_toolhead_pos()

                if afc_self.form_tip_cmd == "AFC":
                    afc_self.tip = afc_self.printer.lookup_object('AFC_form_tip')
                    afc_self.tip.tip_form()
                    afc_self.afcDeltaTime.log_with_time("TOOL_UNLOAD: After afc form tip")
                    afc_self.function.log_toolhead_pos()
                else:
                    afc_self.gcode.run_script_from_command(afc_self.form_tip_cmd)
                    afc_self.afcDeltaTime.log_with_time("TOOL_UNLOAD: After custom form tip")
                    afc_self.function.log_toolhead_pos()

            try:
                # CRITICAL: Unsync from extruder before OpenAMS unload
                # After cut/form_tip, lane is synced to extruder. Must unsync before
                # OAMSM_UNLOAD_FILAMENT can control the spool independently.
                cur_lane.unsync_to_extruder()

                oams_manager = afc_self.printer.lookup_object("oams_manager", None)
                fps_name = None
                if oams_manager is not None:
                    fps_name = oams_manager.get_fps_for_afc_lane(cur_lane.name)

                if not fps_name:
                    message = "OpenAMS unload failed for {}: unable to resolve FPS".format(cur_lane.name)
                    afc_self.error.handle_lane_failure(cur_lane, message)
                    return False

                fps_id = fps_name.split(" ", 1)[1] if fps_name.startswith("fps ") else fps_name
                afc_self.logger.debug(
                    "OpenAMS unload: delegating to OAMSM_UNLOAD_FILAMENT for lane {} (FPS {})".format(
                        cur_lane.name, fps_id
                    )
                )
                afc_self.gcode.run_script_from_command("OAMSM_UNLOAD_FILAMENT FPS={}".format(fps_id))

                # After unload, filament is loaded in AMS (at f1s position), ready for next load
                cur_lane.loaded_to_hub = True
                cur_lane.set_tool_unloaded()
                cur_lane.status = AFCLaneState.LOADED
                cur_lane.unit_obj.lane_tool_unloaded(cur_lane)
                afc_self.save_vars()
            except Exception as e:
                message = "OpenAMS unload failed for {}: {}".format(cur_lane.name, str(e))
                afc_self.error.handle_lane_failure(cur_lane, message)
                return False

            return True

        afc.load_sequence = MethodType(load_sequence_wrapper, afc)
        afc.unload_sequence = MethodType(unload_sequence_wrapper, afc)
        setattr(afc, "_oams_sequences_patched", True)
        self.logger.debug("AFC load/unload sequences patched for OpenAMS delegation")

    def _on_f1s_changed(self, event_type, unit_name, bay, value, eventtime, **kwargs):
        """Handle F1S sensor change events from AMSHardwareService.

        Event-driven architecture: sensor updates published by AMSHardwareService
        when hardware detects state changes, eliminating need for polling.
        """
        if unit_name != self.oams_name:
            return  # Not our unit

        lane = self._lane_for_spool_index(bay)
        if lane is None:
            return

        lane_val = bool(value)
        prev_val = getattr(lane, "load_state", False)

        # Update lane state based on sensor FIRST
        if getattr(lane, "ams_share_prep_load", False):
            self._update_shared_lane(lane, lane_val, eventtime)
        elif lane_val != prev_val:
            lane.load_callback(eventtime, lane_val)
            lane.prep_callback(eventtime, lane_val)
            self._mirror_lane_to_virtual_sensor(lane, eventtime)

        # Detect F1S sensor going False (spool empty) - trigger runout detection AFTER sensor update
        # Only trigger if printer is actively printing (not during filament insertion/removal)
        if prev_val and not lane_val:
            try:
                is_printing = self.afc.function.is_printing()
            except Exception:
                is_printing = False

            if is_printing:
                # CRITICAL: Only trigger runout detection if THIS lane is the one loaded to its extruder
                # Skip runout detection on inactive lanes (e.g., when changing spools on different lane)
                # In multi-extruder setups, check the specific extruder this lane is associated with
                skip_runout = False
                try:
                    extruder_obj = getattr(lane, 'extruder_obj', None)
                    if extruder_obj is not None:
                        lane_loaded = getattr(extruder_obj, 'lane_loaded', None)
                        if lane_loaded is not None and lane_loaded != lane.name:
                            # This lane is NOT the one loaded to its extruder - skip runout detection
                            self.logger.debug(
                                f"F1S sensor False for {lane.name} but lane {lane_loaded} is loaded to "
                                f"{getattr(extruder_obj, 'name', 'extruder')} - skipping runout detection on inactive lane"
                            )
                            skip_runout = True
                except Exception:
                    pass

                if not skip_runout:
                    self.logger.info("F1S sensor False for {} (spool empty, printing), triggering runout detection".format(lane.name))
                    try:
                        self.handle_runout_detected(bay, None, lane_name=lane.name)
                    except Exception:
                        self.logger.error(
                            f"Failed to handle runout detection for {lane.name} "
                            f"(spool_index={bay}, runout_lane={getattr(lane, 'runout_lane', None)})\n"
                            f"{traceback.format_exc()}"
                        )
            else:
                self.logger.debug("F1S sensor False for {} but not printing - skipping runout detection (likely filament insertion/removal)".format(lane.name))

        # Update hardware service snapshot
        if self.hardware_service is not None:
            hub_obj = getattr(lane, "hub_obj", None)
            hub_state = getattr(lane, "loaded_to_hub", False) if hub_obj else None
            tool_state = self._lane_reports_tool_filament(lane)
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name, lane.name, lane_val, hub_state, eventtime,
                    spool_index=bay, tool_state=tool_state
                )
            except Exception:
                self.logger.error(f"Failed to update lane snapshot for {lane.name}")
        # Sync virtual tool sensor
        self._sync_virtual_tool_sensor(eventtime)

    def _on_hub_changed(self, event_type, unit_name, bay, value, eventtime, **kwargs):
        """Handle hub sensor change events from AMSHardwareService.

        Event-driven architecture: sensor updates published by AMSHardwareService
        when hardware detects state changes, eliminating need for polling.
        """
        if unit_name != self.oams_name:
            return  # Not our unit

        lane = self._lane_for_spool_index(bay)
        if lane is None:
            return

        hub = getattr(lane, "hub_obj", None)
        if hub is None:
            return

        hub_val = bool(value)
        if hub_val != getattr(lane, "loaded_to_hub", False):
            hub.switch_pin_callback(eventtime, hub_val)
            # CRITICAL: Update lane.loaded_to_hub to match hub sensor state
            # This field is reported to Mainsail via lane.get_status()
            # Without this, Mainsail shows stale hub status even when hardware sensor is correct
            lane.loaded_to_hub = hub_val
            fila = getattr(hub, "fila", None)
            if fila is not None:
                fila.runout_helper.note_filament_present(eventtime, hub_val)

        # Update hardware service snapshot
        if self.hardware_service is not None:
            lane_state = getattr(lane, "load_state", False)
            tool_state = self._lane_reports_tool_filament(lane)
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name, lane.name, lane_state, hub_val, eventtime,
                    spool_index=bay, tool_state=tool_state
                )
            except Exception:
                self.logger.error(f"Failed to update lane snapshot for {lane.name}")

    def _should_block_sensor_update_for_runout(self, lane, lane_val):
        """Check if sensor update should be blocked due to active runout.

        Returns True if the update should be blocked, False otherwise.
        Automatically clears the runout flag if runout handling is complete.
        """
        # Cross-extruder runouts should not block shared sensor transitions; let AFC handle
        # them like Box Turtle units. Only same-extruder runouts set _oams_runout_detected.
        if getattr(lane, '_oams_cross_extruder_runout', False):
            # Make sure any stale blocking flag is cleared for cross-extruder scenarios
            if hasattr(lane, '_oams_runout_detected'):
                lane._oams_runout_detected = False
            return False

        if not hasattr(lane, '_oams_runout_detected') or not lane._oams_runout_detected:
            return False

        should_block = False
        try:
            is_printing = self.afc.function.is_printing()
            is_tool_loaded = getattr(lane, 'tool_loaded', False)
            lane_status = getattr(lane, 'status', None)
            # Only block if actively printing with this lane loaded and in runout state
            if is_printing and is_tool_loaded and lane_status in (AFCLaneState.INFINITE_RUNOUT, AFCLaneState.TOOL_UNLOADING):
                should_block = True
            else:
                # Clear the flag - runout handling is complete
                lane._oams_runout_detected = False
                self.logger.debug(f"Clearing runout flag for lane {getattr(lane, 'name', 'unknown')} - runout handling complete")
        except Exception:
            # On error, clear the flag to be safe
            lane._oams_runout_detected = False

        # Block only if conditions met and trying to set sensors to True
        if should_block and lane_val:
            return True
        # Sensor confirms empty - always clear flag
        elif not lane_val:
            lane._oams_runout_detected = False
            self.logger.debug(f"Sensor confirmed empty state for lane {getattr(lane, 'name', 'unknown')} - clearing runout flag")
        return False

    def _trigger_td1_capture_delayed(self, lane_name):
        """
        Timer callback to trigger TD-1 capture after 3-second delay.
        This allows the AMS auto-load sequence to complete (loads near hub ? retracts ? settles).

        CRITICAL: This is called from timer context - must not use reactor.pause() or wait=True.
        """
        def _timer_callback(eventtime):
            try:
                # Clean up pending timer tracking
                if lane_name in self._pending_spool_loaded_timers:
                    del self._pending_spool_loaded_timers[lane_name]

                # Find the lane and trigger TD-1 capture
                lane = self.afc.lanes.get(lane_name)
                if lane is not None:
                    try:
                        self.logger.info(f"Triggering delayed TD-1 capture for {lane_name} (AMS settled)")
                        lane._prep_capture_td1()
                    except Exception as e:
                        self.logger.error(f"Failed to trigger TD-1 capture for {lane_name}: {e}")
                else:
                    self.logger.error(f"Lane {lane_name} not found for delayed TD-1 capture")
            except Exception as e:
                self.logger.error(f"Error in _trigger_td1_capture_delayed timer callback for {lane_name}: {e}")

            # Return NEVER to stop the timer from repeating
            return self.reactor.NEVER

        return _timer_callback

    def _update_shared_lane(self, lane, lane_val, eventtime, *, allow_clear: bool = True):
        """Synchronise shared prep/load sensor lanes without triggering errors."""
        # Check if runout handling requires blocking this sensor update
        if self._should_block_sensor_update_for_runout(lane, lane_val):
            return

        previous = getattr(lane, "load_state", False)
        lane_val_bool = bool(lane_val)
        prep_state = getattr(lane, "prep_state", None)
        load_state = getattr(lane, "load_state", None)

        if (
            previous is not None
            and bool(previous) == lane_val_bool
            and (prep_state is None or bool(prep_state) == lane_val_bool)
            and (load_state is None or bool(load_state) == lane_val_bool)
        ):
            return

        if lane_val_bool:
            # Defer metadata application (material, spoolman IDs, colors, etc.) to
            # AFC's callbacks. The callbacks will update prep/load state and apply lane data consistently for both
            # single- and shared-sensor lanes.
            try:
                lane.prep_callback(eventtime, True)
            finally:
                lane.load_callback(eventtime, True)

            self._mirror_lane_to_virtual_sensor(lane, eventtime)

            # Publish spool_loaded event immediately (TD-1 capture delay happens in event handler)
            if self.event_bus is not None:
                try:
                    spool_index = self._get_openams_spool_index(lane)
                    self.event_bus.publish("spool_loaded",
                        unit_name=self.name,
                        spool_index=spool_index,
                        eventtime=eventtime,
                    )
                except Exception as e:
                    self.logger.error(f"Failed to publish spool_loaded event for {lane.name}: {e}")
        else:
            # Sensor False - filament left spool bay
            # Update sensor state but don't aggressively clear everything (align with Box Turtle behavior)
            lane.load_callback(eventtime, False)
            lane.prep_callback(eventtime, False)

            self._mirror_lane_to_virtual_sensor(lane, eventtime)

            # Cancel any pending TD-1 capture timer since filament was removed
            lane_name = lane.name
            if lane_name in self._pending_spool_loaded_timers:
                try:
                    timer = self._pending_spool_loaded_timers[lane_name]
                    self.reactor.unregister_timer(timer)
                    del self._pending_spool_loaded_timers[lane_name]
                    self.logger.debug(f"Cancelled pending TD-1 capture timer for {lane_name} (filament removed)")
                except Exception as e:
                    self.logger.error(f"Error cancelling TD-1 capture timer for {lane_name}: {e}")

            # Shared prep/load sensors stay in sync for AMS lanes; treat False as fully unloaded
            try:
                # PHASE 1 REFACTOR: Remove redundant manual state assignments
                # lane.set_unloaded() already handles tool_loaded and loaded_to_hub
                # Only call set_unloaded if lane isn't already in NONE state
                # This prevents errors when sensor reports empty for an already-empty lane
                if lane.status != AFCLaneState.NONE:
                    lane.set_unloaded()
                if hasattr(lane, "_afc_prep_done"):
                    lane._afc_prep_done = False
                lane.prep_state = lane_val_bool
                lane.load_state = lane_val_bool
            except Exception as e:
                # This is often benign (lane already cleared), log at debug level
                self.logger.debug(f"Could not fully clear shared lane {lane.name} after sensor cleared (lane may already be empty): {e}")
            # For same-FPS runouts: blocking mechanism in _should_block_sensor_update_for_runout()
            # prevents us from reaching here during active runout
            # For cross-extruder runouts: AFC's LANE_UNLOAD wrapper handles cleanup
            # For manual unloads: AFC's LANE_UNLOAD command handles cleanup

            # Only unsync from extruder if not in active cross-extruder runout or tool operation
            try:
                is_printing = self.afc.function.is_printing()
            except Exception:
                is_printing = False
            is_cross_extruder_runout = getattr(lane, '_oams_cross_extruder_runout', False) and is_printing

            # Check if this is a shared extruder (multiple lanes on same extruder)
            # For shared extruders, NEVER clear lane_loaded from sensor callback
            # EXCEPT during runouts when filament actually left the toolhead
            # Sensor can go False during tool parking/swapping without filament leaving,
            # but during runouts the sensor goes False because filament ran out and must be cleared
            is_shared_extruder = False
            is_same_fps_runout = False
            try:
                if hasattr(lane, 'extruder_obj') and lane.extruder_obj is not None:
                    # no_lanes=True means standalone toolhead, no_lanes=False means shared extruder
                    is_shared_extruder = not getattr(lane.extruder_obj, 'no_lanes', True)

                    # Check if we're in a same-FPS runout (sensor went False during print, filament actually ran out)
                    # This is different from tool changes where sensor might briefly go False
                    if is_shared_extruder and is_printing:
                        # During runout, the runout monitor sets a flag to indicate filament actually ran out
                        # We should clear lane_loaded in this case even for shared extruders
                        is_same_fps_runout = getattr(lane, '_oams_same_fps_runout', False)
            except Exception:
                pass

            if not is_cross_extruder_runout and not (is_shared_extruder and not is_same_fps_runout):
                try:
                    if hasattr(lane, 'extruder_obj') and lane.extruder_obj is not None:
                        if lane.extruder_obj.lane_loaded == lane.name:
                            lane.unsync_to_extruder()
                            lane.extruder_obj.lane_loaded = None
                            self.logger.debug(f"Unsynced lane {lane.name} and cleared extruder.lane_loaded when sensor went False")
                except Exception:
                    self.logger.error(
                        f"Failed to unsync lane {lane.name} from extruder when sensor cleared\n"
                        f"{traceback.format_exc()}"
                    )
            elif is_shared_extruder and not is_same_fps_runout:
                self.logger.debug(f"Skipping lane_loaded clear for {lane.name} - shared extruder (only UNLOAD/RUNOUT commands should clear)")
            else:
                self.logger.debug(f"Skipping extruder unsync for {lane.name} - cross-extruder runout (AFC will handle via LANE_UNLOAD)")
        lane.afc.save_vars()
        lane.prep_state = lane_val_bool
        lane.load_state = lane_val_bool

    def _apply_lane_sensor_state(self, lane, lane_val, eventtime):
        """Apply a boolean lane sensor value using existing AFC callbacks."""
        # Check if runout handling requires blocking this sensor update
        if self._should_block_sensor_update_for_runout(lane, lane_val):
            self.logger.debug(f"Ignoring sensor update for lane {getattr(lane, 'name', 'unknown')} - runout in progress")
            return

        try:
            share = getattr(lane, "ams_share_prep_load", False)
        except Exception:
            share = False

        if share:
            self._update_shared_lane(lane, lane_val, eventtime)
            return

        previous = getattr(lane, "load_state", False)

        if previous is not None and bool(previous) == bool(lane_val):
            return

        try:
            lane.load_callback(eventtime, lane_val)
        except TypeError:
            lane.load_callback(eventtime, load_state=lane_val)
        except Exception:
            self.logger.error(f"Failed to update load sensor for lane {lane}")
        try:
            lane.prep_callback(eventtime, lane_val)
        except TypeError:
            lane.prep_callback(eventtime, prep_state=lane_val)
        except Exception:
            self.logger.error(f"Failed to update prep sensor for lane {lane}")
        # When sensor goes False (empty), only clear tool/hub loaded flags
        # Let AFC's normal flow handle status and cleanup (align with Box Turtle)
        if not lane_val and allow_clear:
            lane.tool_loaded = False
            lane.loaded_to_hub = False

        self._mirror_lane_to_virtual_sensor(lane, eventtime)

    def _lane_for_spool_index(self, spool_index: Optional[int]):
        """Use indexed lookup instead of iteration."""
        if spool_index is None:
            return None

        try:
            normalized = int(spool_index)
        except (TypeError, ValueError):
            return None

        registry_unit = self.oams_name or self.name
        if self.registry is not None:
            lane_info = self.registry.get_by_spool(registry_unit, normalized)
            if lane_info is not None:
                lane = self.lanes.get(lane_info.lane_name)
                if lane is not None:
                    return lane

        if normalized < 0 or normalized >= 4:
            return None

        return self._lane_by_local_index(normalized)

    def _resolve_lane_reference(self, lane_name: Optional[str]):
        """Return a lane object by name (or alias), case-insensitively."""
        lane, _trace = self._resolve_lane_reference_with_trace(lane_name)
        return lane

    def _resolve_lane_reference_with_trace(self, lane_name: Optional[str]):
        """Return a lane object and the resolution path for debugging."""
        trace: List[str] = []
        if not lane_name:
            trace.append("no name provided")
            return None, trace

        def _normalize_tool_token(token: Optional[str]) -> Optional[str]:
            if not isinstance(token, str):
                return None
            cleaned = token.strip().lower()
            if cleaned.startswith("t"):
                cleaned = cleaned[1:]
            return cleaned or None

        resolved_name = self._resolve_lane_alias(lane_name)
        registry = self.registry

        def _registry_lookup_by_name(name: str):
            if registry is None:
                return None
            info = registry.get_by_lane(name)
            if info is None:
                return None
            lane_obj = self._get_lane_object(info.lane_name)
            if lane_obj is not None:
                trace.append(
                    f"registry lane {info.lane_name} (unit {info.unit_name}, extruder {info.extruder})"
                )
                return lane_obj
            trace.append(f"registry lane {info.lane_name} unresolved in printer objects")
            return None

        if resolved_name:
            trace.append(f"alias -> {resolved_name}")
            lane = self.lanes.get(resolved_name)
            if lane is not None:
                trace.append(f"found lane {lane.name} via alias")
                return lane, trace

            lane = _registry_lookup_by_name(resolved_name)
            if lane is not None:
                return lane, trace
        else:
            resolved_name = lane_name
            trace.append(f"no alias, using raw '{resolved_name}'")

        lane = self.lanes.get(resolved_name)
        if lane is not None:
            trace.append(f"matched direct lane {lane.name}")
            return lane, trace

        lowered = resolved_name.lower()
        for candidate_name, candidate in self.lanes.items():
            if candidate_name.lower() == lowered:
                trace.append(f"matched case-insensitive lane {candidate_name}")
                return candidate, trace

        registry_lane = _registry_lookup_by_name(resolved_name)
        if registry_lane is not None:
            return registry_lane, trace

        normalized_tool = _normalize_tool_token(resolved_name)
        trace.append(f"normalized tool token '{normalized_tool}'" if normalized_tool else "no tool token available")
        if normalized_tool:
            for candidate in self.lanes.values():
                candidate_tool = _normalize_tool_token(getattr(candidate, "map", None))
                if candidate_tool and candidate_tool == normalized_tool:
                    trace.append(
                        f"matched tool mapping '{getattr(candidate, 'map', None)}' -> lane {candidate.name}"
                    )
                    return candidate, trace

            if registry is not None:
                token = resolved_name if resolved_name.lower().startswith("t") else f"T{normalized_tool}"
                info = registry.resolve_lane_token(token)
                if info is not None:
                    lane = self._get_lane_object(info.lane_name)
                    if lane is not None:
                        trace.append(
                            f"registry token '{token}' -> lane {info.lane_name} (unit {info.unit_name})"
                        )
                        return lane, trace
                    trace.append(
                        f"registry token '{token}' found lane {info.lane_name} but object missing"
                    )

        trace.append("no lane resolved")
        return None, trace

    def _get_snapshot_lane_extruder(self, lane_name: str) -> Optional[str]:
        unit_name = None
        lane_obj = self._get_lane_object(lane_name)
        if lane_obj is not None:
            unit_name = getattr(lane_obj, "unit", None)

        lane_data = self._find_lane_snapshot(lane_name, unit_name=unit_name)
        if isinstance(lane_data, dict):
            return lane_data.get("extruder")
        return None

    def handle_runout_detected(self, spool_index: Optional[int], monitor=None, *, lane_name: Optional[str] = None) -> None:
        """Handle runout notifications coming from OpenAMS monitors."""
        lane = None
        if lane_name:
            lane = self.lanes.get(lane_name)
            if lane is None:
                lowered = lane_name.lower()
                lane = next((candidate for name, candidate in self.lanes.items() if name.lower() == lowered), None)
        if lane is None:
            lane = self._lane_for_spool_index(spool_index)
        if lane is None:
            return

        eventtime = self.reactor.monotonic()

        # Only handle the AMS same-extruder case here. Any other runout should fall back to
        # AFC's normal handling without OpenAMS interjection.
        runout_lane_name = self._canonical_lane_name(getattr(lane, "runout_lane", None))
        saved_runout_lane = self._get_saved_lane_runout_target(lane.name)
        runout_from_saved = False

        if not runout_lane_name and saved_runout_lane:
            runout_lane_name = saved_runout_lane
            runout_from_saved = True
            try:
                lane.runout_lane = runout_lane_name
            except Exception:
                self.logger.debug(f"Unable to write saved runout lane {runout_lane_name} onto {lane.name}")
        target_lane, handoff_trace = self._resolve_lane_reference_with_trace(runout_lane_name) if runout_lane_name else (None, [])
        same_extruder_handoff = False

        source_extruder = _normalize_extruder_name(self._get_snapshot_lane_extruder(lane.name))
        target_extruder = None
        if target_lane:
            target_extruder = _normalize_extruder_name(self._get_snapshot_lane_extruder(target_lane.name))

        if not source_extruder or (target_lane and not target_extruder):
            self.logger.error(
                f"Runout classification failed for {lane.name}: AFC.var.unit missing extruder data "
                f"(source={source_extruder}, target={target_extruder}); pausing for user intervention"
            )
            try:
                gcode = self.printer.lookup_object('gcode')
                gcode.run_script_from_command("PAUSE")
                gcode.respond_info(
                    f"Runout classification failed for {lane.name}: AFC.var.unit missing extruder data. "
                    "Please check AFC.var.unit and lane mappings."
                )
            except Exception:
                self.logger.error("Failed to issue PAUSE after runout classification failure")
            return

        self.logger.debug(
            f"Runout classification for {lane.name}: spool_index={spool_index}, "
            f"runout_lane={runout_lane_name} -> "
            f"{getattr(target_lane, 'name', None) if target_lane else None}, "
            f"source_extruder={source_extruder}, target_extruder={target_extruder}"
        )
        if runout_lane_name:
            self.logger.debug(
                f"Runout handoff resolution trace for {lane.name}: "
                f"{' > '.join(handoff_trace) if handoff_trace else '(no trace)'}"
            )

        if runout_lane_name and not target_lane and saved_runout_lane and saved_runout_lane != runout_lane_name:
            self.logger.info(
                f"Runout lane {runout_lane_name} for {lane.name} could not be resolved; "
                f"falling back to saved AFC state {saved_runout_lane}"
            )
            runout_lane_name = saved_runout_lane
            runout_from_saved = True
            target_lane, handoff_trace = self._resolve_lane_reference_with_trace(runout_lane_name)
            if target_lane:
                try:
                    lane.runout_lane = runout_lane_name
                except Exception:
                    self.logger.debug(f"Unable to persist saved runout lane {runout_lane_name} on {lane.name}")
        if runout_from_saved:
            self.logger.info(f"Resolved runout lane for {lane.name} from saved AFC.var.unit state: {runout_lane_name}")
        if target_lane:
            if source_extruder and target_extruder:
                if source_extruder == target_extruder:
                    same_extruder_handoff = True

        if not same_extruder_handoff:
            # Only block shared sensor transitions for same-FPS handoffs. For any other runout, keep
            # sensors flowing so AFC can perform its own infinite-runout logic just like Box Turtle
            # units.
            try:
                if hasattr(lane, '_oams_runout_detected'):
                    lane._oams_runout_detected = False
            except Exception:
                pass

            if runout_lane_name and not target_lane:
                self.logger.warning(
                    f"Runout handoff lane {runout_lane_name} not found for {lane.name}; "
                    "deferring to AFC's native runout handling"
                )
            elif target_lane:
                # Box Turtle infinite-runout behavior: when a handoff lane is configured on a
                # different extruder, trigger the infinite spool swap directly and let AFC resume
                # afterward instead of treating it as a same-FPS handoff.
                try:
                    lane._oams_cross_extruder_runout = True
                except Exception:
                    pass

                resolved_name = getattr(target_lane, "name", None)
                if resolved_name and resolved_name != getattr(lane, "runout_lane", None):
                    try:
                        lane.runout_lane = resolved_name
                    except Exception:
                        self.logger.debug(f"Could not set resolved runout lane on {lane.name}")
                perform_infinite = getattr(lane, "_perform_infinite_runout", None)
                if callable(perform_infinite):
                    try:
                        current_lane = getattr(self.afc, "current", None)
                        if current_lane != lane.name:
                            if current_lane not in getattr(self.afc, "lanes", {}):
                                self.logger.info(
                                    f"Cross-extruder runout: AFC current lane {current_lane} invalid; "
                                    f"overriding to {lane.name}"
                                )
                            try:
                                self.afc.current = lane.name
                            except Exception:
                                self.logger.debug(f"Unable to set AFC current lane to {lane.name} before infinite runout")
                        self.logger.info(
                            f"Cross-extruder runout: invoking infinite spool handoff from {lane.name} to {resolved_name}"
                        )
                        perform_infinite()
                        return
                    except Exception:
                        self.logger.error(
                            f"Failed infinite spool handoff for {lane.name} -> {resolved_name}; "
                            f"falling back to AFC runout handler\n{traceback.format_exc()}"
                        )
            else:
                try:
                    lane._oams_cross_extruder_runout = False
                except Exception:
                    pass

                self.logger.info(
                    f"Runout for {lane.name} does not target same extruder; letting AFC handle it normally"
                )

            # Allow AFC's built-in runout handler to pause or swap as configured
            extruder = getattr(lane, "extruder_obj", None)
            runout_cb = getattr(extruder, "handle_start_runout", None) if extruder is not None else None
            if callable(runout_cb):
                try:
                    runout_cb(eventtime)
                except TypeError:
                    runout_cb(eventtime=eventtime)
                except Exception:
                    self.logger.error(
                        f"Extruder runout handler failed for {lane.name} "
                        f"(spool_index={spool_index}, runout_lane={runout_lane_name})\n"
                        f"{traceback.format_exc()}"
                    )
            else:
                self.logger.warning(
                    f"Runout detected for {lane.name} but no extruder runout handler is available; "
                    "AFC must handle downstream"
                )
            return

        # Same-extruder runout: set the runout flag so shared sensor updates don't bounce the state.
        # The oams_manager runout monitor will detect the F1S=False via its own polling
        # and handle the reload sequence.
        try:
            if not hasattr(lane, '_oams_runout_detected'):
                lane._oams_runout_detected = False
            lane._oams_runout_detected = True
            lane._oams_cross_extruder_runout = False
            self.logger.info(
                f"Same-extruder runout: Marked lane {lane.name} for runout "
                "(oams_manager monitor will handle reload)"
            )
        except Exception:
            self.logger.error(f"Failed to mark lane {lane.name} for runout tracking")
    def handle_openams_lane_tool_state(self, lane_name: str, loaded: bool, *, spool_index: Optional[int] = None, eventtime: Optional[float] = None) -> bool:
        """Update lane/tool state in response to OpenAMS hardware events."""
        lane = self._resolve_lane_reference(lane_name)
        if lane is None:
            self.logger.warning(f"OpenAMS reported lane {lane_name} but AFC unit {self.name} cannot resolve it")
            return False

        if eventtime is None:
            try:
                eventtime = self.reactor.monotonic()
            except Exception:
                eventtime = 0.0

        lane_state = bool(loaded)
        try:
            self._apply_lane_sensor_state(lane, lane_state, eventtime)
        except Exception:
            self.logger.error(f"Failed to mirror OpenAMS lane sensor state for {lane.name}")
        if self.hardware_service is not None:
            hub_state = getattr(lane, "loaded_to_hub", None)
            tool_state = getattr(lane, "tool_loaded", None)
            mapped_spool = spool_index
            if mapped_spool is None:
                try:
                    mapped_spool = int(getattr(lane, "index", 0)) - 1
                except (TypeError, ValueError):
                    mapped_spool = None
            try:
                self.hardware_service.update_lane_snapshot(self.oams_name, lane.name, lane_state, hub_state if hub_state is not None else None, eventtime, spool_index=mapped_spool, tool_state=tool_state if tool_state is not None else lane_state)
            except Exception:
                self.logger.error(f"Failed to update shared lane snapshot for {lane.name}")
        afc_function = getattr(self.afc, "function", None)

        if lane_state:
            if afc_function is not None:
                try:
                    afc_function.unset_lane_loaded()
                except Exception:
                    self.logger.error("Failed to unset previously loaded lane")
            try:
                # Call set_tool_loaded() instead of set_loaded() since filament is loaded to toolhead
                # This properly sets extruder.lane_loaded which is needed for lane tracking
                lane.set_tool_loaded()
            except Exception:
                self.logger.error(f"Failed to mark lane {lane.name} as loaded")
            try:
                lane.sync_to_extruder()
                # Wait for all moves to complete to prevent "Timer too close" errors
                try:
                    toolhead = self.printer.lookup_object("toolhead")
                    toolhead.wait_moves()
                    # Add a small delay to allow the MCU to catch up
                    self.reactor.pause(self.reactor.monotonic() + 0.05)
                except Exception:
                    pass
            except Exception:
                self.logger.error(f"Failed to sync lane {lane.name} to extruder")
            if afc_function is not None:
                try:
                    afc_function.handle_activate_extruder()
                except Exception:
                    self.logger.error(f"Failed to activate extruder after loading lane {lane.name}")
            try:
                self.afc.save_vars()
            except Exception:
                self.logger.error("Failed to persist AFC state after lane load")
            try:
                self.select_lane(lane)
            except Exception:
                self.logger.debug(f"Unable to select lane {lane.name} during OpenAMS load")
            if self._lane_matches_extruder(lane):
                try:
                    canonical_lane = self._canonical_lane_name(lane.name)
                    force_update = True
                    if lane:
                        force_update = (getattr(lane, "tool_loaded", False) is not False)
                    self._set_virtual_tool_sensor_state(True, eventtime, lane.name, force=force_update, lane_obj=lane)
                except Exception:
                    self.logger.error(f"Failed to mirror tool sensor state for loaded lane {lane.name}")
            return True

        current_lane = None
        if afc_function is not None:
            try:
                current_lane = afc_function.get_current_lane_obj()
            except Exception:
                current_lane = None

        if current_lane is lane and afc_function is not None:
            try:
                afc_function.unset_lane_loaded()
            except Exception:
                self.logger.error(f"Failed to unset currently loaded lane {lane.name}")
            return True

        if getattr(lane, "tool_loaded", False):
            try:
                lane.unsync_to_extruder()
                # Wait for all moves to complete to prevent "Timer too close" errors
                try:
                    toolhead = self.printer.lookup_object("toolhead")
                    toolhead.wait_moves()
                    # Add a small delay to allow the MCU to catch up
                    self.reactor.pause(self.reactor.monotonic() + 0.05)
                except Exception:
                    pass
            except Exception:
                self.logger.error(f"Failed to unsync lane {lane.name} from extruder")
            try:
                lane.set_unloaded()
            except Exception:
                self.logger.error(f"Failed to mark lane {lane.name} as unloaded")
            try:
                self.afc.save_vars()
            except Exception:
                self.logger.error(f"Failed to persist AFC state after unloading lane {lane.name}")

        # Ensure extruder tracking is cleared even if lane.tool_loaded was already false
        extruder_obj = getattr(lane, "extruder_obj", None)
        try:
            extruder_lane = getattr(extruder_obj, "lane_loaded", None)
        except Exception:
            extruder_lane = None
        if extruder_obj is not None and extruder_lane == getattr(lane, "name", None):
            try:
                extruder_obj.lane_loaded = None
                self.logger.debug(f"Cleared extruder.lane_loaded for {getattr(lane, 'name', None)} during unload cleanup")
            except Exception:
                self.logger.debug(f"Failed to clear extruder tracking for {getattr(lane, 'name', None)} during unload cleanup")
        if self._lane_matches_extruder(lane):
            try:
                self._set_virtual_tool_sensor_state(False, eventtime, lane.name, lane_obj=lane)
            except Exception:
                self.logger.error(f"Failed to mirror tool sensor state for unloaded lane {lane.name}")
        return True

    def mark_cross_extruder_runout(self, lane_name: str) -> bool:
        """Mark a lane as cross-extruder for shared sensor handling."""

        lane = self._resolve_lane_reference(lane_name)
        if lane is None:
            self.logger.warning(
                f"Requested cross-extruder runout mark for {lane_name} but AFC unit {self.name} cannot resolve it"
            )
            return False

        try:
            if not hasattr(lane, "_oams_runout_detected"):
                lane._oams_runout_detected = False
            lane._oams_cross_extruder_runout = True
            self.logger.info(
                f"Marked lane {lane.name} as cross-extruder runout participant for shared sensor bypass"
            )
            return True
        except Exception:
            self.logger.error(f"Failed to mark lane {lane.name} for cross-extruder runout")
            return False

    def _is_event_for_unit(self, unit_name: Optional[str]) -> bool:
        """Check whether an event payload targets this unit."""
        if not unit_name:
            return False

        candidates = {str(self.name).lower()}
        if getattr(self, "oams_name", None):
            candidates.add(str(self.oams_name).lower())

        return str(unit_name).lower() in candidates

    def _handle_spool_loaded_event(self, *, event_type=None, **kwargs):
        """Update local state in response to a spool_loaded event."""
        unit_name = kwargs.get("unit_name")
        if not self._is_event_for_unit(unit_name):
            return

        spool_index = kwargs.get("spool_index")
        try:
            normalized_index = int(spool_index) if spool_index is not None else None
        except (TypeError, ValueError):
            normalized_index = None

        lane = self._find_lane_by_spool(normalized_index)
        if lane is None:
            return

        # PHASE 2 REFACTOR: Use AFC native lane.load_state instead of dictionary
        previous_loaded = bool(getattr(lane, "load_state", False))

        eventtime = kwargs.get("eventtime", 0.0)

        # PHASE 1 REFACTOR: Use AFC native set_loaded() instead of direct state assignment
        # lane.set_loaded() handles:
        # - Sets lane.status = AFCLaneState.LOADED
        # - Calls self.unit_obj.lane_loaded(self) for LED updates
        # - Calls self.afc.spool._set_values(self) for spool metadata
        # This eliminates manual state management and ensures proper state transitions
        lane.set_loaded()

        # Schedule TD-1 capture with 3-second delay if td1_when_loaded is enabled
        # The delay allows AMS auto-load sequence to complete (loads near hub ? retracts ? settles)
        if not previous_loaded and getattr(lane, "td1_when_loaded", False):
            lane_name = lane.name
            try:
                # Cancel any existing pending timer for this lane
                if lane_name in self._pending_spool_loaded_timers:
                    try:
                        old_timer = self._pending_spool_loaded_timers[lane_name]
                        self.reactor.unregister_timer(old_timer)
                    except Exception:
                        pass  # Timer may have already fired

                # Register new timer with 3-second delay for TD-1 capture
                timer_callback = self._trigger_td1_capture_delayed(lane_name)
                timer = self.reactor.register_timer(timer_callback, self.reactor.monotonic() + 3.0)
                self._pending_spool_loaded_timers[lane_name] = timer

                self.logger.info(f"Scheduled TD-1 capture for {lane_name} in 3 seconds (allowing AMS to settle)")
            except Exception as e:
                self.logger.error(f"Failed to schedule TD-1 capture for {lane_name}: {e}")
        extruder_name = getattr(lane, "extruder_name", None)
        if extruder_name is None and self.registry is not None:
            try:
                extruder_name = self.registry.resolve_extruder(lane.name)
            except Exception:
                extruder_name = None

        self.record_load(extruder=extruder_name, lane_name=lane.name)

        if self.hardware_service is not None and normalized_index is not None:
            hub_state = getattr(lane, "loaded_to_hub", None)
            tool_state = getattr(lane, "tool_loaded", None)
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name,
                    lane.name,
                    True,
                    hub_state if hub_state is not None else None,
                    eventtime,
                    spool_index=normalized_index,
                    tool_state=tool_state if tool_state is not None else None,
                    emit_spool_event=False,
                )
            except Exception:
                self.logger.error(f"Failed to mirror spool load event for {lane.name}")
    def _handle_spool_unloaded_event(self, *, event_type=None, **kwargs):
        """Update local state in response to a spool_unloaded event."""
        unit_name = kwargs.get("unit_name")
        if not self._is_event_for_unit(unit_name):
            return

        spool_index = kwargs.get("spool_index")
        try:
            normalized_index = int(spool_index) if spool_index is not None else None
        except (TypeError, ValueError):
            normalized_index = None

        lane = self._find_lane_by_spool(normalized_index)
        if lane is None:
            return

        # PHASE 1 REFACTOR: Use AFC native set_unloaded() instead of direct state assignments
        # lane.set_unloaded() handles:
        # - Sets lane.tool_loaded = False
        # - Sets lane.status = AFCLaneState.NONE
        # - Sets lane.loaded_to_hub = False
        # - Clears lane.td1_data = {}
        # - Calls self.afc.spool.clear_values(self)
        # - Calls self.unit_obj.lane_unloaded(self)
        # This eliminates manual state management and ensures proper cleanup
        #
        # PHASE 2 REFACTOR: Removed dictionary updates
        # AFC native lane.set_unloaded() handles all state cleanup
        lane.set_unloaded()

        eventtime = kwargs.get("eventtime", 0.0)
        if self.hardware_service is not None and normalized_index is not None:
            try:
                self.hardware_service.update_lane_snapshot(
                    self.oams_name,
                    lane.name,
                    False,
                    False,
                    eventtime,
                    spool_index=normalized_index,
                    tool_state=False,
                    emit_spool_event=False,
                )
            except Exception:
                self.logger.error(f"Failed to mirror spool unload event for {lane.name}")
    def cmd_AFC_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        """Run the OpenAMS HUB HES calibration for a specific lane."""
        spool_index = gcmd.get_int("SPOOL", None)
        if spool_index is None:
            gcmd.respond_info("SPOOL parameter is required for OpenAMS HUB HES calibration.")
            return

        lane = self._find_lane_by_spool(spool_index)
        if lane is None:
            gcmd.respond_info(f"Could not find lane for spool index {spool_index}.")
            return

        # Check if this lane's extruder has something loaded to toolhead
        extruder_name = getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None
        loaded_lane = self._check_toolhead_loaded(extruder_name)
        if loaded_lane:
            gcmd.respond_info(f"Cannot run OpenAMS calibration while {loaded_lane} is loaded to the toolhead on this extruder. Please unload the tool and try again.")
            return

        lane_name = getattr(lane, "name", None)
        self._calibrate_hub_hes_spool(spool_index, gcmd, lane_name=lane_name)

    def cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL(self, gcmd):
        """Calibrate HUB HES for every loaded OpenAMS lane in this unit."""
        # Check if any lane on THIS UNIT has something loaded to toolhead
        for lane in self.lanes.values():
            if getattr(lane, "tool_loaded", False):
                gcmd.respond_info(f"Cannot run OpenAMS calibration while {lane.name} is loaded to the toolhead. Please unload the tool and try again.")
                return

        prompt = AFCprompt(gcmd, self.logger)
        prompt.p_end()

        calibrations = []
        skipped = []
        for lane in self.lanes.values():
            if not getattr(lane, "load_state", False):
                continue
            spool_index = self._get_openams_spool_index(lane)
            if spool_index is None:
                skipped.append(getattr(lane, "name", str(lane)))
                continue
            calibrations.append((lane, spool_index))

        if not calibrations:
            gcmd.respond_info("No loaded OpenAMS lanes were found to calibrate HUB HES values.")
            return

        successful = 0
        for lane, spool_index in calibrations:
            if self._calibrate_hub_hes_spool(spool_index, gcmd, lane_name=getattr(lane, "name", None)):
                successful += 1

        gcmd.respond_info(f"Completed HUB HES calibration for {successful} OpenAMS lane(s).")

        if skipped:
            skipped_lanes = ", ".join(skipped)
            gcmd.respond_info("Skipped HUB HES calibration for lanes lacking OpenAMS mapping: {}.".format(skipped_lanes))

    def cmd_AFC_OAMS_CALIBRATE_PTFE(self, gcmd):
        """Run the OpenAMS PTFE calibration for a specific lane."""
        spool_index = gcmd.get_int("SPOOL", None)
        if spool_index is None:
            gcmd.respond_info("SPOOL parameter is required for OpenAMS PTFE calibration.")
            return

        lane = self._find_lane_by_spool(spool_index)
        if lane is None:
            gcmd.respond_info(f"Could not find lane for spool index {spool_index}.")
            return

        # Check if this lane's extruder has something loaded to toolhead
        extruder_name = getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None
        loaded_lane = self._check_toolhead_loaded(extruder_name)
        if loaded_lane:
            gcmd.respond_info(f"Cannot run OpenAMS calibration while {loaded_lane} is loaded to the toolhead on this extruder. Please unload the tool and try again.")
            return

        lane_name = getattr(lane, "name", None)
        self._calibrate_ptfe_spool(spool_index, gcmd, lane_name=lane_name)

    def _calibrate_hub_hes_spool(self, spool_index, gcmd, lane_name=None):
        oams_index = self._get_openams_index()
        if oams_index is None:
            gcmd.respond_info("Unable to determine OpenAMS index for HUB HES calibration.")
            return False

        command = f"OAMS_CALIBRATE_HUB_HES OAMS={oams_index} SPOOL={spool_index}"
        lane_label = lane_name or f"spool {spool_index}"
        gcmd.respond_info(f"Running HUB HES calibration for {lane_label} with '{command}'.")

        try:
            messages = self._run_command_with_capture(command)
        except Exception:
            self.logger.error(f"Failed to execute OpenAMS HUB HES calibration for spool {spool_index}")
            gcmd.respond_info(f"Failed to execute HUB HES calibration for {lane_label}. See logs.")
            return False

        hub_values = self._parse_hub_hes_messages(messages)
        if not hub_values:
            gcmd.respond_info(f"Completed {command} but no HUB HES value was reported. Check OpenAMS status logs.")
            return False

        config_values = self._read_config_sequence("hub_hes_on")
        if not config_values:
            config_values = self._last_hub_hes_values or []

        if not config_values:
            gcmd.respond_info("Could not find hub_hes_on in your cfg; update the value manually.")
            return False

        values = list(config_values)
        max_length = len(values)
        updated_indices = []
        for index, parsed_value in sorted(hub_values.items()):
            if index >= max_length:
                gcmd.respond_info("HUB HES calibration reported index {} but your cfg only defines {} value(s); update the remaining entries manually.".format(index, max_length))
                continue
            values[index] = parsed_value
            updated_indices.append(index)

        if not updated_indices:
            gcmd.respond_info("Completed {} but no HUB HES value was stored; check your cfg.".format(command))
            return False

        formatted = self._format_sequence(values)
        if not formatted:
            gcmd.respond_info("Unable to format HUB HES calibration values.")
            return False

        if not self._write_config_value("hub_hes_on", formatted):
            gcmd.respond_info("Failed to update hub_hes_on in your cfg; please update it manually.")
            return False

        self._last_hub_hes_values = values

        if updated_indices:
            if len(updated_indices) == 1:
                index_text = f"index {updated_indices[0]}"
            else:
                index_text = "indices " + ", ".join(str(i) for i in updated_indices)
            gcmd.respond_info(f"Stored OpenAMS hub_hes_on {formatted} in your cfg (updated {index_text}).")
        else:
            gcmd.respond_info(f"Stored OpenAMS hub_hes_on {formatted} in your cfg.")
        return True

    def _calibrate_ptfe_spool(self, spool_index, gcmd, lane_name=None):
        oams_index = self._get_openams_index()
        if oams_index is None:
            gcmd.respond_info("Unable to determine OpenAMS index for PTFE calibration.")
            return False

        command = f"OAMS_CALIBRATE_PTFE_LENGTH OAMS={oams_index} SPOOL={spool_index}"
        lane_label = lane_name or f"spool {spool_index}"
        gcmd.respond_info(f"Running PTFE calibration for {lane_label} with '{command}'.")

        try:
            messages = self._run_command_with_capture(command)
        except Exception:
            self.logger.error(f"Failed to execute OpenAMS PTFE calibration for spool {spool_index}")
            gcmd.respond_info(f"Failed to execute PTFE calibration for {lane_label}. See logs.")
            return False

        captured = self._parse_ptfe_messages(messages)
        value = None
        if captured:
            if 0 <= spool_index < len(captured):
                value = captured[spool_index]
            elif len(captured) == 1:
                value = captured[0]

        if value is None:
            gcmd.respond_info(f"Completed {command} but no PTFE length was reported. Check OpenAMS status logs.")
            return False

        formatted_value = self._format_numeric(value)
        if formatted_value is None:
            gcmd.respond_info("Unable to format PTFE calibration value for config storage.")
            return False

        if not self._write_config_value("ptfe_length", formatted_value):
            gcmd.respond_info("Failed to update ptfe_length in your cfg; please update it manually.")
            return False

        target_name = lane_label
        gcmd.respond_info(f"Stored OpenAMS ptfe_length {formatted_value} for {target_name} in your cfg.")
        return True

    def _run_command_with_capture(self, command):
        captured: List[str] = []
        original = getattr(self.gcode, "respond_info", None)

        if original is None:
            self.gcode.run_script_from_command(command)
            return captured

        def _capture(this, message, *args, **kwargs):
            if isinstance(message, str):
                captured.append(message)
            return original(message, *args, **kwargs)

        self.gcode.respond_info = MethodType(_capture, self.gcode)
        try:
            self.gcode.run_script_from_command(command)
        finally:
            self.gcode.respond_info = original

        return captured

    def _parse_hub_hes_messages(self, messages):
        results = {}
        pattern = re.compile(r"HES\s*([0-9]+)\D+(-?[0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

        for message in messages or []:
            if not isinstance(message, str):
                continue
            for match in pattern.finditer(message):
                try:
                    index = int(match.group(1))
                    value = float(match.group(2))
                except (TypeError, ValueError):
                    continue
                results[index] = value

        return results

    def _parse_ptfe_messages(self, messages):
        values = []
        pattern = re.compile(r"(?:ptfe|bowden)[^0-9\-]*(-?[0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

        for message in messages or []:
            if not isinstance(message, str):
                continue
            for match in pattern.finditer(message):
                try:
                    values.append(float(match.group(1)))
                except (TypeError, ValueError):
                    continue

        return values

    def _format_numeric(self, value):
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None

        if abs(number - round(number)) <= 1e-6:
            return str(int(round(number)))

        return f"{number:.6f}".rstrip("0").rstrip(".")

    def _format_sequence(self, values):
        if values is None:
            return None

        formatted = []
        for value in list(values):
            formatted_value = self._format_numeric(value)
            if formatted_value is None:
                continue
            formatted.append(formatted_value)

        return ", ".join(formatted) if formatted else None

    def _config_section_name(self):
        name = getattr(self, "oams_name", None)
        if not name:
            return None
        return f"oams {name}"

    def _read_config_sequence(self, key):
        section = self._config_section_name()
        config_dir = getattr(self.afc, "cfgloc", None)
        if not section or not config_dir:
            return None

        header = f"[{section}]".strip().lower()
        key_pattern = re.compile(rf"^{re.escape(key)}\s*:\s*(.+)$", re.IGNORECASE)

        try:
            filenames = sorted(filename for filename in os.listdir(config_dir) if filename.lower().endswith(".cfg"))
        except OSError:
            return None

        for filename in filenames:
            path = os.path.join(config_dir, filename)
            try:
                with open(path, "r", encoding="utf-8") as cfg_file:
                    in_section = False
                    for line in cfg_file:
                        stripped = line.strip()
                        if not stripped:
                            continue
                        if stripped.startswith("[") and stripped.endswith("]"):
                            in_section = stripped.lower() == header
                            continue
                        if not in_section:
                            continue
                        match = key_pattern.match(stripped)
                        if not match:
                            continue
                        value_part = match.group(1)
                        if "#" in value_part:
                            value_part = value_part.split("#", 1)[0]
                        raw_value = value_part.strip()
                        return self._parse_sequence_string(raw_value)
            except OSError:
                continue

        return None

    def _parse_sequence_string(self, raw_value):
        if raw_value is None:
            return []

        values = []
        for token in raw_value.split(","):
            token = token.strip()
            if not token:
                continue
            try:
                values.append(float(token))
            except (TypeError, ValueError):
                continue

        return values

    def _write_config_value(self, key, value):
        section = self._config_section_name()
        afc_function = getattr(self.afc, "function", None)
        if not section or afc_function is None:
            return False

        rewrite = getattr(afc_function, "ConfigRewrite", None)
        if not callable(rewrite):
            return False

        msg = f"\n{self.name} {key}: Saved {value}"
        try:
            rewrite(section, key, value, msg)
        except Exception:
            self.logger.error(f"Failed to persist {key} for OpenAMS unit {self.name}")
            return False

        return True

    def _check_toolhead_loaded(self, extruder_name=None):
        """Check if a lane is currently loaded to the specified extruder's toolhead.

        Args:
            extruder_name: Optional extruder name to check. If None, checks all extruders.

        Returns: Lane name if loaded, None otherwise.
        """
        # If specific extruder provided, only check lanes on that extruder
        if extruder_name:
            for lane_name, lane in self.afc.lanes.items():
                if getattr(lane, "tool_loaded", False):
                    lane_extruder = getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None
                    if lane_extruder == extruder_name:
                        return lane_name
        else:
            # Check all lanes across all AFC units
            for lane_name, lane in self.afc.lanes.items():
                if getattr(lane, "tool_loaded", False):
                    return lane_name
        return None

    def _find_lane_by_spool(self, spool_index):
        """Resolve lane by spool index using registry when available."""
        if spool_index is None:
            return None

        try:
            normalized = int(spool_index)
        except (TypeError, ValueError):
            return None

        registry_unit = self.oams_name or self.name
        if self.registry is not None:
            lane_info = self.registry.get_by_spool(registry_unit, normalized)
            if lane_info is not None:
                lane = self.lanes.get(lane_info.lane_name)
                if lane is not None:
                    return lane

        return self._lane_by_local_index(normalized)

    def _lane_by_local_index(self, normalized: int):
        for candidate in self.lanes.values():
            lane_index = getattr(candidate, "index", None)
            try:
                lane_index = int(lane_index) - 1
            except (TypeError, ValueError):
                continue

            if lane_index == normalized:
                return candidate

        return None

    def _get_openams_index(self):
        """Helper to extract OAMS index (OPTIMIZED with caching)."""
        # OPTIMIZATION: Cache OAMS index after first lookup
        if self._cached_oams_index is not None:
            return self._cached_oams_index

        if self.oams is not None:
            oams_idx = getattr(self.oams, "oams_idx", None)
            if oams_idx is not None:
                self._cached_oams_index = oams_idx
                return oams_idx
        return None

    def _get_openams_spool_index(self, lane):
        """Helper to extract spool index from lane."""
        try:
            return int(getattr(lane, "index", 0)) - 1
        except (TypeError, ValueError):
            return None

    def check_runout(self, lane=None):
        if lane is None:
            return False
        if getattr(lane, "unit_obj", None) is not self:
            return False
        try:
            is_printing = self.afc.function.is_printing()
        except Exception:
            is_printing = False
        return bool(is_printing)

    def _register_sync_dispatcher(self) -> None:
        """Ensure the shared sync command is available for all AMS units."""
        cls = self.__class__
        if not cls._sync_command_registered:
            self.gcode.register_command(
                "AFC_AMS_SYNC_TOOL_SENSOR",
                cls._dispatch_sync_tool_sensor,
                desc=self.cmd_SYNC_TOOL_SENSOR_help,
            )
            cls._sync_command_registered = True

        cls._sync_instances[self.name] = self

    @classmethod
    def _extract_raw_param(cls, commandline: str, key: str) -> Optional[str]:
        """Recover multi-word parameter values from the raw command line."""
        if not commandline:
            return None

        key_upper = key.upper() + "="
        command_upper = commandline.upper()
        start = command_upper.find(key_upper)
        if start == -1:
            return None

        start += len(key_upper)
        remainder = commandline[start:]
        match = re.search(r"\s[A-Z0-9_]+=|;", remainder)
        end = start + match.start() if match else len(commandline)

        value = commandline[start:end].strip()
        if not value:
            return None

        if value[0] in ('\'', '"') and value[-1] == value[0]:
            value = value[1:-1]

        return value

    @classmethod
    def _dispatch_sync_tool_sensor(cls, gcmd):
        """Route sync requests to the correct AMS instance, tolerating spaces."""
        unit_value = gcmd.get("UNIT", None)
        if not unit_value:
            unit_value = cls._extract_raw_param(gcmd.get_commandline(), "UNIT")

        lane_name = gcmd.get("LANE", None)
        if lane_name is None:
            lane_name = gcmd.get("FPS", None)

        if lane_name is None:
            commandline = gcmd.get_commandline()
            lane_name = cls._extract_raw_param(commandline, "LANE")
            if lane_name is None:
                lane_name = cls._extract_raw_param(commandline, "FPS")

        for instance in cls._sync_instances.values():
            if not instance._unit_matches(unit_value):
                continue

            resolved_lane = instance._resolve_lane_alias(lane_name)
            eventtime = instance.reactor.monotonic()
            instance._sync_virtual_tool_sensor(eventtime, resolved_lane)

def _patch_lane_pre_sensor_for_ams() -> None:
    """Patch AFCLane.get_toolhead_pre_sensor_state for AMS virtual sensors."""
    if _ORIGINAL_LANE_PRE_SENSOR is None:
        return

    if getattr(AFCLane, "_ams_pre_sensor_patched", False):
        return

    def _ams_get_toolhead_pre_sensor_state(self, *args, **kwargs):
        unit = getattr(self, "unit_obj", None)
        if not isinstance(unit, afcAMS):
            return _ORIGINAL_LANE_PRE_SENSOR(self, *args, **kwargs)

        reactor = getattr(unit, "reactor", None)
        eventtime = None
        if reactor is not None:
            try:
                eventtime = reactor.monotonic()
            except Exception:
                eventtime = None

        if eventtime is None:
            eventtime = 0.0

        # Sync virtual tool sensor state (sensor state is kept up-to-date via events)
        try:
            unit._sync_virtual_tool_sensor(eventtime, self.name)
        except Exception:
            pass

        result = _ORIGINAL_LANE_PRE_SENSOR(self, *args, **kwargs)

        if result:
            return bool(result)

        desired_state = unit._lane_reports_tool_filament(self)
        if desired_state is None:
            desired_state = False

        if desired_state:
            try:
                unit._set_virtual_tool_sensor_state(desired_state, eventtime, getattr(self, "name", None), lane_obj=self)
            except Exception:
                pass
            return True

        return bool(result)

    AFCLane.get_toolhead_pre_sensor_state = _ams_get_toolhead_pre_sensor_state
    AFCLane._ams_pre_sensor_patched = True


def _patch_infinite_runout_handler() -> None:
    """Harden AFCLane infinite runout handling without touching AFC_lane.py."""

    if getattr(AFCLane, "_ams_infinite_runout_patched", False):
        return

    if not callable(_ORIGINAL_PERFORM_INFINITE_RUNOUT):
        return

    def _ams_perform_infinite_runout(self, *args, **kwargs):
        lane_name = getattr(self, "name", "unknown")

        afc = getattr(self, "afc", None)
        lanes = getattr(afc, "lanes", {}) if afc is not None else {}

        if not afc or not lanes:
            raise RuntimeError(f"AFC context unavailable for infinite runout on {lane_name}")

        def _normalize_target(target):
            """Resolve lane names, aliases (T#), or maps to concrete lane keys."""
            if target is None:
                return None

            lookup = str(target).strip()
            if not lookup:
                return None

            lookup_lower = lookup.lower()
            # Direct name match
            for key in lanes:
                if str(key).lower() == lookup_lower:
                    return key

            # Match against lane.map (e.g., T0, t0)
            for key, lane_obj in lanes.items():
                lane_map = getattr(lane_obj, "map", None)
                if isinstance(lane_map, str) and lane_map.strip().lower() == lookup_lower:
                    return key

            # Match T# aliases to lane indices when possible
            if lookup_lower.startswith("t") and lookup_lower[1:].isdigit():
                try:
                    idx = int(lookup_lower[1:])
                    for key, lane_obj in lanes.items():
                        lane_idx = getattr(lane_obj, "lane", None)
                        if lane_idx is not None and int(lane_idx) == idx:
                            return key
                except Exception:
                    pass

            return None

        def _normalize_current(cur):
            if cur in lanes:
                return cur
            try:
                if hasattr(cur, "name") and getattr(cur, "name", None) in lanes:
                    return getattr(cur, "name", None)
            except Exception:
                pass
            return None

        raw_runout_target = getattr(self, "runout_lane", None)
        runout_target = _normalize_target(raw_runout_target)

        if not runout_target:
            raise RuntimeError(
                f"Runout lane {raw_runout_target} unavailable for {lane_name} (known lanes: {', '.join(lanes)})"
            )

        if raw_runout_target != runout_target:
            try:
                self.runout_lane = runout_target
                self.logger.info(f"Normalized runout lane {raw_runout_target} -> {runout_target} for infinite runout")
            except Exception:
                pass

        if runout_target not in lanes:
            raise RuntimeError(f"Runout target lane {runout_target} missing for infinite runout from {lane_name}")

        normalized_current = _normalize_current(getattr(afc, "current", None))
        if normalized_current is None or normalized_current != lane_name:
            try:
                afc.current = lane_name
                normalized_current = lane_name
                self.logger.debug(f"Setting AFC current lane to {lane_name} before infinite runout")
            except Exception:
                normalized_current = normalized_current or lane_name

        empty_lane = lanes.get(normalized_current, self)
        change_lane = lanes.get(runout_target)

        self.logger.debug(
            f"Invoking AFC infinite runout from {lane_name} "
            f"(map={getattr(empty_lane, 'map', None)}, extruder={getattr(empty_lane, 'extruder', None)}) "
            f"to {getattr(change_lane, 'name', None)} "
            f"(map={getattr(change_lane, 'map', None)}, extruder={getattr(change_lane, 'extruder', None)}) "
            f"with current={getattr(afc, 'current', None)}"
        )

        if not callable(_ORIGINAL_PERFORM_INFINITE_RUNOUT):
            raise RuntimeError("Original AFC infinite runout handler is unavailable")

        step = "delegate"
        try:
            return _ORIGINAL_PERFORM_INFINITE_RUNOUT(self, *args, **kwargs)
        except Exception:
            self.logger.error(
                f"Infinite runout failed at step {step} for {lane_name} -> {runout_target} "
                f"(current={getattr(afc, 'current', None)}, empty_map={getattr(empty_lane, 'map', None)}, "
                f"target_map={getattr(change_lane, 'map', None)}, empty_extruder={getattr(empty_lane, 'extruder', None)}, "
                f"target_extruder={getattr(change_lane, 'extruder', None)})"
            )
            raise

    AFCLane._perform_infinite_runout = _ams_perform_infinite_runout
    AFCLane._ams_infinite_runout_patched = True

def _patch_lane_unload_for_ams() -> None:
    """Block LANE_UNLOAD for OpenAMS lanes to prevent Klipper hangs.

    OpenAMS units manage filament automatically via hardware and don't support
    manual lane ejection like Box Turtle units. Attempting manual ejection causes
    the command to hang waiting for operations that won't complete properly.
    """
    global _ORIGINAL_LANE_UNLOAD

    # Import here to avoid circular dependencies
    try:
        from extras.AFC import afc as AFC_Class
    except Exception:
        # If we can't import AFC, we can't patch it
        return

    if getattr(AFC_Class, "_ams_lane_unload_patched", False):
        return

    # Save original LANE_UNLOAD method
    _ORIGINAL_LANE_UNLOAD = getattr(AFC_Class, "LANE_UNLOAD", None)
    if not callable(_ORIGINAL_LANE_UNLOAD):
        return

    def _ams_lane_unload(self, cur_lane):
        """Patched LANE_UNLOAD that blocks manual ejection on OpenAMS lanes."""
        # Check if this is an OpenAMS lane
        unit_obj = getattr(cur_lane, 'unit_obj', None)
        if unit_obj is not None:
            unit_type = getattr(unit_obj, 'type', None)
            has_oams_name = hasattr(unit_obj, 'oams_name')

            if unit_type == "OpenAMS" or has_oams_name:
                # Block LANE_UNLOAD for OpenAMS lanes
                lane_name = getattr(cur_lane, 'name', 'unknown')
                self.logger.info(
                    f"LANE_UNLOAD is not supported for OpenAMS lane {lane_name}. "
                    f"OpenAMS units handle filament automatically - just remove the spool physically. "
                    f"Use TOOL_UNLOAD if you need to unload from the toolhead."
                )

                # Try to respond to user via gcode
                try:
                    gcode = self.gcode or self.printer.lookup_object("gcode")
                    if gcode:
                        gcode.respond_info(
                            f"LANE_UNLOAD is not supported for OpenAMS lanes like {lane_name}. "
                            f"OpenAMS units handle filament automatically - just remove the spool physically. "
                            f"Use TOOL_UNLOAD if you need to unload from the toolhead."
                        )
                except Exception:
                    pass

                return  # Block the operation

        # Not an OpenAMS lane - call original LANE_UNLOAD
        if callable(_ORIGINAL_LANE_UNLOAD):
            return _ORIGINAL_LANE_UNLOAD(self, cur_lane)

    AFC_Class.LANE_UNLOAD = _ams_lane_unload
    AFC_Class._ams_lane_unload_patched = True

def _patch_buffer_multiplier_for_ams() -> None:
    """Guard buffer multiplier updates when OpenAMS lanes lack an extruder stepper."""
    global _ORIGINAL_BUFFER_SET_MULTIPLIER

    try:
        from extras.AFC_buffer import AFCTrigger
    except Exception:
        return

    if getattr(AFCTrigger, "_ams_buffer_multiplier_patched", False):
        return

    _ORIGINAL_BUFFER_SET_MULTIPLIER = getattr(AFCTrigger, "set_multiplier", None)
    if not callable(_ORIGINAL_BUFFER_SET_MULTIPLIER):
        return

    def _ams_set_multiplier(self, multiplier):
        cur_lane = self.afc.function.get_current_lane_obj()
        if cur_lane is None:
            return _ORIGINAL_BUFFER_SET_MULTIPLIER(self, multiplier)

        unit_obj = getattr(cur_lane, "unit_obj", None)
        unit_type = getattr(unit_obj, "type", None) if unit_obj is not None else None
        is_openams = unit_type == "OpenAMS" or hasattr(unit_obj, "oams_name")
        if not is_openams:
            return _ORIGINAL_BUFFER_SET_MULTIPLIER(self, multiplier)

        extruder_stepper = getattr(cur_lane, "extruder_stepper", None)
        stepper = getattr(extruder_stepper, "stepper", None) if extruder_stepper is not None else None
        if is_openams and stepper is None:
            self.logger.debug(
                "Skipping buffer multiplier update for OpenAMS lane {} (no extruder stepper)".format(
                    cur_lane.name
                )
            )
            return None

        return _ORIGINAL_BUFFER_SET_MULTIPLIER(self, multiplier)

    AFCTrigger.set_multiplier = _ams_set_multiplier
    AFCTrigger._ams_buffer_multiplier_patched = True

def _patch_buffer_status_for_missing_stepper() -> None:
    """Guard buffer status reporting when a lane lacks an extruder stepper."""
    global _ORIGINAL_BUFFER_GET_STATUS

    try:
        from extras.AFC_buffer import AFCTrigger
    except Exception:
        return

    if getattr(AFCTrigger, "_ams_buffer_status_patched", False):
        return

    _ORIGINAL_BUFFER_GET_STATUS = getattr(AFCTrigger, "get_status", None)
    if not callable(_ORIGINAL_BUFFER_GET_STATUS):
        return

    def _ams_get_status(self, eventtime=None):
        cur_lane = self.afc.function.get_current_lane_obj()
        unit_obj = getattr(cur_lane, "unit_obj", None) if cur_lane is not None else None
        unit_type = getattr(unit_obj, "type", None) if unit_obj is not None else None
        is_openams = unit_type == "OpenAMS" or hasattr(unit_obj, "oams_name")
        if not is_openams:
            return _ORIGINAL_BUFFER_GET_STATUS(self, eventtime)
        try:
            return _ORIGINAL_BUFFER_GET_STATUS(self, eventtime)
        except AttributeError as exc:
            if "stepper" not in str(exc):
                raise

        response = {}
        response["state"] = self.last_state
        response["lanes"] = [lane.name for lane in self.lanes.values()]
        response["enabled"] = self.enable
        response["rotation_distance"] = None
        response["fault_detection_enabled"] = self.error_sensitivity > 0
        response["error_sensitivity"] = self.error_sensitivity
        response["fault_timer"] = self.fault_timer

        if self.error_sensitivity > 0 and self.filament_error_pos is not None:
            current_pos = self.get_extruder_pos()
            if current_pos is not None:
                response["distance_to_fault"] = self.filament_error_pos - current_pos
                response["filament_error_pos"] = self.filament_error_pos
                response["current_pos"] = current_pos
            else:
                response["distance_to_fault"] = None
        else:
            response["distance_to_fault"] = None

        return response

    AFCTrigger.get_status = _ams_get_status
    AFCTrigger._ams_buffer_status_patched = True

def _patch_buffer_fault_detection_for_ams() -> None:
    """Skip buffer fault detection timers for OpenAMS lanes."""
    global _ORIGINAL_BUFFER_EXTRUDER_POS_UPDATE

    try:
        import extras.AFC_buffer as _afc_buffer_mod
    except Exception:
        return

    AFCTrigger = getattr(_afc_buffer_mod, "AFCTrigger", None)
    if AFCTrigger is None:
        return

    if getattr(AFCTrigger, "_ams_buffer_fault_patched", False):
        return

    _ORIGINAL_BUFFER_EXTRUDER_POS_UPDATE = getattr(AFCTrigger, "extruder_pos_update_event", None)
    if not callable(_ORIGINAL_BUFFER_EXTRUDER_POS_UPDATE):
        return

    timeout = getattr(_afc_buffer_mod, "CHECK_RUNOUT_TIMEOUT", 0.5)

    def _ams_extruder_pos_update_event(self, eventtime):
        cur_lane = self.afc.function.get_current_lane_obj()
        if cur_lane is None:
            return _ORIGINAL_BUFFER_EXTRUDER_POS_UPDATE(self, eventtime)

        unit_obj = getattr(cur_lane, "unit_obj", None)
        if unit_obj is None:
            unit_name = getattr(cur_lane, "unit", None)
            units = getattr(self.afc, "units", {})
            unit_obj = units.get(unit_name) if unit_name else None
        unit_type = getattr(unit_obj, "type", None) if unit_obj is not None else None
        is_openams = unit_type == "OpenAMS" or hasattr(unit_obj, "oams_name")
        if is_openams:
            return eventtime + timeout

        return _ORIGINAL_BUFFER_EXTRUDER_POS_UPDATE(self, eventtime)

    AFCTrigger.extruder_pos_update_event = _ams_extruder_pos_update_event
    AFCTrigger._ams_buffer_fault_patched = True

def _has_openams_hardware(printer):
    """Check if any OpenAMS hardware is configured in the system.

    Returns True if any [oams ...] sections are found in the configuration.
    This prevents unnecessary OpenAMS initialization for users with other unit types.
    """
    try:
        # Try to find any OAMS objects in the printer
        # This will work after all configs have been loaded during handle_ready
        for obj_name in printer.objects:
            if obj_name.startswith('oams '):
                return True
        return False
    except Exception:
        # If we can't check, assume OAMS might be present to avoid breaking existing setups
        return True


def load_config_prefix(config):
    """Load OpenAMS integration - actual hardware check happens at handle_ready."""
    # Note: We can't reliably check for OAMS sections during config load because
    # they may be in [include] files that haven't been processed yet.
    # The actual OAMS hardware check happens in handle_ready() after all configs load.

    # Always apply patches during config load for any afc_openams sections
    # The patches will only take effect if OpenAMS hardware is actually present
    _patch_lane_pre_sensor_for_ams()
    _patch_extruder_for_virtual_ams()
    _patch_infinite_runout_handler()
    _patch_lane_unload_for_ams()
    _patch_buffer_multiplier_for_ams()
    _patch_buffer_status_for_missing_stepper()
    _patch_buffer_fault_detection_for_ams()
    return afcAMS(config)
