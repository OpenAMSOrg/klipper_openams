# OpenAMS Manager
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# OPTIMIZATIONS APPLIED:
# 1. Unified Sensor Polling: Sensor data (encoder, FPS pressure, F1S, Hub HES) is read
#    through AMSHardwareService's central polling timer. OAMSManager monitors consume
#    cached values instead of polling OAMS hardware directly, reducing duplicate MCU
#    communication by ~50%. Falls back to direct reads if cache is unavailable.
# 2. Adaptive Polling Intervals: Monitors use 2.0s active / 4.0s idle intervals
#    to reduce CPU usage when printer is idle (15-25% reduction in polling overhead)
# 3. Object Caching: Frequently accessed objects (idle_timeout, gcode, toolhead, AFC)
#    are cached at initialization to avoid repeated lookups
# 4. State Change Tracking: FPSState tracks consecutive idle polls to intelligently
#    switch between active and idle polling intervals
# 5. Pre-checks: Monitor functions skip expensive sensor reads when idle and stable
#
# CONFIGURABLE DETECTION PARAMETERS:
# The following parameters can be tuned in [oams_manager] config section:
# - stuck_spool_load_grace: Grace period (seconds) after load before stuck spool detection (default: 8.0)
# - stuck_spool_pressure_threshold: Pressure below which stuck detection starts (default: 0.08)
# - stuck_spool_pressure_clear_threshold: Pressure above which stuck state clears - hysteresis (default: 0.12)
# - clog_pressure_target: Target FPS pressure for clog detection (default: 0.50)
# - post_load_pressure_dwell: Duration (seconds) to monitor pressure after load (default: 15.0)
# - load_fps_stuck_threshold: FPS pressure above which load is considered failed (default: 0.75)
# - clog_sensitivity: Detection sensitivity level - "low", "medium", "high" (default: "medium")
# - extra_retract: Default extra retract distance (mm) applied before unload (default: 10.0)

import logging
import traceback
from contextlib import contextmanager
from dataclasses import dataclass
from functools import partial
from typing import Optional, Tuple, Dict, List, Any, Callable

try:
    from extras.openams_integration import (
        AMSRunoutCoordinator,
        AMSHardwareService,
        normalize_extruder_name as _normalize_extruder_name,
        OPENAMS_VERSION,
    )
except Exception:
    AMSRunoutCoordinator = None
    AMSHardwareService = None
    OPENAMS_VERSION = "0.0.3"  # Fallback if import fails
    # Fallback implementation if openams_integration not available
    def _normalize_extruder_name(name: Optional[str]) -> Optional[str]:
        """Fallback: Return a lowercase token for comparing extruder identifiers."""
        if not name or not isinstance(name, str):
            return None
        cleaned = name.strip()
        return cleaned.lower() if cleaned else None

try:
    from extras.oams import OAMSStatus, OAMSOpCode
except Exception:
    OAMSStatus = None
    OAMSOpCode = None

# Configuration constants
PAUSE_DISTANCE = 60
MIN_ENCODER_DIFF = 3  # Increased from 1 to 3 - prevents false positives during print stalls/brief pauses
FILAMENT_PATH_LENGTH_FACTOR = 1.14
MONITOR_ENCODER_PERIOD = 2.0
MONITOR_ENCODER_PERIOD_IDLE = 4.0  # OPTIMIZATION: Longer interval when idle
MONITOR_ENCODER_SPEED_GRACE = 3.0  # Increased from 2.0 to 3.0 - more grace time before stuck detection
AUTO_DETECT_INSERT_GRACE = 10.0  # Seconds to suppress auto-detect after unload completes
MIN_EXTRUDER_ENGAGEMENT_DELTA = 0.5  # Minimum extruder movement to confirm engagement
ENGAGEMENT_SUPPRESSION_WINDOW = 10.0  # Increased from 6.0 to 10.0 - longer suppression after engagement to prevent false clog detection
CLOG_CHECK_INTERVAL = 8.0  # Minimum seconds between clog checks to reduce log/CPU churn
AFC_DELEGATION_TIMEOUT = 30.0
COASTING_TIMEOUT = 1800.0  # Max time to wait for hub to clear and filament to coast through PTFE (30 minutes - typical prints take 15-20 min)
IDLE_POLL_THRESHOLD = 3  # OPTIMIZATION: Polls before switching to idle interval

STUCK_SPOOL_PRESSURE_THRESHOLD = 0.08
STUCK_SPOOL_PRESSURE_CLEAR_THRESHOLD = 0.12  # Hysteresis upper threshold
STUCK_SPOOL_DWELL = 5.0  # Increased from 3.5 to 5.0 - debouncing for high-speed printing with fast extrudes/retracts (FPS is slow to react)
STUCK_SPOOL_LOAD_GRACE = 8.0
STUCK_SPOOL_MAX_ATTEMPTS = 2  # User requirement: max 2 attempts for stuck spool pre-engagement detection

CLOG_PRESSURE_TARGET = 0.50
CLOG_SENSITIVITY_LEVELS = {
    "low": {"extrusion_window": 48.0, "encoder_slack": 15, "pressure_band": 0.08, "dwell": 12.0},
    "medium": {"extrusion_window": 24.0, "encoder_slack": 8, "pressure_band": 0.06, "dwell": 10.0},  # Increased from 8.0 to 10.0 for debouncing
    "high": {"extrusion_window": 12.0, "encoder_slack": 4, "pressure_band": 0.04, "dwell": 8.0},  # Increased from 6.0 to 8.0 for debouncing
}
CLOG_SENSITIVITY_DEFAULT = "medium"

POST_LOAD_PRESSURE_THRESHOLD = 0.65
POST_LOAD_PRESSURE_DWELL = 15.0
POST_LOAD_PRESSURE_CHECK_PERIOD = 0.5

# Threshold for detecting failed load - if FPS stays above this during LOADING, filament isn't engaging
LOAD_FPS_STUCK_THRESHOLD = 0.75


class OAMSRunoutState:
    """Enum for runout monitor states."""
    STOPPED = "STOPPED"
    MONITORING = "MONITORING"
    DETECTED = "DETECTED"
    COASTING = "COASTING"
    RELOADING = "RELOADING"
    PAUSED = "PAUSED"


class FPSLoadState:
    """Enum for FPS loading states"""
    UNLOADED = 0
    LOADED = 1
    LOADING = 2
    UNLOADING = 3


@dataclass
class FollowerState:
    """Consolidated state tracking for OAMS follower motor per unit.

    Replaces 5 separate dictionaries with a single cohesive state object.
    """
    coasting: bool = False                           # Is follower in coast mode (hub empty, coasting before disable)
    coast_start_pos: float = 0.0                     # Extruder position when coast started (mm)
    had_filament: bool = False                       # Previous state - whether follower had filament
    last_state: Optional[Tuple[int, int]] = None     # (enable, direction) to avoid redundant MCU commands


@dataclass
class StuckSpoolState:
    """Tracks stuck spool detection state for a single FPS.

    Stuck spool detection monitors for filament that won't feed properly,
    typically due to tangles, binding, or mechanical issues.
    """
    active: bool = False                        # Is stuck spool currently detected
    start_time: Optional[float] = None          # When stuck condition was first detected
    restore_follower: bool = False              # Should follower be re-enabled after clearing
    restore_direction: int = 1                  # Direction to restore follower to


@dataclass
class ClogState:
    """Tracks clog detection state for a single FPS.

    Clog detection monitors for blockages in the hotend or nozzle that
    prevent filament from extruding properly.
    """
    active: bool = False                        # Is clog currently detected
    start_extruder: Optional[float] = None      # Extruder position when clog started
    start_encoder: Optional[int] = None         # Encoder clicks when clog started
    start_time: Optional[float] = None          # Timestamp when clog was first detected
    min_pressure: Optional[float] = None        # Minimum FPS pressure observed during clog
    max_pressure: Optional[float] = None        # Maximum FPS pressure observed during clog
    last_extruder: Optional[float] = None       # Last extruder position checked
    last_wait_log_time: Optional[float] = None  # Last time we logged a wait for extrusion window
    last_check_time: Optional[float] = None     # Last time clog detection ran
    retraction_count: int = 0                   # Number of retractions detected in current window
    last_retraction_time: Optional[float] = None  # Timestamp of most recent retraction
    restore_follower: bool = False              # Whether follower was enabled before clog pause
    restore_direction: int = 1                  # Follower direction to restore after resume


class OAMSRunoutMonitor:
    """Monitors filament runout for a specific FPS."""
    
    def __init__(self,
                 printer,
                 fps_name: str,
                 fps,
                 fps_state,
                 oams: Dict[str, Any],
                 reload_callback: Callable,
                 logger: logging.Logger,
                 follower_callback: Optional[Callable[..., None]] = None,
                 reload_before_toolhead_distance: float = 0.0,
                 debounce_delay: float = 0.0):
        self.oams = oams
        self.printer = printer
        self.fps_name = fps_name
        self.fps_state = fps_state
        self.fps = fps
        self.logger = logger
        self._follower_callback = follower_callback

        self.state = OAMSRunoutState.STOPPED
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        self.runout_spool_idx: Optional[int] = None
        self.is_cross_extruder_runout: bool = False  # Track if runout is cross-extruder

        # Track when hub sensor clears during COASTING
        self.hub_cleared: bool = False
        self.hub_clear_position: Optional[float] = None
        self.coasting_start_time: Optional[float] = None

        self.reload_before_toolhead_distance = reload_before_toolhead_distance
        self.reload_callback = reload_callback

        self.reactor = self.printer.get_reactor()

        # F1S sensor debounce tracking (uses AFC's global debounce_delay)
        self.debounce_delay = debounce_delay
        self.f1s_empty_start: Optional[float] = None

        self.hardware_service = None
        self.latest_lane_name: Optional[str] = None
        self._logged_f1s_error: bool = False
        self._logged_lane_resolution_fallback: bool = False
        if AMSRunoutCoordinator is not None:
            try:
                self.hardware_service = AMSRunoutCoordinator.register_runout_monitor(self)
            except Exception as e:
                self.logger.error(
                    "CRITICAL: Failed to register OpenAMS monitor with AFC (AMSRunoutCoordinator). "
                    f"Infinite runout and AFC integration will not function. Error: {e}"
                )
                self.hardware_service = None

        def _get_cached_sensor_values(unit_name: str, oams_obj: Any) -> Dict[str, Any]:
            """Get sensor values from hardware service cache or fall back to direct read.

            Returns dict with f1s_hes_value and hub_hes_value lists.
            """
            result = {"f1s_hes_value": None, "hub_hes_value": None, "from_cache": False}

            # Try hardware service cache first
            if self.hardware_service is not None:
                try:
                    status = self.hardware_service.latest_status()
                    if status:
                        f1s = status.get("f1s_hes_value")
                        hub = status.get("hub_hes_value")
                        if f1s is not None and hub is not None:
                            result["f1s_hes_value"] = f1s
                            result["hub_hes_value"] = hub
                            result["from_cache"] = True
                            return result
                except Exception:
                    pass

            # Fall back to direct OAMS read
            if oams_obj is not None:
                try:
                    result["f1s_hes_value"] = list(getattr(oams_obj, "f1s_hes_value", []) or [])
                    result["hub_hes_value"] = list(getattr(oams_obj, "hub_hes_value", []) or [])
                except Exception:
                    pass

            return result

        self._get_cached_sensor_values = _get_cached_sensor_values

        def _monitor_runout(eventtime):
            try:
                idle_timeout = self.printer.lookup_object("idle_timeout")

                is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
                spool_idx = self.fps_state.current_spool_idx or self.runout_spool_idx

                if not is_printing and self.state == OAMSRunoutState.MONITORING:
                    return eventtime + MONITOR_ENCODER_PERIOD_IDLE

                if self.state in (OAMSRunoutState.STOPPED, OAMSRunoutState.PAUSED, OAMSRunoutState.RELOADING):
                    # When not actively monitoring, use the idle interval to reduce timer churn
                    return eventtime + MONITOR_ENCODER_PERIOD_IDLE

                if self.state == OAMSRunoutState.MONITORING:
                    if getattr(fps_state, "afc_delegation_active", False):
                        now = self.reactor.monotonic()
                        if now < getattr(fps_state, "afc_delegation_until", 0.0):
                            return eventtime + MONITOR_ENCODER_PERIOD
                        fps_state.afc_delegation_active = False
                        fps_state.afc_delegation_until = 0.0
        
                    oams_obj = self._get_oams_object(fps_state.current_oams)
                    if oams_obj is None:
                        return eventtime + MONITOR_ENCODER_PERIOD
        
                    spool_idx = fps_state.current_spool_idx
                    if spool_idx is None:
                        self.latest_lane_name = None
                        return eventtime + MONITOR_ENCODER_PERIOD
        
                    lane_name = None
                    spool_empty = None
                    # Get OAMS name - strip "oams " prefix if present for hardware service lookup
                    # fps_state.current_oams is "oams oams1" but hardware service expects "oams1"
                    oams_full_name = getattr(fps_state, "current_oams", None) or self.fps_name
                    unit_name = oams_full_name
                    if isinstance(unit_name, str) and unit_name.startswith("oams "):
                        unit_name = unit_name[5:]  # Strip "oams " prefix

                    if self.hardware_service is not None:
                        try:
                            lane_name = self.hardware_service.resolve_lane_for_spool(unit_name, spool_idx)
                            # Successfully resolved - clear fallback flag so we log if it fails again
                            if self._logged_lane_resolution_fallback:
                                if lane_name is not None:
                                    self.logger.debug(f"OAMS: Hardware service lane resolution recovered for {self.fps_name}")
                                self._logged_lane_resolution_fallback = False
                        except Exception as e:
                            # Log exception details for debugging (only once)
                            if not self._logged_lane_resolution_fallback:
                                self.logger.debug(
                                    f"OAMS: Hardware service resolve_lane_for_spool failed for {self.fps_name} "
                                    f"(unit_name={unit_name}, spool_idx={spool_idx}): {e}"
                                )
                                self._logged_lane_resolution_fallback = True

                    if lane_name is None and fps_state.current_lane is not None:
                        lane_name = fps_state.current_lane
                        # Only log once to prevent log spam (like F1S error handling)
                        if not self._logged_lane_resolution_fallback:
                            self.logger.debug(
                                f"OAMS: Hardware service failed to resolve lane name for {self.fps_name}, "
                                f"using fps_state.current_lane '{lane_name}' as fallback"
                            )
                            self._logged_lane_resolution_fallback = True


                    # Use cached sensor data from AMSHardwareService when available
                    try:
                        cached = self._get_cached_sensor_values(unit_name, oams_obj)
                        f1s_values = cached.get("f1s_hes_value")
                        if f1s_values is None:
                            if not self._logged_f1s_error:
                                self.logger.error(f"OAMS: Failed to read F1S values for {self.fps_name} - runout detection paused")
                                self._logged_f1s_error = True
                            return eventtime + MONITOR_ENCODER_PERIOD
                        if spool_idx < 0 or spool_idx >= len(f1s_values):
                            return eventtime + MONITOR_ENCODER_PERIOD
                        spool_empty = not bool(f1s_values[spool_idx])
                        if self._logged_f1s_error:
                            self.logger.debug(f"OAMS: F1S values recovered for {self.fps_name}")
                            self._logged_f1s_error = False
                    except Exception:
                        if not self._logged_f1s_error:
                            self.logger.error(f"OAMS: Failed to read F1S values for {self.fps_name} - runout detection paused")
                            self._logged_f1s_error = True
                        return eventtime + MONITOR_ENCODER_PERIOD
        
                    if lane_name is None:
                        lane_name = self.latest_lane_name

                    self.latest_lane_name = lane_name

                    # F1S debounce: only trigger runout if sensor has been empty for the configured duration
                    if spool_empty:
                        current_time = self.reactor.monotonic()
                        if self.f1s_empty_start is None:
                            # First time seeing empty, start debounce timer
                            self.f1s_empty_start = current_time
                            if self.debounce_delay > 0.0:
                                self.logger.debug(
                                    f"OAMS: F1S reports empty on {self.fps_name} spool {spool_idx}, starting {self.debounce_delay:.1f}s debounce timer"
                                )

                        elif current_time - self.f1s_empty_start >= self.debounce_delay:
                            # Debounce duration elapsed, trigger runout
                            self._detect_runout(oams_obj, lane_name, spool_idx, eventtime)
                            # Keep f1s_empty_start set so we don't re-trigger during the same runout
                        # else: still debouncing, wait for next check
                    else:
                        # F1S shows filament present, reset debounce timer
                        if self.f1s_empty_start is not None:
                            if self.debounce_delay > 0.0:
                                self.logger.debug(
                                    f"OAMS: F1S shows filament present again on {self.fps_name} spool {spool_idx}, resetting debounce timer"
                                )

                            self.f1s_empty_start = None
        
                elif self.state in (OAMSRunoutState.DETECTED, OAMSRunoutState.COASTING):
                    # Cross-extruder runouts don't need coasting; trigger the
                    # reload immediately so only same-FPS runouts use the
                    # hub/PTFE coasting path.
                    if self.is_cross_extruder_runout:
                        self.state = OAMSRunoutState.RELOADING
                        self.reload_callback()
                        return eventtime + MONITOR_ENCODER_PERIOD

                    # If we have traveled the configured pause distance since F1S
                    # reported empty, treat the hub as cleared even if the hub
                    # sensor still reads present. This allows same-FPS runouts to
                    # continue into the coasting phase when the hub sensor is slow
                    # or stuck.
                    if (self.state == OAMSRunoutState.DETECTED and self.runout_position is not None and
                            fps.extruder.last_position - self.runout_position >= PAUSE_DISTANCE):
                        self.hub_cleared = False
                        self.hub_clear_position = None
                        self.runout_after_position = None
                        self.coasting_start_time = None
                        self.state = OAMSRunoutState.COASTING
                        self.logger.info(
                            f"OAMS: Pause complete, entering COASTING (waiting for hub to clear before counting) on {self.fps_name}"
                        )

                    oams_obj = self._get_oams_object(fps_state.current_oams)
                    if oams_obj is None:
                        return eventtime + MONITOR_ENCODER_PERIOD

                    # Get unit name for cached lookup
                    coast_unit_name = fps_state.current_oams or self.fps_name
                    if isinstance(coast_unit_name, str) and coast_unit_name.startswith("oams "):
                        coast_unit_name = coast_unit_name[5:]

                    # Use cached sensor data from AMSHardwareService when available
                    try:
                        cached = self._get_cached_sensor_values(coast_unit_name, oams_obj)
                        hub_values = cached.get("hub_hes_value")
                        if hub_values is None or spool_idx >= len(hub_values):
                            self.logger.error(f"OAMS: Failed to read hub HES values during COASTING on {self.fps_name}")
                            return eventtime + MONITOR_ENCODER_PERIOD
                        spool_present = bool(hub_values[spool_idx])
                    except Exception as e:
                        self.logger.error(f"OAMS: Failed to read hub HES values during COASTING on {self.fps_name}: {e}")
                        return eventtime + MONITOR_ENCODER_PERIOD
        
                    if spool_present:
                        if not self.hub_cleared:
                            if self.coasting_start_time is None:
                                self.coasting_start_time = self.reactor.monotonic()
                            else:
                                elapsed = self.reactor.monotonic() - self.coasting_start_time
                                if elapsed >= COASTING_TIMEOUT:
                                    self.logger.info(
                                        f"OAMS: COASTING timeout reached ({elapsed:.1f}s) on {self.fps_name}; proceeding to reload"
                                    )
                                    self.state = OAMSRunoutState.RELOADING
                                    self.reload_callback()
                        # If we previously cleared the hub but sensor reports present again,
                        # reset coasting so we wait for a clean clear signal.
                        self.hub_cleared = False
                        self.hub_clear_position = None
                        self.runout_after_position = None
                    else:
                        if not self.hub_cleared:
                            self.hub_cleared = True
                            self.hub_clear_position = fps.extruder.last_position
                            self.runout_after_position = 0.0
                            self.coasting_start_time = None
                            self.logger.info(
                                f"OAMS: Hub sensor cleared at position {self.hub_clear_position:.1f}, starting shared PTFE countdown"
                            )

                    if self.hub_clear_position is not None:
                        traveled_distance_after_hub_clear = max(fps.extruder.last_position - self.hub_clear_position, 0.0)
                        self.runout_after_position = traveled_distance_after_hub_clear
                    elif self.hub_cleared:
                        # Hub marked as cleared (pause distance reached) but we
                        # still don't have a clear position yet; treat distance
                        # as zero until we record the position on the next pass.
                        self.runout_after_position = 0.0

                try:
                    path_length = getattr(oams_obj, "filament_path_length", 0.0)
                except Exception as e:
                    self.logger.error(f"OAMS: Failed to read filament path length while coasting on {self.fps_name}: {e}")
                    return eventtime + MONITOR_ENCODER_PERIOD

                effective_path_length = (path_length / FILAMENT_PATH_LENGTH_FACTOR if path_length else 0.0)
                # When the hub sensor sticks or the clear position hasn't been
                # set yet, don't let None arithmetic crash the monitor. Treat
                # the traveled distance after the hub as zero until we have a
                # valid clear position so the loop can continue to evaluate
                # coasting progress and eventually trigger the reload.
                runout_after_position = self.runout_after_position or 0.0

                if self.hub_clear_position is None and self.hub_cleared:
                    self.hub_clear_position = fps.extruder.last_position
                    runout_after_position = 0.0

                consumed_with_margin = (runout_after_position + self.reload_before_toolhead_distance)

                if not hasattr(self, '_last_coast_log_position'):
                    self._last_coast_log_position = 0.0
                    self.logger.debug(
                        "OAMS: COASTING - path_length="
                        f"{path_length:.1f}, effective_path_length={effective_path_length:.1f}, "
                        f"reload_margin={self.reload_before_toolhead_distance:.1f}"
                    )

                if self.hub_cleared and runout_after_position - self._last_coast_log_position >= 100.0:
                    self._last_coast_log_position = runout_after_position
                    remaining = effective_path_length - consumed_with_margin
                    self.logger.debug(
                        "OAMS: COASTING progress (after hub clear) - "
                        f"runout_after={runout_after_position:.1f}, "
                        f"consumed_with_margin={consumed_with_margin:.1f}, remaining={remaining:.1f}"
                    )

                if self.hub_cleared and consumed_with_margin >= effective_path_length:
                    self.logger.info(
                        "OAMS: Old filament cleared shared PTFE "
                        f"({runout_after_position:.2f} mm after hub clear, "
                        f"{effective_path_length:.2f} mm effective path), loading new lane"
                    )
                    self._last_coast_log_position = 0.0  # Reset for next runout
                    self.state = OAMSRunoutState.RELOADING
                    self.reload_callback()
        
                return eventtime + MONITOR_ENCODER_PERIOD
            except Exception as e:
                self.logger.error(
                    f"Runout monitor crashed for {self.fps_name}; continuing after backoff: {e}",
                    traceback=traceback.format_exc(),
                )
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

        self._timer_callback = _monitor_runout
        self.timer = None  # Don't register timer until start() is called

    def _detect_runout(self, oams_obj, lane_name, spool_idx, eventtime):
        """Handle runout detection when the F1S sensor reports empty."""
        try:
            idle_timeout = self.printer.lookup_object("idle_timeout")

            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        except Exception:
            is_printing = False

        fps_state = self.fps_state
        fps = self.fps

        if not (is_printing and fps_state.state == FPSLoadState.LOADED and
                fps_state.current_lane is not None and fps_state.current_spool_idx is not None):
            return

        try:
            afc = self.printer.lookup_object('AFC')
            current_lane_obj = None
            target_lane_obj = None
            target_lane_name = None

            if afc and hasattr(afc, 'lanes'):
                current_lane_obj = afc.lanes.get(lane_name)
                if current_lane_obj:
                    target_lane_name = getattr(current_lane_obj, 'runout_lane', None)

                    # If no runout lane is configured, pause immediately without reload attempt
                    if target_lane_name is None:
                        self.logger.info(f"OAMS: No runout_lane configured for {lane_name} - pausing without reload")
                        self.state = OAMSRunoutState.PAUSED
                        self.runout_position = fps.extruder.last_position
                        fps_state.is_cross_extruder_runout = False

                        gcode = self.printer.lookup_object('gcode')
                        gcode.run_script_from_command("PAUSE")

                        gcode.respond_info(f"Filament runout detected on {lane_name} with no reload lane configured")

                        return

                    target_lane_obj = afc.lanes.get(target_lane_name)
        except Exception as e:
            self.logger.error(f"OAMS: Failed to resolve runout lane mapping for {lane_name}: {e}")
            current_lane_obj = None
            target_lane_obj = None
            target_lane_name = None
        if target_lane_name is None and lane_name:
            try:
                target_lane_name = self._get_runout_lane_from_snapshot(lane_name)
                if target_lane_name:
                    afc = self.printer.lookup_object('AFC', None)
                    if afc and hasattr(afc, 'lanes'):
                        target_lane_obj = afc.lanes.get(target_lane_name)
            except Exception:
                self.logger.debug(f"OAMS: Failed to read AFC.var.unit runout_lane for {lane_name}")

        try:
            current_extruder = self._get_lane_extruder_name(current_lane_obj)
            target_extruder = self._get_lane_extruder_name(target_lane_obj)

            if current_lane_obj and target_lane_obj and current_extruder and target_extruder and current_extruder != target_extruder:
                self.is_cross_extruder_runout = True
                self.logger.info(
                    f"OAMS: Detected cross-extruder runout: {lane_name} (extruder {current_extruder}) -> {target_lane_name} (extruder {target_extruder})"
                )

            else:
                self.is_cross_extruder_runout = False
                if current_lane_obj and target_lane_obj and current_extruder == target_extruder:
                    self.logger.info(
                        f"OAMS: Detected same-extruder runout: {lane_name} -> {target_lane_name} (both on extruder {current_extruder})"
                    )


                    # Set flag on current lane to allow lane_loaded clearing during sensor callback
                    # This flag tells AFC_OpenAMS that sensor going False is due to runout, not tool change
                    if current_lane_obj:
                        current_lane_obj._oams_same_fps_runout = True
                elif current_lane_obj or target_lane_obj:
                    self.logger.warning(
                        f"OAMS: Defaulting to same-extruder runout (missing extruder info): {lane_name} -> {target_lane_name or 'unknown'}"
                    )


                    # Set flag anyway for default case
                    if current_lane_obj:
                        current_lane_obj._oams_same_fps_runout = True
        except Exception as e:
            self.logger.error(f"OAMS: Failed to determine cross-extruder runout status, defaulting to same-FPS: {e}")
            self.is_cross_extruder_runout = False

        self.state = OAMSRunoutState.DETECTED
        self.runout_position = fps.extruder.last_position
        self.runout_spool_idx = spool_idx
        fps_state.is_cross_extruder_runout = self.is_cross_extruder_runout

        if self.is_cross_extruder_runout and lane_name:
            try:
                afc = self.printer.lookup_object('AFC')
                if afc and hasattr(afc, 'lanes'):
                        lane_obj = afc.lanes.get(lane_name)
                        if lane_obj:
                            lane_obj._oams_cross_extruder_runout = True
                        self.logger.info(
                            f"OAMS: Set cross-extruder runout flag on lane {lane_name} to bypass shared load/prep validation"
                        )

            except Exception as e:
                self.logger.error(f"OAMS: Failed to set cross-extruder runout flag on lane {lane_name}: {e}")

        if self.is_cross_extruder_runout:
            self.logger.info(
                f"OAMS: Cross-extruder runout detected on FPS {self.fps_name} (F1S empty, target on different extruder) - will trigger immediate tool change"
            )

        else:
            self.logger.info(
                f"OAMS: Same-extruder runout detected on FPS {self.fps_name} (F1S empty), pausing for {PAUSE_DISTANCE} mm"
            )
            if self._follower_callback is not None:
                try:
                    self._follower_callback(
                        self.fps_name,
                        fps_state,
                        oams_obj,
                        1,
                        1,
                        "same-FPS runout coast",
                        True,
                    )
                except Exception:
                    self.logger.error(
                        f"Failed to keep follower enabled for {self.fps_name} during same-FPS runout"
                    )


        if AMSRunoutCoordinator is not None:
            try:
                AMSRunoutCoordinator.notify_runout_detected(self, spool_idx, lane_name=lane_name)
            except Exception:
                self.logger.error("Failed to notify AFC about OpenAMS runout")

    def _get_runout_lane_from_snapshot(self, lane_name: str) -> Optional[str]:
        unit_name = None
        afc = self.printer.lookup_object("AFC", None)
        if afc is not None:
            lane_obj = getattr(afc, "lanes", {}).get(lane_name)
            if lane_obj is not None:
                unit_name = getattr(lane_obj, "unit", None)

        lane_data = self._get_lane_snapshot(lane_name, unit_name=unit_name)
        if isinstance(lane_data, dict):
            return lane_data.get("runout_lane")
        return None

    def _resolve_oams_name(
        self,
        oams_name: Optional[str],
        oams_obj: Optional[Any] = None,
    ) -> Tuple[Optional[str], Optional[Any]]:
        if not oams_name:
            return None, None

        if oams_name in self.oams:
            return oams_name, self.oams.get(oams_name)

        if oams_obj is not None:
            obj_name = getattr(oams_obj, "name", None)
            if obj_name in self.oams:
                return obj_name, self.oams.get(obj_name)

        prefixed = f"oams {oams_name}"
        if prefixed in self.oams:
            return prefixed, self.oams.get(prefixed)

        if oams_name.startswith("oams "):
            unprefixed = oams_name[5:]
            if unprefixed in self.oams:
                return unprefixed, self.oams.get(unprefixed)

        return oams_name, None

    def _has_gcode_command(self, gcode, command: str) -> bool:
        handlers = getattr(gcode, "ready_gcode_handlers", None)
        if isinstance(handlers, dict):
            return command in handlers
        handlers = getattr(gcode, "gcode_handlers", None)
        if isinstance(handlers, dict):
            return command in handlers
        handlers = getattr(gcode, "_gcode_handlers", None)
        if isinstance(handlers, dict):
            return command in handlers
        get_command = getattr(gcode, "get_command", None)
        if callable(get_command):
            try:
                return get_command(command) is not None
            except Exception:
                return False
        lookup_command = getattr(gcode, "lookup_command", None)
        if callable(lookup_command):
            try:
                lookup_command(command)
                return True
            except Exception:
                return False
        return False

    def _run_tool_crash_detection(self, enable: bool) -> bool:
        if getattr(self, "crash_detection_mode", "disabled") == "disabled":
            self.logger.debug("Tool crash detection disabled; skipping")
            return True
        gcode = self._gcode_obj
        if gcode is None:
            try:
                gcode = self.printer.lookup_object("gcode")
            except Exception as exc:
                self.logger.debug(f"Skipping tool crash detection; no gcode object: {exc}")
                return False
            self._gcode_obj = gcode
        if enable:
            if self.crash_detection_mode == "probe":
                commands = ("START_TOOL_PROBE_CRASH_DETECTION",)
            else:
                commands = ("START_TOOL_CRASH_DETECTION",)
        else:
            if self.crash_detection_mode == "probe":
                commands = ("STOP_TOOL_PROBE_CRASH_DETECTION",)
            else:
                commands = ("STOP_TOOL_CRASH_DETECTION",)
        last_exc = None
        for command in commands:
            for candidate in (command, command.lower()):
                try:
                    self.logger.debug(f"Running tool crash detection command: {candidate}")
                    gcode.run_script_from_command(candidate)
                    self.logger.debug(f"Tool crash detection command completed: {candidate}")
                    return True
                except Exception as exc:
                    self.logger.debug(
                        f"Tool crash detection command failed: {candidate} ({exc})"
                    )
                    last_exc = exc
                    continue
        if last_exc is not None:
            self.logger.debug(f"Skipping tool crash detection; failed {commands}: {last_exc}")
        else:
            self.logger.debug("Skipping tool crash detection command; none available")
        return False

    def _get_oams_object(self, oams_name: Optional[str]):
        _, oams_obj = self._resolve_oams_name(oams_name)
        return oams_obj

    def _normalize_oams_name(self, oams_name: Optional[str], oams_obj: Optional[Any] = None) -> Optional[str]:
        resolved_name, _ = self._resolve_oams_name(oams_name, oams_obj)
        return resolved_name

    def _get_lane_extruder_name(self, lane) -> Optional[str]:
        if lane is None:
            return None
        extruder_obj = getattr(lane, "extruder_obj", None)
        extruder_name = getattr(lane, "extruder_name", None)
        lane_name = getattr(lane, "name", None)
        name = None
        if extruder_obj is not None:
            name = getattr(extruder_obj, "name", None)
        if not name and extruder_name:
            name = extruder_name
        if not name and lane_name:
            try:
                unit_name = getattr(lane, "unit", None)
                lane_data = self._get_lane_snapshot(lane_name, unit_name=unit_name)
                if isinstance(lane_data, dict):
                    name = lane_data.get("extruder")
            except Exception:
                self.logger.debug(f"OAMS: Failed to read AFC.var.unit extruder for {lane_name}")
        return _normalize_extruder_name(name)

    def _resolve_oams_for_lane(self, lane_name: Optional[str]) -> Optional[Any]:
        if not lane_name:
            return None

        afc = self._get_afc()
        if afc is None or not hasattr(afc, "lanes"):
            return None

        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None

        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None:
            base_unit_name = getattr(lane, "unit", None)
            if isinstance(base_unit_name, str) and ":" in base_unit_name:
                base_unit_name = base_unit_name.split(":", 1)[0]
            if base_unit_name and hasattr(afc, "units"):
                unit_obj = afc.units.get(str(base_unit_name))

        oams_name = getattr(unit_obj, "oams_name", None) if unit_obj is not None else None
        return self._get_oams_object(oams_name)

    def start(self) -> None:
        if self.timer is None:
            self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)
        self.state = OAMSRunoutState.MONITORING
        self.f1s_empty_start = None  # Reset debounce timer on start

    def stop(self) -> None:
        self.state = OAMSRunoutState.STOPPED
        self.f1s_empty_start = None  # Reset debounce timer on stop

    def reloading(self) -> None:
        self.state = OAMSRunoutState.RELOADING
        self.runout_position = None
        self.runout_after_position = None
        self.runout_spool_idx = None
        self.is_cross_extruder_runout = False
        # Clear COASTING state tracking
        self.hub_cleared = False
        self.f1s_empty_start = None  # Reset debounce timer on reload
        self.hub_clear_position = None
        self.coasting_start_time = None

    def paused(self) -> None:
        self.state = OAMSRunoutState.PAUSED
        self.f1s_empty_start = None  # Reset debounce timer on pause

    def reset(self) -> None:
        self.state = OAMSRunoutState.STOPPED
        self.f1s_empty_start = None  # Reset debounce timer on reset
        self.runout_position = None
        self.runout_after_position = None
        self.runout_spool_idx = None
        self.is_cross_extruder_runout = False
        # Clear COASTING state tracking
        self.hub_cleared = False
        self.hub_clear_position = None
        self.coasting_start_time = None
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None

class OAMSState:
    """Global state container for all FPS units."""
    def __init__(self):
        self.fps_state: Dict[str, 'FPSState'] = {}
        
    def add_fps_state(self, fps_name: str) -> None:
        self.fps_state[fps_name] = FPSState()
        

class FPSState:
    """Tracks the state of a single FPS"""

    def __init__(self,
                 state: int = FPSLoadState.UNLOADED,
                 current_lane: Optional[str] = None,
                 current_oams: Optional[str] = None,
                 current_spool_idx: Optional[int] = None):

        # Core FPS state
        self.state = state
        self.current_lane = current_lane
        self.current_oams = current_oams
        self.current_spool_idx = current_spool_idx

        # Runout tracking
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None

        # Timers
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None

        # Encoder tracking
        self.encoder_sample_prev: Optional[int] = None
        self.encoder_sample_current: Optional[int] = None

        # Follower state
        self.following: bool = False
        self.direction: int = 0
        self.since: Optional[float] = None

        # AFC delegation
        self.afc_delegation_active: bool = False
        self.afc_delegation_until: float = 0.0

        # Cross-extruder runout tracking
        self.is_cross_extruder_runout: bool = False
        self.last_lane_change_time: Optional[float] = None

        # Complex sub-system states (now dataclasses for better organization)
        self.stuck_spool = StuckSpoolState()
        self.clog = ClogState()

        # Post-load pressure monitoring
        self.post_load_pressure_timer = None
        self.post_load_pressure_start: Optional[float] = None

        # OPTIMIZATION: Adaptive polling state with exponential backoff
        self.consecutive_idle_polls: int = 0
        self.idle_backoff_level: int = 0  # 0-3 for exponential backoff (1x, 2x, 4x, 8x)
        self.last_state_change: Optional[float] = None

        # Engagement tracking - prevents false stuck spool detection after successful engagement
        self.engaged_with_extruder: bool = False
        self.engagement_checked_at: Optional[float] = None
        self.engagement_extruder_pos: Optional[float] = None
        self.load_pressure_dropped: bool = False
        self.engagement_in_progress: bool = False  # Set during engagement verification to suppress stuck detection
        self.engagement_retry_active: bool = False  # Suppress clog detection during engagement retry flow

    def record_encoder_sample(self, value: int) -> Optional[int]:
        """Record encoder sample and return diff if we have 2 samples."""
        self.encoder_sample_prev = self.encoder_sample_current
        self.encoder_sample_current = value
        
        if self.encoder_sample_prev is not None:
            return abs(self.encoder_sample_current - self.encoder_sample_prev)
        return None
    
    def clear_encoder_samples(self):
        """Reset encoder tracking."""
        self.encoder_sample_prev = None
        self.encoder_sample_current = None

    def reset_runout_positions(self) -> None:
        self.runout_position = None
        self.runout_after_position = None

    def reset_stuck_spool_state(self, preserve_restore: bool = False) -> None:
        """Reset stuck spool detection state."""
        self.stuck_spool.start_time = None
        self.stuck_spool.active = False
        if not preserve_restore:
            self.stuck_spool.restore_follower = False
            self.stuck_spool.restore_direction = 1

    def reset_clog_tracker(self, preserve_restore: bool = False) -> None:
        """Reset clog detection state."""
        self.clog.active = False
        self.clog.start_extruder = None
        self.clog.start_encoder = None
        self.clog.start_time = None
        self.clog.min_pressure = None
        self.clog.max_pressure = None
        self.clog.last_extruder = None
        self.clog.last_wait_log_time = None
        self.clog.last_check_time = None
        self.clog.retraction_count = 0
        self.clog.last_retraction_time = None
        if not preserve_restore:
            self.clog.restore_follower = False
            self.clog.restore_direction = 1

    def reset_engagement_tracking(self) -> None:
        """Reset engagement tracking state for clean retry attempts."""
        self.engaged_with_extruder = False
        self.engagement_checked_at = None
        self.engagement_extruder_pos = None
        self.load_pressure_dropped = False
        self.engagement_in_progress = False

    def prime_clog_tracker(self, extruder_pos: float, encoder_clicks: int, pressure: float, timestamp: float) -> None:
        self.clog.start_extruder = extruder_pos
        self.clog.last_extruder = extruder_pos
        self.clog.start_encoder = encoder_clicks
        self.clog.start_time = timestamp
        self.clog.min_pressure = pressure
        self.clog.max_pressure = pressure
        self.clog.last_wait_log_time = None
        self.clog.last_check_time = timestamp
        self.clog.retraction_count = 0
        self.clog.last_retraction_time = None
        
    def __repr__(self) -> str:
        state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
        return f"FPSState(state={state_names.get(self.state, self.state)}, lane={self.current_lane}, oams={self.current_oams}, spool={self.current_spool_idx})"


class OAMSManager:
    """Main coordinator for OpenAMS system"""

    def _run_tool_crash_detection(self, enable: bool) -> bool:
        if getattr(self, "crash_detection_mode", "disabled") == "disabled":
            self.logger.debug("Tool crash detection disabled; skipping")
            return True
        gcode = self._gcode_obj
        if gcode is None:
            try:
                gcode = self.printer.lookup_object("gcode")
            except Exception as exc:
                self.logger.debug(f"Skipping tool crash detection; no gcode object: {exc}")
                return False
            self._gcode_obj = gcode
        if enable:
            if self.crash_detection_mode == "probe":
                commands = ("START_TOOL_PROBE_CRASH_DETECTION",)
            else:
                commands = ("START_TOOL_CRASH_DETECTION",)
        else:
            if self.crash_detection_mode == "probe":
                commands = ("STOP_TOOL_PROBE_CRASH_DETECTION",)
            else:
                commands = ("STOP_TOOL_CRASH_DETECTION",)
        last_exc = None
        for command in commands:
            for candidate in (command, command.lower()):
                try:
                    self.logger.debug(f"Running tool crash detection command: {candidate}")
                    gcode.run_script_from_command(candidate)
                    self.logger.debug(f"Tool crash detection command completed: {candidate}")
                    return True
                except Exception as exc:
                    self.logger.debug(
                        f"Tool crash detection command failed: {candidate} ({exc})"
                    )
                    last_exc = exc
                    continue
        if last_exc is not None:
            self.logger.debug(f"Skipping tool crash detection; failed {commands}: {last_exc}")
        else:
            self.logger.debug("Skipping tool crash detection command; none available")
        return False
    
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        # Use load_object to ensure AFC is loaded even if not yet initialized
        self.afc = self.printer.load_object(config, "AFC")
        self.logger = self.afc.logger

        self.oams: Dict[str, Any] = {}
        self.fpss: Dict[str, Any] = {}

        self.current_state = OAMSState()
        self._afc_logged = False

        self.monitor_timers: List[Any] = []
        self.runout_monitors: Dict[str, OAMSRunoutMonitor] = {}
        self.ready: bool = False

        # Follower coast tracking: when all hub sensors go empty, coast 50mm before disabling
        self.follower_coast_distance = 50.0  # mm to coast after hub empties
        self.follower_state: Dict[str, FollowerState] = {}  # oams_name -> FollowerState (consolidated tracking)

        # LED state tracking: avoid sending redundant LED commands to MCU
        # Key format: "oams_name:spool_idx" -> error_state (0 or 1)
        self.led_error_state: Dict[str, int] = {}  # Track last commanded LED state to avoid redundant MCU commands

        # MCU command completion tracking: prevent command queue overflow
        # Track pending commands per OAMS and wait for completion before sending next
        # Key format: "oams_name" -> list of (command_fn, args, kwargs) tuples
        self._mcu_command_queue: Dict[str, List[Tuple[Callable, tuple, dict]]] = {}
        self._mcu_command_in_flight: Dict[str, bool] = {}  # Track if command is being processed
        self._mcu_command_poll_timers: Dict[str, Any] = {}  # Completion polling timers

        self.reload_before_toolhead_distance: float = config.getfloat("reload_before_toolhead_distance", 0.0)

        # F1S sensor debounce: Use AFC's global debounce_delay to prevent false runouts
        # from momentary sensor flutter. This requires the F1S to read empty for
        # this duration before triggering a runout.
        try:
            afc_obj = self.printer.lookup_object('AFC')
            default_debounce = getattr(afc_obj, 'debounce_delay', 0.0)
        except Exception:
            default_debounce = 0.0
        self.debounce_delay = config.getfloat("debounce_delay", default_debounce, minval=0.0, maxval=5.0)

        sensitivity = config.get("clog_sensitivity", CLOG_SENSITIVITY_DEFAULT).lower()
        if sensitivity not in CLOG_SENSITIVITY_LEVELS:
            self.logger.warning(f"Unknown clog_sensitivity '{sensitivity}', using {CLOG_SENSITIVITY_DEFAULT}")

            sensitivity = CLOG_SENSITIVITY_DEFAULT
        self.clog_sensitivity = sensitivity
        self.clog_settings = CLOG_SENSITIVITY_LEVELS[self.clog_sensitivity]

        # Enable/disable flags for detection systems
        self.enable_clog_detection = config.getboolean("enable_clog_detection", True)
        self.enable_stuck_spool_detection = config.getboolean("enable_stuck_spool_detection", True)

        if not self.enable_clog_detection:
            self.logger.info("Clog detection is DISABLED by config")
        if not self.enable_stuck_spool_detection:
            self.logger.info("Stuck spool detection is DISABLED by config")

        crash_detection_raw = config.get("crash_detection", "0")
        crash_detection_mode = str(crash_detection_raw).strip().lower()
        if crash_detection_mode in {"0", "off", "false", "disabled", "disable"}:
            crash_detection_mode = "disabled"
        elif crash_detection_mode in {"tool", "probe"}:
            pass
        else:
            self.logger.warning(
                f"Unknown crash_detection '{crash_detection_raw}', defaulting to disabled."
            )
            crash_detection_mode = "disabled"
        self.crash_detection_mode = crash_detection_mode

        # Configurable detection thresholds and timing parameters with validation
        self.stuck_spool_load_grace = config.getfloat("stuck_spool_load_grace", STUCK_SPOOL_LOAD_GRACE, minval=0.0, maxval=60.0)
        self.stuck_spool_max_attempts = config.getint("stuck_spool_max_attempts", STUCK_SPOOL_MAX_ATTEMPTS, minval=1, maxval=5)
        self.stuck_spool_pressure_threshold = config.getfloat("stuck_spool_pressure_threshold", STUCK_SPOOL_PRESSURE_THRESHOLD, minval=0.0, maxval=1.0)
        self.stuck_spool_pressure_clear_threshold = config.getfloat("stuck_spool_pressure_clear_threshold", STUCK_SPOOL_PRESSURE_CLEAR_THRESHOLD, minval=0.0, maxval=1.0)
        self.clog_pressure_target = config.getfloat("clog_pressure_target", CLOG_PRESSURE_TARGET, minval=0.0, maxval=1.0)
        self.post_load_pressure_dwell = config.getfloat("post_load_pressure_dwell", POST_LOAD_PRESSURE_DWELL, minval=0.0, maxval=60.0)
        self.load_fps_stuck_threshold = config.getfloat("load_fps_stuck_threshold", LOAD_FPS_STUCK_THRESHOLD, minval=0.0, maxval=1.0)
        self.engagement_pressure_threshold = config.getfloat("engagement_pressure_threshold", 0.6, minval=0.0, maxval=1.0)
        self.extra_retract_default = config.getfloat("extra_retract", 10.0)

        # Validate hysteresis: clear threshold must be > trigger threshold
        if self.stuck_spool_pressure_clear_threshold <= self.stuck_spool_pressure_threshold:
            raise config.error(
                f"stuck_spool_pressure_clear_threshold ({self.stuck_spool_pressure_clear_threshold}) "
                f"must be greater than stuck_spool_pressure_threshold ({self.stuck_spool_pressure_threshold})"
            )

        self._lane_unit_map: Dict[str, str] = {}
        self._lane_by_location: Dict[Tuple[str, int], str] = {}
        self._lane_to_fps_cache: Dict[str, str] = {}  # OPTIMIZATION: Lane?FPS direct mapping cache

        # OPTIMIZATION: Cache hardware service lookups
        self._hardware_service_cache: Dict[str, Any] = {}
        self._idle_timeout_obj = None
        self._gcode_obj = None
        self._toolhead_obj = None
        self._pause_resume_obj = None
        # Prevent duplicate detection logs when the same lane remains loaded
        self._last_logged_detected_lane: Dict[str, Optional[str]] = {}

        self._initialize_oams()

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("idle_timeout:printing", self._handle_printing_resumed)
        self.printer.register_event_handler("pause:resume", self._handle_printing_resumed)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

        self.printer.add_object("oams_manager", self)
        self.register_commands()

    def get_status(self, eventtime: float) -> Dict[str, Dict[str, Any]]:
        """Return current status of all FPS units and OAMS hardware."""
        attributes: Dict[str, Dict[str, Any]] = {"oams": {}}

        for name, oam in self.oams.items():
            status_name = name.split()[-1]
            try:
                oam_status = {
                    "action_status": oam.action_status,
                    "action_status_code": oam.action_status_code,
                    "action_status_value": oam.action_status_value,
                }
            except Exception:
                self.logger.error(f"Failed to fetch status from {name}")
                oam_status = {"action_status": "error", "action_status_code": None, "action_status_value": None}
            attributes["oams"][status_name] = oam_status
            if status_name != name:
                attributes["oams"][name] = oam_status

        state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
        for fps_name, fps_state in self.current_state.fps_state.items():
            attributes[fps_name] = {
                "current_lane": fps_state.current_lane,
                "current_oams": fps_state.current_oams,
                "current_spool_idx": fps_state.current_spool_idx,
                "state_name": state_names.get(fps_state.state, str(fps_state.state)),
                "since": fps_state.since,
            }

        return attributes

    def _get_hardware_service(self, oams_name: str) -> Optional[Any]:
        """Get or create AMSHardwareService for an OAMS unit.

        Uses cached services to avoid repeated lookups. The service provides
        cached sensor data that is polled centrally, reducing duplicate MCU reads.
        Ensures the service is polling and has an attached controller.
        """
        if AMSHardwareService is None:
            return None

        # Normalize oams_name - strip "oams " prefix if present
        normalized_name = oams_name
        if isinstance(oams_name, str) and oams_name.startswith("oams "):
            normalized_name = oams_name[5:]

        # Check cache first
        if normalized_name in self._hardware_service_cache:
            return self._hardware_service_cache[normalized_name]

        try:
            service = AMSHardwareService.for_printer(self.printer, normalized_name, self.logger)

            # Ensure controller is attached if we have the OAMS object
            oams_obj = self.oams.get(f"oams {normalized_name}") or self.oams.get(normalized_name)
            if oams_obj is not None and service.resolve_controller() is None:
                service.attach_controller(oams_obj)

            # Ensure polling is started - this is idempotent (won't start twice)
            if hasattr(service, 'start_polling') and not getattr(service, '_polling_enabled', False):
                try:
                    service.start_polling()
                    self.logger.debug(f"Started hardware service polling for {normalized_name}")
                except Exception as poll_err:
                    self.logger.debug(f"Failed to start polling for {normalized_name}: {poll_err}")

            self._hardware_service_cache[normalized_name] = service
            return service
        except Exception as e:
            self.logger.debug(f"Failed to get hardware service for {normalized_name}: {e}")
            return None

    def _get_cached_sensor_data(self, oams_name: str, oams_obj: Any) -> Dict[str, Any]:
        """Get sensor data from AMSHardwareService cache or fall back to direct read.

        This consolidates sensor reads through AMSHardwareService's polling,
        reducing duplicate MCU communication. Falls back to direct OAMS reads
        if service is unavailable or cache has missing values.

        Returns:
            Dict with keys: encoder_clicks, fps_value, f1s_hes_value, hub_hes_value
        """
        result = {
            "encoder_clicks": None,
            "fps_value": None,
            "f1s_hes_value": None,
            "hub_hes_value": None,
            "from_cache": False,
        }

        # Try to get from hardware service cache first
        service = self._get_hardware_service(oams_name)
        if service is not None:
            try:
                status = service.latest_status()
                if status:
                    result["encoder_clicks"] = status.get("encoder_clicks")
                    result["fps_value"] = status.get("fps_value")
                    result["f1s_hes_value"] = status.get("f1s_hes_value")
                    result["hub_hes_value"] = status.get("hub_hes_value")
                    result["from_cache"] = True

                    # If we got ALL required values from cache, return it
                    # Otherwise fall through to direct read to fill in missing values
                    if (result["encoder_clicks"] is not None and
                        result["hub_hes_value"] is not None and
                        result["f1s_hes_value"] is not None):
                        return result
            except Exception as e:
                self.logger.debug(f"Failed to get cached sensor data for {oams_name}: {e}")

        # Fall back to direct OAMS read if cache miss or service unavailable
        # This fills in any missing values from partial cache hits
        if oams_obj is not None:
            try:
                if result["encoder_clicks"] is None:
                    result["encoder_clicks"] = getattr(oams_obj, "encoder_clicks", None)
                if result["f1s_hes_value"] is None:
                    result["f1s_hes_value"] = list(getattr(oams_obj, "f1s_hes_value", []) or [])
                if result["hub_hes_value"] is None:
                    result["hub_hes_value"] = list(getattr(oams_obj, "hub_hes_value", []) or [])
                result["from_cache"] = False
            except Exception as e:
                self.logger.debug(f"Failed to read sensors directly from {oams_name}: {e}")

        return result

    def _get_cached_fps_value(self, fps_obj: Any, oams_name: str) -> Optional[float]:
        """Get FPS pressure value from cache or direct read.

        The FPS object has the fps_value attribute, but we can also get it
        from the hardware service cache for consistency.
        """
        # Try hardware service cache first
        service = self._get_hardware_service(oams_name)
        if service is not None:
            try:
                status = service.latest_status()
                if status and "fps_value" in status:
                    return float(status["fps_value"])
            except Exception:
                pass

        # Fall back to direct read
        if fps_obj is not None:
            try:
                return float(getattr(fps_obj, "fps_value", 0.0))
            except Exception:
                pass

        return None

    def determine_state(self) -> None:
        """Analyze hardware state and update FPS state tracking."""
        for fps_name, fps_state in self.current_state.fps_state.items():
            # Check if there's an active runout for this FPS
            monitor = self.runout_monitors.get(fps_name)
            is_runout_active = monitor and monitor.state != OAMSRunoutState.MONITORING
            was_loaded = (fps_state.state == FPSLoadState.LOADED)
            is_printing = False
            try:
                idle_timeout = self._idle_timeout_obj
                if idle_timeout is None:
                    idle_timeout = self.printer.lookup_object("idle_timeout")
                    self._idle_timeout_obj = idle_timeout
                if idle_timeout is not None:
                    is_printing = idle_timeout.get_status(self.reactor.monotonic())["state"] == "Printing"
            except Exception:
                is_printing = False

            # Check F1S sensor state if we have OAMS and spool info
            # This helps detect runout conditions before monitor transitions state
            f1s_empty = False
            hub_has_filament = False
            if fps_state.current_oams and fps_state.current_spool_idx is not None:
                oams_obj = self._get_oams_object(fps_state.current_oams)
                if oams_obj:
                    try:
                        f1s_values = getattr(oams_obj, 'f1s_hes_value', None)
                        if f1s_values and fps_state.current_spool_idx < len(f1s_values):
                            f1s_empty = not bool(f1s_values[fps_state.current_spool_idx])

                        hub_values = getattr(oams_obj, 'hub_hes_value', None)
                        if hub_values and fps_state.current_spool_idx < len(hub_values):
                            hub_has_filament = bool(hub_values[fps_state.current_spool_idx])
                    except Exception:
                        pass

            # Query what AFC thinks is loaded (don't assign yet!)
            detected_lane, current_oams, detected_spool_idx = self.determine_current_loaded_lane(fps_name)

            if current_oams is not None and detected_spool_idx is not None:
                same_fps_runout_configured = False
                if is_printing and detected_lane:
                    afc = self._get_afc()
                    if afc is not None and hasattr(afc, "lanes"):
                        source_lane = afc.lanes.get(detected_lane)
                        if source_lane is not None:
                            raw_runout_lane = getattr(source_lane, "runout_lane", None)
                            runout_lane_name = self._resolve_afc_lane_name(afc, raw_runout_lane)
                            target_lane = afc.lanes.get(runout_lane_name) if runout_lane_name else None
                            source_extruder = self._get_lane_extruder_name(source_lane)
                            target_extruder = self._get_lane_extruder_name(target_lane)
                            same_fps_runout_configured = bool(
                                source_extruder and target_extruder and source_extruder == target_extruder
                            )
                hardware_empty = False
                try:
                    hub_values = getattr(current_oams, 'hub_hes_value', None)
                    f1s_values = getattr(current_oams, 'f1s_hes_value', None)
                    if (hub_values is not None and f1s_values is not None and
                            detected_spool_idx < len(hub_values) and detected_spool_idx < len(f1s_values)):
                        hub_has_filament = bool(hub_values[detected_spool_idx])
                        f1s_present = bool(f1s_values[detected_spool_idx])
                        hardware_empty = (not hub_has_filament and not f1s_present)
                except Exception:
                    hardware_empty = False

                if (hardware_empty and not same_fps_runout_configured and not is_runout_active and
                        fps_state.state not in (
                            FPSLoadState.LOADING, FPSLoadState.UNLOADING
                        )):
                    self.logger.info(
                        "State detection: Hardware reports hub and F1S empty for %s (bay %s) on %s; clearing AFC loaded lane %s",
                        getattr(current_oams, "name", "<unknown>"),
                        detected_spool_idx,
                        fps_name,
                        detected_lane,
                    )
                    self._clear_afc_loaded_lane(detected_lane)
                    detected_lane = None
                    current_oams = None
                    detected_spool_idx = None

            if current_oams is not None and detected_spool_idx is not None:
                # Lane is detected as loaded - update state normally
                fps_state.current_lane = detected_lane
                fps_state.current_oams = current_oams.name
                fps_state.current_spool_idx = detected_spool_idx
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._ensure_forward_follower(fps_name, fps_state, "state detection")


                # Sync AFC's lane_loaded with detected state to prevent stale vars file issues
                self._sync_afc_lane_loaded(fps_name, detected_lane)
            else:
                # AFC says no lane loaded (lane_loaded = None)
                if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
                    self.logger.debug(
                        "State detection: Ignoring empty lane report while %s is %s",
                        fps_name,
                        fps_state.state,
                    )
                elif was_loaded and f1s_empty and hub_has_filament:
                    self.logger.info(
                        "State detection: Keeping %s as LOADED (F1S empty, hub has filament, runout about to be detected)",
                        fps_name,
                    )
                    # Keep all existing state - runout detection needs this info
                    pass
                elif is_runout_active and was_loaded:
                    self.logger.info(
                        "State detection: Keeping %s as LOADED during active runout (state=%s, AFC lane_loaded cleared)",
                        fps_name,
                        monitor.state if monitor else "unknown",
                    )
                    # CRITICAL: Don't overwrite current_lane and current_spool_idx!
                    # Keep the existing values so runout detection can complete
                    # Only update current_oams if it was detected (might be None during runout)
                    if current_oams is not None:
                        fps_state.current_oams = current_oams.name
                elif was_loaded:
                    # AFC occasionally clears lane_loaded when tools are parked. Preserve last
                    # known lane if sensors still see filament so we unload before loading anew.
                    if hub_has_filament or not f1s_empty:
                        if current_oams is not None:
                            fps_state.current_oams = current_oams.name
                        self.logger.debug(
                            "State detection: Retaining last known lane %s on %s while AFC lane_loaded is empty (sensors show filament)",
                            fps_state.current_lane,
                            fps_name,
                        )
                    else:
                        # Sensors agree the hub is empty and AFC reports nothing loaded; clear state.
                        old_lane_name = fps_state.current_lane
                        old_oams_name = fps_state.current_oams
                        old_spool_idx = fps_state.current_spool_idx

                        fps_state.current_lane = None
                        fps_state.current_oams = None
                        fps_state.current_spool_idx = None
                        fps_state.state = FPSLoadState.UNLOADED
                        fps_state.reset_stuck_spool_state()
                        fps_state.reset_clog_tracker()
                        self._cancel_post_load_pressure_check(fps_state)

                        if old_lane_name and AMSRunoutCoordinator is not None:
                            try:
                                AMSRunoutCoordinator.notify_lane_tool_state(
                                    self.printer,
                                    old_oams_name,
                                    old_lane_name,
                                    loaded=False,
                                    spool_index=old_spool_idx,
                                    eventtime=self.reactor.monotonic()
                                )
                                self.logger.debug(f"Notified AFC that {old_lane_name} unloaded during state detection (clears virtual sensor)")
                            except Exception:
                                self.logger.error(f"Failed to notify AFC about {old_lane_name} unload during state detection")
                else:
                    # No active runout and not about to detect one - transition to UNLOADED normally
                    # Save lane info before clearing for AFC notification
                    old_lane_name = fps_state.current_lane
                    old_oams_name = fps_state.current_oams
                    old_spool_idx = fps_state.current_spool_idx

                    fps_state.current_lane = None
                    fps_state.current_oams = None
                    fps_state.current_spool_idx = None
                    fps_state.state = FPSLoadState.UNLOADED
                    fps_state.reset_stuck_spool_state()
                    fps_state.reset_clog_tracker()
                    self._cancel_post_load_pressure_check(fps_state)

                    # Notify AFC that lane is unloaded to update virtual sensors (AMS_Extruder#)
                    # This ensures virtual sensors show correct state when lane becomes empty
                    if was_loaded and old_lane_name and AMSRunoutCoordinator is not None:
                        try:
                            AMSRunoutCoordinator.notify_lane_tool_state(
                                self.printer,
                                old_oams_name,
                                old_lane_name,
                                loaded=False,
                                spool_index=old_spool_idx,
                                eventtime=self.reactor.monotonic()
                            )
                            self.logger.debug(f"Notified AFC that {old_lane_name} unloaded during state detection (clears virtual sensor)")
                        except Exception:
                            self.logger.error(f"Failed to notify AFC about {old_lane_name} unload during state detection")

    def _clear_afc_loaded_lane(self, lane_name: Optional[str]) -> None:
        if not lane_name:
            return
        try:
            afc = self._get_afc()
            if afc is None or not hasattr(afc, 'lanes'):
                return
            afc_function = getattr(afc, "function", None)
            if afc_function is not None and hasattr(afc_function, "get_current_lane_obj"):
                try:
                    current_lane = afc_function.get_current_lane_obj()
                except Exception:
                    current_lane = None
                if current_lane is not None and getattr(current_lane, "name", None) == lane_name:
                    unset_lane_loaded = getattr(afc_function, "unset_lane_loaded", None)
                    if callable(unset_lane_loaded):
                        unset_lane_loaded()
                        return
            lane_obj = afc.lanes.get(lane_name)
            if lane_obj is None:
                return
            extruder_obj = getattr(lane_obj, 'extruder_obj', None)
            if extruder_obj is not None:
                extruder_obj.lane_loaded = None
            if getattr(lane_obj, 'tool_loaded', False):
                lane_obj.tool_loaded = False
            if hasattr(afc, 'save_vars') and callable(afc.save_vars):
                try:
                    afc.save_vars()
                except Exception:
                    self.logger.error(f"Failed to save AFC vars after clearing loaded lane {lane_name}")
        except Exception:
            self.logger.error(f"Failed to clear AFC loaded lane for {lane_name}")

    def sync_state_with_afc(self) -> None:
        """Synchronize OAMS manager state with AFC's current state.

        This function validates that OAMS hardware state (current_spool) matches AFC's
        lane tracking (extruder.lane_loaded). It corrects any discrepancies to prevent
        state desync issues like attempting to load a lane when another is already loaded.

        The mapping is: OAMS current_spool 0-3 = AFC lane ams_#:1-4
        """
        try:
            afc = self._get_afc()
            if afc is None:
                self.logger.debug("sync_state_with_afc: AFC not available, skipping")
                return

            # Iterate through all OAMS units and validate their state
            for oams_name, oams_obj in self.oams.items():
                current_spool_idx = getattr(oams_obj, 'current_spool', None)

                # Get the AFC unit object for this OAMS
                afc_unit = None
                if hasattr(afc, 'units'):
                    for unit_name, unit_obj in afc.units.items():
                        # Check if this AFC unit is an OpenAMS unit for this OAMS
                        if hasattr(unit_obj, 'oams_name') and unit_obj.oams_name == oams_name:
                            afc_unit = unit_obj
                            break

                if afc_unit is None:
                    continue

                # Find which lane corresponds to current_spool (0-3 maps to ams_#:1-4)
                if current_spool_idx is not None and hasattr(afc_unit, '_find_lane_by_spool'):
                    lane_obj = afc_unit._find_lane_by_spool(current_spool_idx)
                    if lane_obj:
                        lane_name = getattr(lane_obj, 'name', None)
                        extruder_obj = getattr(lane_obj, 'extruder_obj', None)

                        if lane_name and extruder_obj:
                            # Check if AFC thinks this lane is loaded to the extruder
                            afc_lane_loaded = getattr(extruder_obj, 'lane_loaded', None)

                            # Validate consistency
                            if afc_lane_loaded != lane_name:
                                self.logger.warning(
                                    f"State sync: OAMS {oams_name} has spool {current_spool_idx} loaded "
                                    f"({lane_name}), but AFC thinks {afc_lane_loaded} is loaded. "
                                    f"Syncing AFC to match OAMS hardware state."
                                )

                                # Update AFC to match OAMS hardware
                                extruder_obj.lane_loaded = lane_name

                                # Also update lane.tool_loaded if needed
                                if not getattr(lane_obj, 'tool_loaded', False):
                                    lane_obj.tool_loaded = True

                                # Persist to vars file
                                if hasattr(afc, 'save_vars') and callable(afc.save_vars):
                                    try:
                                        afc.save_vars()
                                        self.logger.info(f"Synced AFC state to match OAMS hardware: {lane_name} loaded")
                                    except Exception as e:
                                        self.logger.error(f"Failed to save AFC vars after state sync: {e}")
                else:
                    # current_spool is None - validate that AFC also shows nothing loaded
                    if hasattr(afc_unit, 'lanes'):
                        for lane_obj in afc_unit.lanes.values():
                            if getattr(lane_obj, 'tool_loaded', False):
                                extruder_obj = getattr(lane_obj, 'extruder_obj', None)
                                lane_name = getattr(lane_obj, 'name', None)

                                self.logger.warning(
                                    f"State sync: OAMS {oams_name} has no spool loaded, "
                                    f"but AFC thinks {lane_name} is loaded. Clearing AFC state."
                                )

                                # Clear the lane from AFC
                                if extruder_obj:
                                    extruder_obj.lane_loaded = None
                                lane_obj.tool_loaded = False

                                # Persist to vars file
                                if hasattr(afc, 'save_vars') and callable(afc.save_vars):
                                    try:
                                        afc.save_vars()
                                        self.logger.info(f"Cleared AFC state to match OAMS hardware: {lane_name} unloaded")
                                    except Exception as e:
                                        self.logger.error(f"Failed to save AFC vars after state clear: {e}")

            # After syncing with hardware state, update FPS state tracking
            self.determine_state()

        except Exception as e:
            self.logger.error(
                f"Failed to sync state with AFC: {e}",
                traceback=traceback.format_exc(),
            )

    def _handle_disconnect(self) -> None:
        """Cleanup timers and resources on disconnect/shutdown."""
        self.logger.info("OAMS Manager shutting down, cleaning up timers...")

        # Clean up MCU command poll timers
        for oams_name, timer in list(self._mcu_command_poll_timers.items()):
            try:
                self.reactor.unregister_timer(timer)
                self.logger.debug(f"Unregistered MCU poll timer for {oams_name}")
            except Exception as e:
                self.logger.warning(f"Failed to unregister MCU poll timer for {oams_name}: {e}")
        self._mcu_command_poll_timers.clear()

        # Clean up monitor timers
        for timer in self.monitor_timers:
            try:
                self.reactor.unregister_timer(timer)
            except Exception as e:
                self.logger.warning(f"Failed to unregister monitor timer: {e}")
        self.monitor_timers.clear()

        self.logger.info("OAMS Manager cleanup complete")

    def handle_ready(self) -> None:
        """Initialize system when printer is ready."""
        for fps_name, fps in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)

        if not self.fpss:
            raise ValueError("No FPS found in system, this is required for OAMS to work")


        # OPTIMIZATION: Cache frequently accessed objects
        try:
            self._idle_timeout_obj = self.printer.lookup_object("idle_timeout")

        except Exception:
            self._idle_timeout_obj = None

        try:
            self._gcode_obj = self.printer.lookup_object("gcode")

        except Exception:
            self._gcode_obj = None

        try:
            self._toolhead_obj = self.printer.lookup_object("toolhead")

        except Exception:
            self._toolhead_obj = None

        self.determine_state()

        try:
            self._sync_virtual_tool_sensors()
        except Exception:
            self.logger.error("Failed to sync virtual tool sensors during startup")

        try:
            self._ensure_followers_for_loaded_hubs()
        except Exception:
            self.logger.error("Failed to enable followers for loaded hubs during startup")

        self.start_monitors()
        self.ready = True

    def _initialize_oams(self) -> None:
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam

    def _resolve_oams_name(
        self,
        oams_name: Optional[str],
        oams_obj: Optional[Any] = None,
    ) -> Tuple[Optional[str], Optional[Any]]:
        if not oams_name:
            return None, None

        if oams_name in self.oams:
            return oams_name, self.oams.get(oams_name)

        if oams_obj is not None:
            obj_name = getattr(oams_obj, "name", None)
            if obj_name in self.oams:
                return obj_name, self.oams.get(obj_name)

        prefixed = f"oams {oams_name}"
        if prefixed in self.oams:
            return prefixed, self.oams.get(prefixed)

        if oams_name.startswith("oams "):
            unprefixed = oams_name[5:]
            if unprefixed in self.oams:
                return unprefixed, self.oams.get(unprefixed)

        return oams_name, None


    def _get_follower_state(self, oams_name: str) -> FollowerState:
        """Get or create FollowerState for an OAMS unit."""
        resolved_name, _ = self._resolve_oams_name(oams_name)
        if resolved_name is None:
            resolved_name = oams_name

        if resolved_name != oams_name and oams_name in self.follower_state:
            if resolved_name not in self.follower_state:
                self.follower_state[resolved_name] = self.follower_state.pop(oams_name)

        if resolved_name not in self.follower_state:
            self.follower_state[resolved_name] = FollowerState()
        return self.follower_state[resolved_name]

    def _sync_afc_lane_loaded(self, fps_name: str, detected_lane: Optional[str]) -> None:
        """Sync AFC's extruder.lane_loaded with OAMS-detected state.

        When OAMS detects a lane is loaded via sensors, update AFC's tracking
        and persist to vars file. This prevents stale vars from causing issues
        on bootup where AFC thinks lane8 is loaded but sensors show lane7.

        Args:
            fps_name: FPS that detected the lane (e.g., "fps fps1")

            detected_lane: Lane name that was detected (e.g., "lane7")

        """
        if not detected_lane:
            return

        try:
            afc = self._get_afc()
            if afc is None:
                return

            # Find which extruder this lane belongs to
            if not hasattr(afc, 'lanes'):
                return

            lane_obj = afc.lanes.get(detected_lane)
            if lane_obj is None:
                return

            extruder_obj = getattr(lane_obj, 'extruder_obj', None)
            if extruder_obj is None:
                return

            extruder_name = getattr(extruder_obj, 'name', None)
            if not extruder_name:
                return

            # Check if AFC's lane_loaded matches what we detected
            current_lane_loaded = getattr(extruder_obj, 'lane_loaded', None)
            if current_lane_loaded == detected_lane:
                return  # Already in sync

            # Update AFC's lane_loaded
            extruder_obj.lane_loaded = detected_lane

            # Persist to vars file
            if hasattr(afc, 'save_vars') and callable(afc.save_vars):
                try:
                    afc.save_vars()
                    self.logger.info(
                        f"Synced AFC: {extruder_name}.lane_loaded = {detected_lane} "
                        f"(was {current_lane_loaded}, detected via sensors)"
                    )
                except Exception:
                    self.logger.error(
                        f"Failed to save AFC vars after syncing {extruder_name}.lane_loaded to {detected_lane}"
                    )
            else:
                self.logger.debug(
                    f"Updated AFC: {extruder_name}.lane_loaded = {detected_lane} "
                    "(vars not saved - AFC has no save_vars method)"
                )
        except Exception:
            self.logger.error(
                f"Failed to sync AFC lane_loaded for {detected_lane} detected on {fps_name}",
                traceback=traceback.format_exc(),
            )

    def _sync_extruder_lane_loaded(self, fps_name: str, extruder_obj, extruder_name: str, afc) -> None:
        """Sync a specific extruder's lane_loaded with OAMS hardware sensors.

        Args:
            fps_name: FPS name (e.g., "fps fps1")
            extruder_obj: AFC extruder object
            extruder_name: Extruder name (e.g., "extruder5")
            afc: AFC object
        """
        # Check what AFC currently thinks is loaded
        current_lane_loaded = getattr(extruder_obj, 'lane_loaded', None)

        # Check the actual HARDWARE sensor state - read OAMS hub sensors directly
        sensor_detected_lanes = []
        for lane_name, lane in afc.lanes.items():
            try:
                # Only check lanes on this FPS
                lane_fps = self.get_fps_for_afc_lane(lane_name)
                if lane_fps != fps_name:
                    continue

                # Only check lanes for this extruder
                lane_extruder = getattr(lane, 'extruder_obj', None)
                if lane_extruder != extruder_obj:
                    continue

                # Read ACTUAL hardware sensor
                unit_obj = getattr(lane, 'unit_obj', None)
                if unit_obj and getattr(unit_obj, 'type', None) == 'OpenAMS':
                    # Get OAMS hardware object
                    oams_name = getattr(unit_obj, 'oams_name', None)
                    if oams_name:
                        oams_obj = self.oams.get(f"oams {oams_name}")
                        if not oams_obj:
                            oams_obj = self.oams.get(oams_name)

                        if oams_obj:
                            # Get the bay/spool index for this lane
                            lane_index = getattr(lane, 'index', None)
                            if lane_index is not None:
                                try:
                                    spool_idx = int(lane_index) - 1
                                    if spool_idx >= 0:
                                        # Read hub sensor from hardware
                                        hub_values = getattr(oams_obj, 'hub_hes_value', None)
                                        if hub_values and spool_idx < len(hub_values):
                                            hub_has_filament = bool(hub_values[spool_idx])
                                            if hub_has_filament:
                                                sensor_detected_lanes.append(lane_name)
                                except (TypeError, ValueError):
                                    pass
            except Exception:
                continue

        # Determine which lane should be loaded based on sensors
        sensor_lane = None
        if len(sensor_detected_lanes) == 1:
            # Exactly one lane shows hub sensor active - use it as source of truth
            sensor_lane = sensor_detected_lanes[0]
        elif len(sensor_detected_lanes) > 1:
            # Multiple lanes show hub sensor active - conflict
            self.logger.warning(
                f"Multiple lanes show hub sensor active for {extruder_name} on {fps_name}: "
                f"{sensor_detected_lanes}. Cannot sync AFC state."
            )
            return

        lane_to_sync = sensor_lane

        # Determine if we need to sync
        # Case 1: Already in sync (both None or both point to same lane)
        if lane_to_sync == current_lane_loaded:
            return

        # Case 2: All hubs empty but AFC thinks a lane is loaded ? clear the state
        if lane_to_sync is None and current_lane_loaded is not None:
            old_lane_obj = afc.lanes.get(current_lane_loaded)
            if old_lane_obj:
                # Clear tool_loaded flag
                if hasattr(old_lane_obj, 'tool_loaded'):
                    old_lane_obj.tool_loaded = False
                    self.logger.debug(f"Cleared tool_loaded flag on {current_lane_loaded} (all hubs empty)")

                # Clear loaded_to_hub flag
                if hasattr(old_lane_obj, 'loaded_to_hub'):
                    old_lane_obj.loaded_to_hub = False
                    self.logger.debug(f"Cleared loaded_to_hub flag on {current_lane_loaded} (all hubs empty)")

            # Clear extruder's lane_loaded
            extruder_obj.lane_loaded = None

            # Update virtual tool sensor to show empty
            try:
                unit_obj = getattr(old_lane_obj, 'unit_obj', None) if old_lane_obj else None
                if unit_obj and getattr(unit_obj, 'type', None) == 'OpenAMS':
                    if hasattr(unit_obj, '_sync_virtual_tool_sensor'):
                        eventtime = self.reactor.monotonic()
                        unit_obj._sync_virtual_tool_sensor(eventtime, None, force=True)
                        self.logger.debug(f"Synced virtual tool sensor to empty state")
            except Exception as e:
                self.logger.error(f"Failed to sync virtual tool sensor to empty: {e}")

            # Persist to vars file
            try:
                if hasattr(afc, 'save_vars'):
                    afc.save_vars()
            except Exception:
                pass

            self.logger.info(
                f"Synced AFC lane mismatch: {extruder_name} cleared (was {current_lane_loaded}, "
                f"all OAMS hardware hub sensors show empty)"
            )
            return

        # Case 3: Sensor shows a lane loaded (possibly different from current) ? sync to that lane
        if lane_to_sync is None:
            return  # No sync needed

        # Get the lane object to sync
        lane_obj_to_sync = afc.lanes.get(lane_to_sync)
        if not lane_obj_to_sync:
            return

        # STEP 1: Clear the old lane's state if it exists
        if current_lane_loaded:
            old_lane_obj = afc.lanes.get(current_lane_loaded)
            if old_lane_obj:
                # Clear tool_loaded flag - the old lane is not actually in the tool
                if hasattr(old_lane_obj, 'tool_loaded'):
                    old_lane_obj.tool_loaded = False
                    self.logger.debug(f"Cleared tool_loaded flag on old lane {current_lane_loaded}")

                # Check if the old lane's hub sensor also shows empty
                # If hardware shows it's not in the hub, clear loaded_to_hub too
                old_lane_hub_has_filament = False
                try:
                    unit_obj = getattr(old_lane_obj, 'unit_obj', None)
                    if unit_obj and getattr(unit_obj, 'type', None) == 'OpenAMS':
                        oams_name = getattr(unit_obj, 'oams_name', None)
                        if oams_name:
                            oams_obj = self.oams.get(f"oams {oams_name}")
                            if not oams_obj:
                                oams_obj = self.oams.get(oams_name)
                            if oams_obj:
                                lane_index = getattr(old_lane_obj, 'index', None)
                                if lane_index is not None:
                                    spool_idx = int(lane_index) - 1
                                    if spool_idx >= 0:
                                        hub_values = getattr(oams_obj, 'hub_hes_value', None)
                                        if hub_values and spool_idx < len(hub_values):
                                            old_lane_hub_has_filament = bool(hub_values[spool_idx])
                except Exception:
                    pass

                # If hub sensor also shows empty, clear loaded_to_hub
                if not old_lane_hub_has_filament and hasattr(old_lane_obj, 'loaded_to_hub'):
                    old_lane_obj.loaded_to_hub = False
                    self.logger.debug(f"Cleared loaded_to_hub flag on old lane {current_lane_loaded} (hub sensor empty)")

        # STEP 2: Update AFC's lane_loaded to point to the correct lane
        extruder_obj.lane_loaded = lane_to_sync

        # STEP 3: Update new lane state
        if hasattr(lane_obj_to_sync, 'loaded_to_hub'):
            lane_obj_to_sync.loaded_to_hub = True

        # Set tool_loaded flag on the new lane
        # The extruder is pointing to this lane, and hardware shows filament in hub
        # So the lane is effectively loaded to the tool
        if hasattr(lane_obj_to_sync, 'tool_loaded'):
            lane_obj_to_sync.tool_loaded = True
            self.logger.debug(f"Set tool_loaded flag on new lane {lane_to_sync}")

        # STEP 4: Persist to vars file
        try:
            if hasattr(afc, 'save_vars'):
                afc.save_vars()
        except Exception:
            pass

        # STEP 5: Update lane selection if needed
        if hasattr(lane_obj_to_sync, 'unit_obj'):
            unit_obj = lane_obj_to_sync.unit_obj
            if hasattr(unit_obj, 'select_lane'):
                try:
                    unit_obj.select_lane(lane_obj_to_sync, False)
                except Exception as e:
                    self.logger.error(f"Failed to select_lane for {lane_to_sync}: {e}")

        # STEP 6: Update virtual tool sensor for OpenAMS units
        try:
            unit_obj = getattr(lane_obj_to_sync, 'unit_obj', None)
            if unit_obj and getattr(unit_obj, 'type', None) == 'OpenAMS':
                # Sync the virtual tool sensor to reflect the corrected lane state
                if hasattr(unit_obj, '_sync_virtual_tool_sensor'):
                    eventtime = self.reactor.monotonic()
                    unit_obj._sync_virtual_tool_sensor(eventtime, lane_to_sync, force=True)
                    self.logger.debug(f"Synced virtual tool sensor for {lane_to_sync}")
        except Exception as e:
            self.logger.error(f"Failed to sync virtual tool sensor for {lane_to_sync}: {e}")

        self.logger.info(
            f"Synced AFC lane mismatch: {extruder_name} now loaded with {lane_to_sync} "
            f"(was {current_lane_loaded}, OAMS hardware hub sensor detected {lane_to_sync})"
        )

    def _sync_all_fps_lanes_after_prep(self) -> None:
        """Sync all FPS lane states with AFC after PREP completes.

        Called after all OpenAMS units have completed PREP.
        At this point, hardware sensors are initialized and lane states are stable.
        """
        try:
            for fps_name, fps_state in self.current_state.fps_state.items():
                # For each FPS, check all lanes on that FPS and sync based on hardware
                try:
                    afc = self._get_afc()
                    if afc is None or not hasattr(afc, 'lanes'):
                        continue

                    # Find all extruders that have lanes on this FPS
                    extruders_on_fps = set()
                    for lane_name, lane in afc.lanes.items():
                        lane_fps = self.get_fps_for_afc_lane(lane_name)
                        if lane_fps == fps_name:
                            extruder_obj = getattr(lane, 'extruder_obj', None)
                            if extruder_obj:
                                extruders_on_fps.add(extruder_obj)

                    # Sync each extruder on this FPS
                    for extruder_obj in extruders_on_fps:
                        extruder_name = getattr(extruder_obj, 'name', None)
                        if not extruder_name:
                            continue

                        try:
                            self._sync_extruder_lane_loaded(fps_name, extruder_obj, extruder_name, afc)
                        except Exception as e:
                            self.logger.error(f"Failed to sync {extruder_name} on {fps_name}: {e}")

                except Exception as e:
                    self.logger.error(
                        f"Failed to sync lanes on {fps_name} after prep: {e}",
                        traceback=traceback.format_exc()
                    )
        except Exception:
            self.logger.error(
                "Failed to sync all FPS lanes after prep",
                traceback=traceback.format_exc(),
            )

    def _fix_afc_runout_helper_time(self, lane_name: str) -> None:
        """Workaround for AFC bug: Update min_event_systime after load operations.

        AFC's handle_load_runout() function has a docstring stating it should update
        min_event_systime to prevent sensor event queue buildup, but the code is missing.
        This causes Klipper crashes during manual load operations.

        This is a defensive workaround until AFC fixes the bug in AFC_lane.py:676

        Args:
            lane_name: AFC lane name (e.g., "lane5")

        """
        try:
            afc = self._get_afc()
            if afc is None or not hasattr(afc, 'lanes'):
                return

            lane_obj = afc.lanes.get(lane_name)
            if lane_obj is None:
                return

            # Check if lane has fila_load runout helper
            if not hasattr(lane_obj, 'fila_load'):
                return

            fila_load = lane_obj.fila_load
            if not hasattr(fila_load, 'runout_helper'):
                return

            runout_helper = fila_load.runout_helper
            if not hasattr(runout_helper, 'min_event_systime') or not hasattr(runout_helper, 'event_delay'):
                return

            # Update min_event_systime to allow future switch changes to be detected
            # This mimics how it's done in AFC_extruder.py handle_start_runout()
            runout_helper.min_event_systime = self.reactor.monotonic() + runout_helper.event_delay

            self.logger.debug(
                "Fixed min_event_systime for %s load sensor (workaround for AFC bug)",
                lane_name
            )
        except Exception:
            # Don't crash if this workaround fails - just log it
            self.logger.debug(
                f"Failed to fix min_event_systime for {lane_name} (AFC may not have this lane or sensor)"
            )

    def determine_current_loaded_lane(self, fps_name: str) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """Determine which lane is currently loaded in the specified FPS."""
        fps = self.fpss.get(fps_name)
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")


        # Lane-based detection only
        return self._determine_loaded_lane_for_fps(fps_name, fps)

    def _determine_loaded_lane_for_fps(self, fps_name: str, fps) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """Determine which AFC lane is loaded by asking AFC which lane is loaded to each extruder.

        With load_to_hub: False configuration, filament bypasses OAMS hub sensors and goes
        directly to toolhead. AFC tracks filament position via its own sensors and is the
        authoritative source for what's loaded.
        """
        afc = self._get_afc()
        if afc is None:
            self.logger.warning("State detection: AFC not found")

            return None, None, None

        if not hasattr(afc, 'tools'):
            self.logger.warning("State detection: AFC has no 'tools' attribute")

            return None, None, None

        fps_state = self.current_state.fps_state.get(fps_name)

        # Snapshot of AFC.var.unit (authoritative for lane_loaded mapping)
        snapshot = self._load_afc_var_unit_snapshot()
        snapshot_extruders = {}
        if isinstance(snapshot, dict):
            system = snapshot.get("system", {})
            if isinstance(system, dict):
                extruders = system.get("extruders", {})
                if isinstance(extruders, dict):
                    snapshot_extruders = extruders

        # Check each AFC tool/extruder to see which lane is loaded
        for extruder_name, extruder_obj in afc.tools.items():
            loaded_lane_name = None

            # Prefer snapshot data if available
            extr_state = snapshot_extruders.get(extruder_name, {})
            if isinstance(extr_state, dict):
                loaded_lane_name = extr_state.get("lane_loaded")

            # Fall back to live object if snapshot missing
            if not loaded_lane_name:
                loaded_lane_name = getattr(extruder_obj, 'lane_loaded', None)

            if not loaded_lane_name:
                # Clear last log so a future detection of the same lane after unload will log again
                self._last_logged_detected_lane.pop(extruder_name, None)
                continue

            # Check if this lane is on the current FPS
            lane_fps = self.get_fps_for_afc_lane(loaded_lane_name)
            if lane_fps != fps_name:
                # Lane is loaded, but on a different FPS - clear last log for this extruder
                self._last_logged_detected_lane.pop(extruder_name, None)
                continue  # This lane is on a different FPS

            # Get the lane object
            lane = afc.lanes.get(loaded_lane_name)
            if lane is None:
                self.logger.warning(f"Lane {loaded_lane_name} not found in afc.lanes")
                continue

            # Get the OAMS and bay index for this lane
            unit_str = getattr(lane, "unit", None)
            if not unit_str:
                self.logger.warning(f"Lane {loaded_lane_name} has no unit")
                continue

            # Parse unit and slot
            if isinstance(unit_str, str) and ':' in unit_str:
                base_unit_name, slot_str = unit_str.split(':', 1)
                try:
                    slot_number = int(slot_str)
                except ValueError:
                    self.logger.warning(f"Invalid slot number in unit {unit_str}")
                    continue
            else:
                base_unit_name = str(unit_str)
                slot_number = getattr(lane, "index", None)
                if slot_number is None:
                    self.logger.warning(f"No index found for lane {loaded_lane_name}")
                    continue

            # Convert to bay index
            bay_index = slot_number - 1
            if bay_index < 0:
                self.logger.warning(f"Invalid bay index {bay_index} (slot {slot_number})")
                continue

            # Get OAMS name from AFC unit
            unit_obj = getattr(lane, "unit_obj", None)
            if unit_obj is None:
                units = getattr(afc, "units", {})
                unit_obj = units.get(base_unit_name)
            if unit_obj is None:
                self.logger.warning(f"Unit {base_unit_name} not found")
                continue

            oams_name = getattr(unit_obj, "oams_name", None)
            if not oams_name:
                self.logger.warning(f"Unit {base_unit_name} has no oams_name")
                continue

            # Find OAMS object - check both short and prefixed names
            oam = self.oams.get(oams_name)
            if oam is None:
                oam = self.oams.get(f"oams {oams_name}")

            if oam is None:
                self.logger.warning(f"OAMS {oams_name} not found")
                continue

            # Found loaded lane! Return lane name (e.g., "lane8") not map (e.g., "T4")

            # Map can be retrieved from lane object if needed for display
            last_logged = self._last_logged_detected_lane.get(extruder_name)
            if last_logged != loaded_lane_name:
                self._last_logged_detected_lane[extruder_name] = loaded_lane_name
                self.logger.debug(
                    f"Detected {loaded_lane_name} loaded to {extruder_name} (bay {bay_index} on {oams_name})"
                )

            return loaded_lane_name, oam, bay_index

        # Fallback: if FPS state already tracks a lane as loaded, prefer that
        if fps_state is not None and fps_state.state == FPSLoadState.LOADED and fps_state.current_lane:
            lane = afc.lanes.get(fps_state.current_lane)
            if lane is not None:
                lane_tool_loaded = bool(getattr(lane, "tool_loaded", False))
                lane_loaded_to_hub = bool(getattr(lane, "loaded_to_hub", False))
                lane_load_state = bool(getattr(lane, "load_state", False))
                lane_extruder = getattr(lane, "extruder_obj", None)
                extruder_loaded_lane = getattr(lane_extruder, "lane_loaded", None) if lane_extruder else None
                if (not lane_tool_loaded and not lane_loaded_to_hub and not lane_load_state and
                        extruder_loaded_lane != fps_state.current_lane):
                    self.logger.debug(
                        f"Skipping fps_state fallback for {fps_state.current_lane} on {fps_name}: "
                        "lane has no tool_loaded/loaded_to_hub/load_state flags and extruder lane_loaded is "
                        f"{extruder_loaded_lane or 'None'}"
                    )
                    lane = None

            if lane is not None:
                unit_str = getattr(lane, "unit", None)
                if isinstance(unit_str, str) and ':' in unit_str:
                    base_unit_name, slot_str = unit_str.split(':', 1)
                    try:
                        bay_index = int(slot_str) - 1
                    except ValueError:
                        bay_index = None
                else:
                    base_unit_name = str(unit_str) if unit_str is not None else None
                    try:
                        bay_index = int(getattr(lane, "index", 0)) - 1
                    except Exception:
                        bay_index = None

                oam = None
                if base_unit_name:
                    unit_obj = getattr(lane, "unit_obj", None)
                    if unit_obj is None:
                        units = getattr(afc, "units", {})
                        unit_obj = units.get(base_unit_name)
                    oams_name = getattr(unit_obj, "oams_name", None) if unit_obj is not None else None
                    if oams_name:
                        oam = self.oams.get(oams_name) or self.oams.get(f"oams {oams_name}")

                if oam is not None and bay_index is not None and bay_index >= 0:
                    self.logger.info(
                        f"Using fps_state fallback: {fps_state.current_lane} treated as loaded on {fps_name} (bay {bay_index} on {getattr(oam, 'name', 'unknown')})"
                    )
                    return fps_state.current_lane, oam, bay_index

        # Third fallback: Check all lanes on this FPS for tool_loaded = True
        # This handles "empty shuttle with stale loaded filament" scenario where:
        # - AFC thinks lane_loaded = None (empty shuttle)
        # - But filament is actually stuck in extruder from previous session
        # - lane.tool_loaded = True indicates filament in extruder
        if hasattr(afc, 'lanes'):
            for lane_name, lane_obj in afc.lanes.items():
                # Check if this lane is on the current FPS
                try:
                    lane_fps = self.get_fps_for_afc_lane(lane_name)
                    if lane_fps != fps_name:
                        continue  # Not on this FPS
                except Exception:
                    continue

                # Check if tool_loaded flag is set
                if not getattr(lane_obj, 'tool_loaded', False):
                    continue

                # Found a lane with tool_loaded = True on this FPS!
                # Get OAMS and bay index for this lane
                unit_str = getattr(lane_obj, "unit", None)
                if not unit_str:
                    continue

                # Parse unit and slot
                if isinstance(unit_str, str) and ':' in unit_str:
                    base_unit_name, slot_str = unit_str.split(':', 1)
                    try:
                        bay_index = int(slot_str) - 1
                    except ValueError:
                        continue
                else:
                    base_unit_name = str(unit_str)
                    slot_number = getattr(lane_obj, "index", None)
                    if slot_number is None:
                        continue
                    bay_index = slot_number - 1

                if bay_index < 0:
                    continue

                # Get OAMS object
                unit_obj = getattr(lane_obj, "unit_obj", None)
                if unit_obj is None:
                    units = getattr(afc, "units", {})
                    unit_obj = units.get(base_unit_name)

                oams_name = getattr(unit_obj, "oams_name", None) if unit_obj is not None else None
                if not oams_name:
                    continue

                oam = self.oams.get(oams_name) or self.oams.get(f"oams {oams_name}")
                if oam is None:
                    continue

                # Found it! This lane has tool_loaded = True even though shuttle is empty
                self.logger.info(
                    f"Detected stale filament: {lane_name} has tool_loaded=True on {fps_name} "
                    f"(bay {bay_index} on {oams_name}) but shuttle is empty - needs auto-unload"
                )
                return lane_name, oam, bay_index

        return None, None, None
        
    def register_commands(self):
        # Use load_object to ensure gcode is loaded
        gcode = self.printer.load_object(self.config, "gcode")

        commands = [
            ("OAMSM_UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT, self.cmd_UNLOAD_FILAMENT_help),
            ("OAMSM_LOAD_FILAMENT", self.cmd_LOAD_FILAMENT, self.cmd_LOAD_FILAMENT_help),
            ("OAMSM_FOLLOWER", self.cmd_FOLLOWER, self.cmd_FOLLOWER_help),
            ("OAMSM_FOLLOWER_RESET", self.cmd_FOLLOWER_RESET, self.cmd_FOLLOWER_RESET_help),
            ("OAMSM_CLEAR_ERRORS", self.cmd_CLEAR_ERRORS, self.cmd_CLEAR_ERRORS_help),
            ("OAMSM_CLEAR_LANE_MAPPINGS", self.cmd_CLEAR_LANE_MAPPINGS, self.cmd_CLEAR_LANE_MAPPINGS_help),
            ("OAMSM_STATUS", self.cmd_STATUS, self.cmd_STATUS_help),
        ]
        for cmd_name, handler, help_text in commands:
            gcode.register_command(cmd_name, handler, desc=help_text)

    cmd_CLEAR_ERRORS_help = "Clear OAMS errors and sync all state (hardware sensors, AFC live state, AFC.var.unit)"
    def cmd_CLEAR_ERRORS(self, gcmd):
        monitors_were_running = bool(self.monitor_timers)
        restart_monitors = True
        ready_oams = {name: oam for name, oam in self.oams.items() if self._is_oams_mcu_ready(oam)}
        if not ready_oams:
            restart_monitors = False
            self.logger.info(
                "No OAMS MCUs ready; skipping hardware clears and leaving monitors paused until connectivity returns"
            )
        with self._monitors_suspended("OAMSM_CLEAR_ERRORS", restart_on_exit=False):
            # Clear AFC lane_loaded state first to enable bidirectional sync, but only if
            # we have MCU connectivity and no active loaded state to preserve.
            #
            # State Sync Flow:
            # 1. UNSET_LANE_LOADED -> clears AFC extruder.lane_loaded (breaks stale state)
            # 2. determine_state() -> reads hardware sensors (F1S, hub, etc.)
            # 3. If sensors show lane loaded -> _sync_afc_lane_loaded() writes back to AFC
            # 4. If sensors show empty -> AFC stays unset
            has_loaded_state = any(
                state.state == FPSLoadState.LOADED
                for state in self.current_state.fps_state.values()
            )
            has_loaded_state = has_loaded_state or any(
                getattr(oam, "current_spool", None) is not None
                for oam in ready_oams.values()
            )
            if ready_oams and not has_loaded_state:
                try:
                    afc = self._get_afc()
                    afc_function = getattr(afc, "function", None) if afc is not None else None
                    unset_fn = getattr(afc_function, "unset_lane_loaded", None)
                    if unset_fn is not None:
                        unset_fn()
                        self.logger.info("Cleared AFC lane_loaded state - will resync from hardware sensors")
                    else:
                        gcode = self.printer.lookup_object("gcode")
                        if gcode:
                            gcode.run_script_from_command("UNSET_LANE_LOADED")
                            self.logger.info("Cleared AFC lane_loaded state via G-code fallback")
                except Exception as e:
                    # Don't block CLEAR_ERRORS if UNSET fails, but log it
                    self.logger.warning(f"Failed to clear AFC lane state during OAMSM_CLEAR_ERRORS: {e}")
            else:
                self.logger.debug(
                    "Skipped clearing AFC lane_loaded state (filament already loaded or MCU not ready)"
                )

            # Reset all runout monitors to clear COASTING and other states
            for fps_name, monitor in list(self.runout_monitors.items()):
                try:
                    monitor.reset()
                except Exception:
                    restart_monitors = False
                    self.logger.error(f"Failed to reset runout monitor for {fps_name}")

            # Clear all FPS state error flags and tracking
            for fps_name, fps_state in self.current_state.fps_state.items():
                fps_state.clear_encoder_samples()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                fps_state.reset_engagement_tracking()  # Clear engagement state too
                fps_state.afc_delegation_active = False
                fps_state.afc_delegation_until = 0.0
                fps_state.state = FPSLoadState.UNLOADED
                fps_state.current_lane = None
                fps_state.current_oams = None
                fps_state.current_spool_idx = None
                fps_state.following = False
                fps_state.direction = 0
                fps_state.since = self.reactor.monotonic()
                self._cancel_post_load_pressure_check(fps_state)

            # Clear OAMS hardware errors and LEDs
            # CRITICAL: Use reactor.pause() to wait for MCU responses between commands
            # This ensures LEDs actually clear before determine_state() runs
            for oams_name, oam in ready_oams.items():
                if not self._is_oams_mcu_ready(oam):
                    restart_monitors = False
                    continue
                try:
                    # Step 0: Abort any in-flight action to clear "busy" state
                    oam.abort_current_action()
                    self.reactor.pause(self.reactor.monotonic() + 0.1)
                    if not self._is_oams_mcu_ready(oam):
                        restart_monitors = False
                        continue

                    # Step 1: Clear hardware errors (this also clears all LED errors)
                    # Note: clear_errors() already loops through all LEDs and clears them
                    oam.clear_errors()
                    self.reactor.pause(self.reactor.monotonic() + 0.2)  # Wait for MCU to process all commands
                    if not self._is_oams_mcu_ready(oam):
                        restart_monitors = False
                        continue

                    # Step 2: Clear LED tracking state
                    bay_count = getattr(oam, "num_spools", 4) or 4
                    for bay_idx in range(bay_count):
                        led_key = f"{oams_name}:{bay_idx}"
                        self.led_error_state[led_key] = 0

                    self.logger.debug(f"Cleared {bay_count} LED errors on {oams_name}")
                except Exception as e:
                    restart_monitors = False
                    self.logger.error(f"Failed to clear errors on {oams_name}: {e}")

            # NOTE: LED restore moved to AFTER state sync so it uses correct hardware-reconciled state
            # instead of potentially stale AFC state

            # Preserve lane mappings during error clearing to avoid losing lane state.
            # We still surface any active redirects so an operator can clear them explicitly
            # with OAMSM_CLEAR_LANE_MAPPINGS if desired.
            try:
                afc = self.printer.lookup_object('AFC')
                if afc and hasattr(afc, 'lanes'):
                    retained_maps = []
                    for lane_name, lane in afc.lanes.items():
                        if hasattr(lane, 'map') and lane.map is not None:
                            if isinstance(lane.map, str) and lane.map.startswith('lane') and lane.map != lane_name:
                                retained_maps.append(f"{lane_name}->{lane.map}")

                    if retained_maps:
                        self.logger.info(
                            "Retaining OAMS lane mappings during OAMSM_CLEAR_ERRORS: %s. "
                            "Use OAMSM_CLEAR_LANE_MAPPINGS to clear these redirects if needed.",
                            ", ".join(retained_maps),
                        )
            except Exception:
                self.logger.debug("Could not inspect lane mappings (AFC not available or no mappings set)")


            # Re-detect state from hardware sensors
            if ready_oams:
                try:
                    self.determine_state()
                except Exception:
                    restart_monitors = False
                    self.logger.error("State detection failed during OAMSM_CLEAR_ERRORS")

            # Sync state between OAMS hardware and AFC to prevent state desync issues
            # This validates that AFC's lane_loaded matches OAMS hardware current_spool
            if ready_oams:
                try:
                    self.sync_state_with_afc()
                except Exception:
                    restart_monitors = False
                    self.logger.error("State sync with AFC failed during OAMSM_CLEAR_ERRORS")

            # Comprehensive 3-way state reconciliation (like PREP does at startup)
            # This reconciles: Hardware sensors <-> AFC live state <-> AFC.var.unit
            # Uses hardware sensors as ground truth to fix any stale state
            if ready_oams:
                try:
                    afc = self._get_afc()
                    if afc and hasattr(afc, 'units'):
                        synced_units = 0
                        for unit_name, unit_obj in afc.units.items():
                            # Only sync OpenAMS units
                            if not hasattr(unit_obj, 'oams_name'):
                                continue

                            oams_name = unit_obj.oams_name
                            # Check both possible name formats: "oams1" and "oams oams1"
                            oams_ready = (oams_name in ready_oams) or (f"oams {oams_name}" in ready_oams)
                            if not oams_ready:
                                self.logger.debug(f"Skipping state reconciliation for {unit_name} - MCU not ready")
                                continue

                            # Call the AFC_OpenAMS comprehensive sync method
                            if hasattr(unit_obj, '_sync_afc_from_hardware_at_startup'):
                                try:
                                    unit_obj._sync_afc_from_hardware_at_startup()
                                    synced_units += 1
                                    self.logger.debug(f"Reconciled state for {unit_name} with hardware sensors")
                                except Exception as e:
                                    self.logger.warning(f"Failed to reconcile state for {unit_name}: {e}")

                        if synced_units > 0:
                            self.logger.info(f"Reconciled state for {synced_units} OpenAMS unit(s) with hardware sensors")
                except Exception as e:
                    restart_monitors = False
                    self.logger.error(f"Failed to reconcile hardware/AFC state during OAMSM_CLEAR_ERRORS: {e}")
                    import traceback
                    self.logger.debug(f"Traceback: {traceback.format_exc()}")

            # NOTE: We do NOT load from AFC.var.unit file here because:
            # 1. determine_state() already read from hardware sensors (ground truth)
            # 2. _sync_afc_from_hardware_at_startup() reconciled AFC with hardware
            # 3. Loading from file would overwrite correct state with potentially stale data
            # The hardware sensors are the authoritative source, not the saved file.

            # Sync virtual tool sensors to match the refreshed lane state.
            try:
                self._sync_virtual_tool_sensors()
            except Exception as e:
                restart_monitors = False
                self.logger.error(f"Failed to sync virtual tool sensors during OAMSM_CLEAR_ERRORS: {e}")
                import traceback
                self.logger.debug(f"Traceback: {traceback.format_exc()}")

            # Restore normal LED states based on NOW-CORRECT hardware-reconciled state
            # This happens AFTER state sync so LEDs reflect actual hardware state, not stale AFC state
            try:
                afc = self.printer.lookup_object('AFC', None)
                if afc and hasattr(afc, 'units'):
                    for unit_name, unit_obj in afc.units.items():
                        # Only process OpenAMS units
                        if not hasattr(unit_obj, 'oams_name'):
                            continue

                        oams_name = unit_obj.oams_name
                        if oams_name not in ready_oams:
                            continue

                        # Restore LED state for each lane based on reconciled state
                        if hasattr(unit_obj, 'lanes'):
                            for lane_name, lane in unit_obj.lanes.items():
                                try:
                                    # Check lane state and restore appropriate LED
                                    if getattr(lane, 'tool_loaded', False):
                                        # Lane is loaded to toolhead - set tool loaded LED
                                        unit_obj.lane_tool_loaded(lane)
                                        self.logger.debug(f"Restored tool loaded LED for {lane_name}")
                                    elif getattr(lane, 'load_state', False):
                                        # Lane is loaded to hub - set loaded LED
                                        unit_obj.lane_loaded(lane)
                                        self.logger.debug(f"Restored loaded LED for {lane_name}")
                                    else:
                                        # Lane is unloaded - set unloaded LED
                                        unit_obj.lane_unloaded(lane)
                                        self.logger.debug(f"Restored unloaded LED for {lane_name}")
                                except Exception as e:
                                    self.logger.warning(f"Failed to restore LED for {lane_name}: {e}")

                    self.logger.debug("Restored LED states based on hardware-reconciled state")
            except Exception as e:
                self.logger.error(f"Failed to restore LED states: {e}")

            # Clear all manual follower overrides and coast state - return to automatic hub sensor control
            # Also clear last state tracking so follower state is refreshed from actual sensors
            for oams_name in self.oams.keys():
                state = self._get_follower_state(oams_name)
                state.last_state = None  # Force state refresh
                state.coasting = False
                state.coast_start_pos = 0.0
                state.had_filament = False

            # Clear LED state tracking so LEDs are refreshed from actual state
            self.led_error_state.clear()

            self.logger.debug(
                "Cleared all manual follower overrides, coast state, LED state, and state tracking"
            )

            # Force followers on so CLEAR_ERRORS never leaves them disabled, even if sensors
            # are empty or state is still settling. This keeps manual extrusion available
            # while the operator recovers from the error condition.
            if ready_oams:
                try:
                    self._force_enable_followers(ready_oams)
                except Exception:
                    restart_monitors = False
                    self.logger.error("Failed to force followers on during OAMSM_CLEAR_ERRORS")


            # After clearing errors and detecting state, ensure followers are enabled for any
            # lanes that have filament loaded to the hub (even if not loaded to toolhead)
            # This keeps filament pressure up for manual operations during troubleshooting
            if ready_oams:
                try:
                    self._ensure_followers_for_loaded_hubs()
                except Exception:
                    restart_monitors = False
                    self.logger.error("Failed to refresh followers after OAMSM_CLEAR_ERRORS")


        if monitors_were_running:
            if restart_monitors:
                try:
                    self.start_monitors()
                except Exception:
                    self.logger.error("Failed to restart monitors after OAMSM_CLEAR_ERRORS")

            else:
                self.logger.info(
                    "Leaving monitors paused after OAMSM_CLEAR_ERRORS due to earlier errors; run OAMSM_STATUS once issues are resolved"
                )

        # Build a helpful summary message
        summary_parts = []
        summary_parts.append("[OK] OAMS system cleared and ready")

        # Show how many OAMS units were cleared
        if ready_oams:
            oams_names = ", ".join(ready_oams.keys())
            summary_parts.append(f"  Cleared {len(ready_oams)} unit(s): {oams_names}")
        else:
            summary_parts.append("  WARNING: No OAMS MCUs were ready - check connections")

        # Show what's loaded
        loaded_lanes = []
        for fps_name, fps_state in self.current_state.fps_state.items():
            if fps_state.state == FPSLoadState.LOADED and fps_state.current_lane:
                loaded_lanes.append(f"{fps_state.current_lane}")

        if loaded_lanes:
            summary_parts.append(f"  Loaded: {', '.join(loaded_lanes)}")
        else:
            summary_parts.append("  No lanes loaded")

        # Show monitor status
        if restart_monitors and monitors_were_running:
            summary_parts.append("  Monitors: Running")
        elif not restart_monitors:
            summary_parts.append("  Monitors: Paused (errors occurred)")
        else:
            summary_parts.append("  Monitors: Not running")

        gcmd.respond_info("\n".join(summary_parts))

    def _load_afc_var_unit_snapshot(self) -> Optional[Dict[str, Any]]:
        """Load the current AFC.var.unit snapshot from the JSON file on disk."""
        import json
        import os

        afc = self._get_afc()
        if afc is None:
            return None

        try:
            # AFC saves to VarFile + '.unit' (e.g., printer_data/config/AFC/AFC.var.unit)
            var_file_path = getattr(afc, "VarFile", None)
            if not var_file_path:
                self.logger.debug("AFC.VarFile path not found")
                return None

            unit_file = f"{var_file_path}.unit"

            # Check if file exists and has content
            if not os.path.exists(unit_file) or os.stat(unit_file).st_size == 0:
                self.logger.debug(f"AFC.var.unit file not found or empty: {unit_file}")
                return None

            # Load and parse JSON
            with open(unit_file, 'r') as f:
                snapshot = json.load(f)

            if isinstance(snapshot, dict):
                return snapshot
            else:
                self.logger.debug("AFC.var.unit file contents are not a dictionary")
                return None

        except json.JSONDecodeError as e:
            self.logger.debug(f"Failed to parse AFC.var.unit JSON: {e}")
            return None
        except Exception as e:
            self.logger.debug(f"Failed to read AFC.var.unit from file: {e}")
            return None

    def _sync_virtual_tool_sensors(self, gcmd: Optional[Any] = None) -> None:
        """Sync AFC virtual tool sensors based on current lane state."""
        afc = self._get_afc()
        if afc is None:
            if gcmd is not None:
                gcmd.respond_info("AFC: Not found!")
            return

        synced_count = 0
        try:
            units = getattr(afc, 'units', {})
            eventtime = self.reactor.monotonic()

            for unit_name, unit_obj in units.items():
                # Check if this is an OpenAMS unit with virtual sensor sync capability
                if hasattr(unit_obj, '_sync_virtual_tool_sensor'):
                    try:
                        # Force update to ensure sensor state is corrected after reboot
                        unit_obj._sync_virtual_tool_sensor(eventtime, force=True)
                        synced_count += 1
                    except Exception:
                        self.logger.error(f"Failed to sync virtual tool sensor for unit {unit_name}")
                        if gcmd is not None:
                            gcmd.respond_info(f"  Warning: Failed to sync virtual sensor for {unit_name}")

            if gcmd is not None:
                if synced_count > 0:
                    gcmd.respond_info(f"Successfully synced {synced_count} virtual tool sensor(s)")
                else:
                    gcmd.respond_info("No OpenAMS units found with virtual sensors to sync")
        except Exception:
            self.logger.error("Failed to sync virtual tool sensors")
            if gcmd is not None:
                gcmd.respond_info("  Error during virtual sensor sync - check klippy.log")

    def _get_lane_snapshot(self, lane_name: Optional[str], unit_name: Optional[str] = None) -> Optional[Dict[str, Any]]:
        if not lane_name:
            return None

        snapshot = self._load_afc_var_unit_snapshot()
        if not isinstance(snapshot, dict):
            return None

        unit_key = unit_name
        if isinstance(unit_key, str) and ":" in unit_key:
            unit_key = unit_key.split(":", 1)[0]

        if unit_key:
            unit_data = snapshot.get(unit_key)
            if isinstance(unit_data, dict):
                lane_data = unit_data.get(lane_name)
                if isinstance(lane_data, dict):
                    return lane_data

        for unit_key, unit_data in snapshot.items():
            if unit_key in ("system", "Tools") or not isinstance(unit_data, dict):
                continue
            lane_data = unit_data.get(lane_name)
            if isinstance(lane_data, dict):
                return lane_data

        return None

    def _sync_openams_sensors_for_oams(
        self,
        oams_name: str,
        eventtime: float,
        *,
        allow_f1s_updates: bool,
        allow_lane_clear: bool,
    ) -> None:
        afc = self._get_afc()
        if afc is None or not hasattr(afc, "units"):
            return
        for unit_obj in afc.units.values():
            if not hasattr(unit_obj, "oams_name") or unit_obj.oams_name != oams_name:
                continue
            if hasattr(unit_obj, "sync_openams_sensors"):
                try:
                    unit_obj.sync_openams_sensors(
                        eventtime,
                        sync_hub=True,
                        sync_f1s=allow_f1s_updates,
                        allow_lane_clear=allow_lane_clear,
                    )
                except Exception:
                    self.logger.error(f"Failed to sync OpenAMS sensors for {oams_name}")

    def _refresh_state_from_afc_snapshot(self) -> None:
        """Update FPS state from the latest AFC.var.unit snapshot."""

        snapshot = self._load_afc_var_unit_snapshot()
        if not snapshot:
            self.logger.debug("No AFC.var.unit snapshot available for state refresh")

            return

        system_state = snapshot.get("system", {}) if isinstance(snapshot, dict) else {}
        extruders = system_state.get("extruders", {}) if isinstance(system_state, dict) else {}
        if not isinstance(extruders, dict):
            self.logger.debug("AFC.var.unit snapshot missing extruder data")

            return

        afc = self._get_afc()
        for extruder_name, extruder_state in extruders.items():
            try:
                if not isinstance(extruder_state, dict):
                    continue

                lane_name = extruder_state.get("lane_loaded")

                if not lane_name:
                    continue

                fps_name = self.get_fps_for_afc_lane(lane_name)
                if not fps_name:
                    continue

                fps_state = self.current_state.fps_state.get(fps_name)
                if fps_state is None:
                    continue

                lane_obj = getattr(afc, "lanes", {}).get(lane_name) if afc else None
                lane_unit = getattr(lane_obj, "unit", None) if lane_obj is not None else None
                lane_data = self._get_lane_snapshot(lane_name, unit_name=lane_unit)

                spool_idx = None
                if lane_obj is not None:
                    unit_str = getattr(lane_obj, "unit", None)
                    if isinstance(unit_str, str) and ":" in unit_str:
                        _, slot_str = unit_str.split(":", 1)
                        try:
                            spool_idx = int(slot_str) - 1
                        except ValueError:
                            spool_idx = None
                    elif hasattr(lane_obj, "index"):
                        try:
                            spool_idx = int(getattr(lane_obj, "index")) - 1
                        except Exception:
                            spool_idx = None

                if spool_idx is None and isinstance(lane_data, dict):
                    try:
                        spool_idx = int(lane_data.get("lane", 0)) - 1
                    except Exception:
                        spool_idx = None

                if spool_idx is None or spool_idx < 0:
                    self.logger.debug(f"Skipping AFC.var.unit lane {lane_name} due to missing spool index")
                    continue

                # Resolve the OAMS object backing this lane
                oams_obj = None
                oams_name = None
                if lane_obj is not None:
                    unit_obj = getattr(lane_obj, "unit_obj", None)
                    if unit_obj is None and afc is not None:
                        unit_obj = getattr(afc, "units", {}).get(getattr(lane_obj, "unit", None))
                    oams_name = getattr(unit_obj, "oams_name", None) if unit_obj is not None else None

                if not oams_name and isinstance(lane_data, dict):
                    oams_name = lane_data.get("unit")


                if oams_name:
                    oams_obj = self.oams.get(oams_name) or self.oams.get(f"oams {oams_name}")


                if oams_obj is None:
                    self.logger.debug(f"Could not resolve OAMS for lane {lane_name} from AFC.var.unit")
                    continue

                # Update FPS state tracking
                fps_state.current_lane = lane_name
                fps_state.current_oams = getattr(oams_obj, "name", oams_name)
                fps_state.current_spool_idx = spool_idx
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()

                # Only enable follower if OAMS MCU is ready
                # During CLEAR_ERRORS, some MCUs might not be ready yet
                if self._is_oams_mcu_ready(oams_obj):
                    try:
                        self._ensure_forward_follower(fps_name, fps_state, "AFC.var.unit refresh")
                    except Exception as e:
                        self.logger.warning(f"Failed to enable follower for {fps_name} during state refresh: {e}")
                else:
                    self.logger.debug(f"Skipping follower enable for {fps_name} - OAMS MCU not ready")

                self.logger.debug(
                    f"Restored state: {lane_name} loaded on {fps_state.current_oams} slot {spool_idx}"
                )
            except Exception as e:
                # Don't let one lane failure abort the whole refresh
                lane_info = extruder_state.get("lane_loaded", "unknown") if isinstance(extruder_state, dict) else "unknown"
                self.logger.error(f"Failed to refresh state for extruder {extruder_name} lane {lane_info}: {e}")
                import traceback
                self.logger.debug(f"Traceback: {traceback.format_exc()}")
                continue

    cmd_CLEAR_LANE_MAPPINGS_help = "Clear OAMS cross-extruder lane mappings (call after RESET_AFC_MAPPING)"
    def cmd_CLEAR_LANE_MAPPINGS(self, gcmd):
        """Clear any lane.map redirects created by OAMS cross-extruder runouts.

        Use this after RESET_AFC_MAPPING or when you explicitly want to clear
        redirects; OAMSM_CLEAR_ERRORS leaves mappings intact.

        Useful to add to your PRINT_END or CANCEL_PRINT macros after RESET_AFC_MAPPING:

        Example:
            RESET_AFC_MAPPING
            OAMSM_CLEAR_LANE_MAPPINGS
        """
        cleared_count = 0
        try:
            afc = self.printer.lookup_object('AFC')
            if afc and hasattr(afc, 'lanes'):
                for lane_name, lane in afc.lanes.items():
                    if hasattr(lane, 'map') and lane.map is not None:
                        # Check if map points to another lane (cross-extruder redirect)
                        if isinstance(lane.map, str) and lane.map.startswith('lane') and lane.map != lane_name:
                            self.logger.info(f"Clearing OAMS lane mapping: {lane_name} -> {lane.map}")
                            gcmd.respond_info(f"Clearing OAMS lane mapping: {lane_name} -> {lane.map}")

                            lane.map = None
                            cleared_count += 1
        except Exception:
            gcmd.respond_info("Could not clear lane mappings (AFC not available)")

            return

        if cleared_count > 0:
            gcmd.respond_info(f"Cleared {cleared_count} OAMS lane mapping(s)")

        else:
            gcmd.respond_info("No OAMS lane mappings to clear")

    cmd_STATUS_help = "Show OAMS manager state and run state detection diagnostics"
    def cmd_STATUS(self, gcmd):
        """Diagnostic command to show current state and test state detection."""
        afc = self._get_afc()

        gcmd.respond_info("=== OAMS Manager Status ===")


        # Show AFC info
        if afc is None:
            gcmd.respond_info("AFC: Not found!")

        else:
            gcmd.respond_info(f"AFC: Found, has {len(getattr(afc, 'tools', {}))} tools, {len(getattr(afc, 'lanes', {}))} lanes")


            # Show what AFC thinks is loaded
            if hasattr(afc, 'tools'):
                for tool_name, tool_obj in afc.tools.items():
                    lane_loaded = getattr(tool_obj, 'lane_loaded', None)
                    gcmd.respond_info(f"  Tool {tool_name}: lane_loaded={lane_loaded}")


        # Show FPS states
        for fps_name, fps_state in self.current_state.fps_state.items():
            gcmd.respond_info(f"\n{fps_name}:")

            gcmd.respond_info(f"  State: {fps_state.state}")

            gcmd.respond_info(f"  Current OAMS: {fps_state.current_oams}")

            gcmd.respond_info(f"  Current Lane: {fps_state.current_lane}")

            gcmd.respond_info(f"  Spool Index: {fps_state.current_spool_idx}")

            gcmd.respond_info(f"  Following: {fps_state.following}, Direction: {fps_state.direction}")


        # Run state detection and show results
        gcmd.respond_info("\n=== Running State Detection ===")

        self.determine_state()

        # Show state after detection
        for fps_name, fps_state in self.current_state.fps_state.items():
            gcmd.respond_info(f"\nAfter detection - {fps_name}:")

            gcmd.respond_info(f"  State: {fps_state.state}")

            gcmd.respond_info(f"  Current OAMS: {fps_state.current_oams}")

            gcmd.respond_info(f"  Current Lane: {fps_state.current_lane}")

            gcmd.respond_info(f"  Spool Index: {fps_state.current_spool_idx}")


        # Sync virtual tool sensors based on which lanes are actually loaded
        # This fixes virtual sensor state after reboot (e.g., extruder4 showing filament when empty)
        gcmd.respond_info("\n=== Syncing Virtual Tool Sensors ===")
        self._sync_virtual_tool_sensors(gcmd)

    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        # DIRECTION is optional when disabling (ENABLE=0), defaults to 0
        direction = gcmd.get_int('DIRECTION', 0)
        fps_name = "fps " + gcmd.get('FPS')
        # Optional OAMS parameter to override auto-detection
        oams_override = gcmd.get('OAMS', None)

        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")

            return

        fps_state = self.current_state.fps_state[fps_name]

        # Allow enabling follower when UNLOADED (before load starts) or LOADED
        # Only block during active LOADING/UNLOADING operations
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            self.logger.debug(f"OAMSM_FOLLOWER ignored because {fps_name} is busy")

            return

        # Apply OAMS override if provided
        if oams_override:
            fps_state.current_oams = oams_override
            self.logger.info(f"OAMSM_FOLLOWER: overriding current_oams to {oams_override} for {fps_name}")

        if not fps_state.current_oams:
            # Skip auto-detection during tool operations - trust AFC state machine
            afc = self.printer.lookup_object("AFC", None)
            is_tool_operation = getattr(afc, 'in_toolchange', False) if afc else False

            if not is_tool_operation:
                try:
                    detected_lane, detected_oams, detected_spool_idx = self.determine_current_loaded_lane(fps_name)
                except Exception:
                    detected_lane, detected_oams, detected_spool_idx = None, None, None
                if detected_oams is not None:
                    fps_state.current_oams = detected_oams.name
                    fps_state.current_spool_idx = detected_spool_idx
                    fps_state.current_lane = detected_lane

        # When disabling (ENABLE=0), disable follower and keep state tracking
        if not enable:
            # If already unloaded or no OAMS, just mark as not following and return
            if not fps_state.current_oams:
                fps_state.following = False
                self.logger.info(f"Follower disable requested on {fps_name} but no OAMS loaded, marking as not following")
                return

            oams_obj = self._get_oams_object(fps_state.current_oams)
            if oams_obj:
                try:
                    if fps_state.current_oams != oams_obj.name:
                        fps_state.current_oams = oams_obj.name
                    state = self._get_follower_state(fps_state.current_oams)
                    oams_obj.set_oams_follower(0, direction)
                    fps_state.following = False
                    # Update state tracker to avoid redundant commands
                    state.last_state = (0, direction)
                    self.logger.debug(f"Disabled follower on {fps_name}")
                except Exception:
                    self.logger.error(f"Failed to disable follower on {fps_state.current_oams}")
                    gcmd.respond_info(f"Failed to disable follower. Check logs.")

            else:
                # OAMS not found but mark as not following anyway
                fps_state.following = False
                self.logger.info(f"Follower disable: OAMS {fps_state.current_oams} not found, marking as not following")
            return

        # When enabling, we need a valid OAMS
        oams_obj = self._get_oams_object(fps_state.current_oams)
        if oams_obj is None:
            if fps_state.current_oams is None:
                gcmd.respond_info(
                    f"Cannot enable follower on {fps_name}: no OAMS/lane currently loaded. "
                    f"Load filament first or specify OAMS=<oams_name> to override."
                )
            else:
                gcmd.respond_info(f"OAMS {fps_state.current_oams} is not available for {fps_name}")
            return

        try:
            self.logger.debug(f"OAMSM_FOLLOWER: enabling follower on {fps_name}, direction={direction}")
            if fps_state.current_oams != oams_obj.name:
                fps_state.current_oams = oams_obj.name
            state = self._get_follower_state(fps_state.current_oams)
            oams_obj.set_oams_follower(enable, direction)
            fps_state.following = bool(enable)
            fps_state.direction = direction
            # Update state tracker to avoid redundant commands
            state.last_state = (enable, direction)
            self.logger.debug(f"OAMSM_FOLLOWER: successfully enabled follower on {fps_name}")
        except Exception:
            self.logger.error(f"Failed to set follower on {fps_state.current_oams}")
            gcmd.respond_info(f"Failed to set follower. Check logs.")

    cmd_FOLLOWER_RESET_help = "Return follower to automatic control based on hub sensors"
    def cmd_FOLLOWER_RESET(self, gcmd):
        oams_param = gcmd.get("OAMS", None)
        fps_name = "fps " + gcmd.get('FPS')

        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")

            return

        fps_state = self.current_state.fps_state[fps_name]

        # If OAMS parameter provided, use it directly
        if oams_param:
            fps_state.current_oams = self._normalize_oams_name(oams_param)
        elif not fps_state.current_oams:
            gcmd.respond_info(f"No OAMS associated with {fps_name}")

            return

        self.logger.info(f"Follower reset requested for {fps_name}")
        gcmd.respond_info(f"Follower reset requested for {fps_name}")


        # Immediately update follower based on current hub sensor state
        oams_obj = self._get_oams_object(fps_state.current_oams)
        if oams_obj:
            self._update_follower_for_oams(fps_state.current_oams, oams_obj)
            gcmd.respond_info(f"Follower state updated based on current hub sensors")

    def get_fps_for_afc_lane(self, lane_name: str) -> Optional[str]:
        """Get the FPS name for an AFC lane by querying its unit configuration.

        Returns the FPS name (e.g., "fps fps1") or None if not found.
        Uses cached mapping when available for performance.
        """
        # OPTIMIZATION: Check cache first
        cached = self._lane_to_fps_cache.get(lane_name)
        if cached is not None:
            return cached

        # Cache miss - compute and cache the result
        fps_name = self._compute_fps_for_afc_lane(lane_name)
        if fps_name is not None:
            self._lane_to_fps_cache[lane_name] = fps_name
        return fps_name

    def _compute_fps_for_afc_lane(self, lane_name: str) -> Optional[str]:
        """Compute the FPS name for an AFC lane (internal helper for caching)."""
        afc = self._get_afc()
        if afc is None:
            return None

        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None

        # Get the unit string (e.g., "AMS_1:1")

        unit_str = getattr(lane, "unit", None)
        if not unit_str or not isinstance(unit_str, str):
            return None

        # Extract base unit name (e.g., "AMS_1" from "AMS_1:1")

        if ':' in unit_str:
            base_unit_name = unit_str.split(':')[0]
        else:
            base_unit_name = unit_str

        # Look up the AFC unit object
        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None:
            units = getattr(afc, "units", {})
            unit_obj = units.get(base_unit_name)

        if unit_obj is None:
            return None

        # Get the OAMS name from the unit (e.g., "oams1")

        oams_name = getattr(unit_obj, "oams_name", None)
        if not oams_name:
            return None

        # Find which FPS has this OAMS
        for fps_name, fps in self.fpss.items():
            if hasattr(fps, "oams"):
                fps_oams = fps.oams
                # fps.oams could be a list or a single oams object
                if isinstance(fps_oams, list):
                    for oam in fps_oams:
                        oam_name_full = getattr(oam, "name", None)
                        # OAMS objects can be registered as "oams1", "oams oams1", or "OAMS oams1"
                        if (oam_name_full == oams_name or
                            oam_name_full == f"oams {oams_name}" or
                            oam_name_full == f"OAMS {oams_name}"):
                            return fps_name
                else:
                    oam_name_check = getattr(fps_oams, "name", None)
                    if (oam_name_check == oams_name or
                        oam_name_check == f"oams {oams_name}" or
                        oam_name_check == f"OAMS {oams_name}"):
                        return fps_name

        return None

    def _validate_afc_oams_integration(self, afc) -> None:
        """Validate AFC lane configs match OAMS hardware configuration.

        Checks for common integration issues:
        - Lanes without unit definitions
        - Lanes referencing non-existent AFC units
        - Units without OAMS names
        - OAMS names that don't exist in OAMS manager
        - Lanes that can't be mapped to any FPS
        """
        lanes = getattr(afc, "lanes", {})
        if not lanes:
            return

        issues = []
        valid_lanes = 0

        for lane_name, lane in lanes.items():
            # Check lane has valid unit
            unit_str = getattr(lane, "unit", None)
            if not unit_str:
                issues.append(f"Lane {lane_name} has no unit defined")

                continue

            # Parse unit string to get base unit name
            if isinstance(unit_str, str) and ':' in unit_str:
                base_unit_name = unit_str.split(':')[0]
            else:
                base_unit_name = str(unit_str)

            # Check unit exists in AFC
            unit_obj = getattr(lane, "unit_obj", None)
            if unit_obj is None:
                units = getattr(afc, "units", {})
                unit_obj = units.get(base_unit_name)

            if unit_obj is None:
                issues.append(f"Lane {lane_name} references non-existent AFC unit '{base_unit_name}'")

                continue

            # Check unit has OAMS name
            oams_name = getattr(unit_obj, "oams_name", None)
            if not oams_name:
                issues.append(f"AFC unit {base_unit_name} has no oams_name defined")

                continue

            # Check OAMS exists in OAMS manager
            oam = self.oams.get(oams_name)
            if oam is None:
                oam = self.oams.get(f"oams {oams_name}")

            if oam is None:
                oam = self.oams.get(f"OAMS {oams_name}")

            if oam is None:
                issues.append(f"Lane {lane_name} references OAMS '{oams_name}' which doesn't exist in OAMS manager")

                continue

            # Check FPS can be found (use cache since we just built it)
            fps_name = self._lane_to_fps_cache.get(lane_name)
            if not fps_name:
                issues.append(f"Lane {lane_name} cannot be mapped to any FPS (OAMS {oams_name} not found in any FPS config)")

                continue

            valid_lanes += 1

        # Log results
        if issues:
            self.logger.warning(f"AFC-OAMS integration validation found {len(issues)} issue(s):")
            for issue in issues:
                self.logger.warning(f"  - {issue}")
            if valid_lanes > 0:
                self.logger.info(f"AFC-OAMS integration: {valid_lanes} lanes configured correctly")
        else:
            self.logger.info(f"AFC-OAMS integration validated: {valid_lanes} lanes configured correctly")
    def _ensure_afc_lane_cache(self, afc) -> None:
        """Build caches for AFC lane metadata and mappings."""
        lanes = getattr(afc, "lanes", {})
        cache_built = False

        for lane_name, lane in lanes.items():
            # Cache unit mapping
            unit_name = getattr(lane, "unit", None)
            if unit_name and lane_name not in self._lane_unit_map:
                self._lane_unit_map[lane_name] = unit_name

            # OPTIMIZATION: Pre-populate lane?FPS cache
            if lane_name not in self._lane_to_fps_cache:
                fps_name = self._compute_fps_for_afc_lane(lane_name)
                if fps_name is not None:
                    self._lane_to_fps_cache[lane_name] = fps_name
                    cache_built = True

        # Validate AFC-OAMS integration after building cache
        if cache_built and not self._afc_logged:
            self._validate_afc_oams_integration(afc)

    def _resolve_afc_lane_name(self, afc, identifier: Optional[str]) -> Optional[str]:
        """Resolve lane identifiers or map aliases (e.g., T#) to AFC lane names."""
        if not identifier:
            return None

        lanes = getattr(afc, "lanes", {})
        lookup = identifier.strip()
        if lookup in lanes:
            return lookup

        lowered = lookup.lower()
        for lane_name, lane in lanes.items():
            if lane_name.lower() == lowered:
                return lane_name

            lane_map = getattr(lane, "map", None)
            if not isinstance(lane_map, str):
                continue

            if lane_map == lookup or lane_map.lower() == lowered:
                return lane_name

            normalized_map = lane_map.replace(" ", "").lower()
            normalized_lookup = lookup.replace(" ", "").lower()
            if normalized_map == normalized_lookup:
                return lane_name

        return None

    def _resolve_lane_for_state(self, fps_state: 'FPSState', lane_name: Optional[str], afc) -> Tuple[Optional[str], Optional[str]]:
        """Resolve lane name from FPS state. Returns (lane_name, None) - group support removed."""
        # If lane_name provided, return it
        if lane_name:
            return lane_name, None

        # Try to get from current state (legacy location-based lookup)
        if fps_state.current_oams and fps_state.current_spool_idx is not None:
            located_lane = self._lane_by_location.get((fps_state.current_oams, fps_state.current_spool_idx))
            if located_lane:
                return located_lane, None

        return None, None

    def _get_lane_extruder_name(self, lane) -> Optional[str]:
        if lane is None:
            return None
        extruder_obj = getattr(lane, "extruder_obj", None)
        extruder_name = getattr(lane, "extruder_name", None)
        name = None
        if extruder_obj is not None:
            name = getattr(extruder_obj, "name", None)
        if not name and extruder_name:
            name = extruder_name
        return _normalize_extruder_name(name)

    def _get_oams_object(self, oams_name: Optional[str]):
        if not oams_name:
            return None
        if oams_name in self.oams:
            return self.oams.get(oams_name)
        prefixed = f"oams {oams_name}"
        if prefixed in self.oams:
            return self.oams.get(prefixed)
        if oams_name.startswith("oams "):
            unprefixed = oams_name[5:]
            return self.oams.get(unprefixed)
        return None

    def _normalize_oams_name(self, oams_name: Optional[str], oams_obj: Optional[Any] = None) -> Optional[str]:
        if not oams_name:
            return oams_name

        if oams_name in self.oams:
            return oams_name

        if oams_obj is not None:
            obj_name = getattr(oams_obj, "name", None)
            if obj_name in self.oams:
                return obj_name

        prefixed = f"oams {oams_name}"
        if prefixed in self.oams:
            return prefixed

        if oams_name.startswith("oams "):
            unprefixed = oams_name[5:]
            if unprefixed in self.oams:
                return unprefixed

        return oams_name

    def _get_afc(self):
        # OPTIMIZATION: Cache AFC object lookup with validation
        if self.afc is not None:
            # Validate cached object is still alive
            try:
                _ = self.afc.lanes  # Quick attribute access test
                return self.afc
            except Exception:
                self.logger.warning("Cached AFC object invalid, re-fetching")

                self.afc = None

        cached_afc = self._hardware_service_cache.get("afc_object")

        if cached_afc is not None:
            # Validate hardware service cache
            try:
                _ = cached_afc.lanes
                self.afc = cached_afc
                return self.afc
            except Exception:
                self.logger.warning("Cached AFC object in hardware service invalid, re-fetching")

                self._hardware_service_cache.pop("afc_object", None)

        try:
            afc = self.printer.lookup_object('AFC')
        except Exception:
            self.afc = None
            return None

        self.afc = afc
        self._hardware_service_cache["afc_object"] = afc
        self._ensure_afc_lane_cache(afc)
        if not self._afc_logged:
            self.logger.info("AFC integration detected; enabling same-FPS infinite runout support.")

            self._afc_logged = True
        return self.afc

    def _get_reload_params(self, lane_name: str) -> Tuple[Optional[float], Optional[float]]:
        """Get reload length and speed from AFC extruder config.

        Returns:
            (reload_length, reload_speed) tuple, or (None, None) if not available
        """
        try:
            afc = self._get_afc()
            if afc is None:
                return None, None

            lane = afc.lanes.get(lane_name)
            if lane is None:
                return None, None

            # Get extruder name from lane
            extruder_name = getattr(lane, 'extruder_name', None)
            if not extruder_name:
                return None, None

            # Look up AFC_extruder object
            afc_extruder_name = f'AFC_extruder {extruder_name}'
            afc_extruder = self.printer.lookup_object(afc_extruder_name, None)
            if afc_extruder is None:
                return None, None

            # Get reload parameters from AFC_extruder
            # RELOAD_LENGTH = (tool_stn / 2) + tool_sensor_after_extruder + retract_length + hotend_meltzone_compensation
            tool_stn = getattr(afc_extruder, 'tool_stn', 0.0)
            tool_sensor_after = getattr(afc_extruder, 'tool_sensor_after_extruder', 0.0)
            tool_load_speed = getattr(afc_extruder, 'tool_load_speed', 25.0)

            # Get additional components from macro variables
            try:
                macro_vars = self.printer.lookup_object('gcode_macro _oams_macro_variables', None)
                hotend_compensation = getattr(macro_vars, 'hotend_meltzone_compensation', 0.0) if macro_vars else 0.0

                cut_tip_vars = self.printer.lookup_object('gcode_macro _AFC_CUT_TIP_VARS', None)
                retract_length = getattr(cut_tip_vars, 'retract_length', 0.0) if cut_tip_vars else 0.0
            except Exception:
                hotend_compensation = 0.0
                retract_length = 0.0

            # Calculate post-engagement reload length (remaining move after engagement check)
            reload_length = (tool_stn / 2.0) + tool_sensor_after + retract_length + hotend_compensation
            reload_speed = tool_load_speed * 60.0  # Convert to mm/min

            return reload_length, reload_speed

        except Exception:
            self.logger.error(f"Failed to get reload params for {lane_name}")
            return None, None

    def _get_engagement_params(self, lane_name: str) -> Tuple[Optional[float], Optional[float]]:
        """Get engagement extrusion length and speed from AFC extruder config."""
        try:
            afc = self._get_afc()
            if afc is None:
                return None, None

            lane = afc.lanes.get(lane_name)
            if lane is None:
                return None, None

            # Get extruder name from lane
            extruder_name = getattr(lane, 'extruder_name', None)
            if not extruder_name:
                return None, None

            # Look up AFC_extruder object
            afc_extruder_name = f'AFC_extruder {extruder_name}'
            afc_extruder = self.printer.lookup_object(afc_extruder_name, None)
            if afc_extruder is None:
                return None, None

            tool_stn = getattr(afc_extruder, 'tool_stn', None)
            engagement_length = (tool_stn / 2.0) if tool_stn is not None else None
            engagement_speed = getattr(afc_extruder, 'tool_load_speed', None)
            engagement_speed = engagement_speed * 60.0 if engagement_speed is not None else None

            return engagement_length, engagement_speed
        except Exception:
            self.logger.error(f"Failed to get engagement params for {lane_name}")
            return None, None

    def _get_unload_params(self, lane_name: str) -> Tuple[Optional[float], Optional[float]]:
        """Get unload retract length and speed from AFC extruder config."""
        try:
            afc = self._get_afc()
            if afc is None:
                return None, None

            lane = afc.lanes.get(lane_name)
            if lane is None:
                return None, None

            extruder_name = getattr(lane, 'extruder_name', None)
            if not extruder_name:
                return None, None

            afc_extruder_name = f'AFC_extruder {extruder_name}'
            afc_extruder = self.printer.lookup_object(afc_extruder_name, None)
            if afc_extruder is None:
                return None, None

            unload_length = getattr(afc_extruder, 'tool_stn_unload', None)
            if unload_length is None or unload_length <= 0:
                unload_length = getattr(afc_extruder, 'tool_stn', None)

            unload_speed = getattr(afc_extruder, 'tool_unload_speed', None)
            unload_speed = unload_speed * 60.0 if unload_speed is not None else None

            return unload_length, unload_speed
        except Exception:
            self.logger.error(f"Failed to get unload params for {lane_name}")
            return None, None

    def _verify_engagement_with_extrude(self, fps_name: str, fps_state: 'FPSState', fps,
                                      lane_name: str, oams) -> bool:
        """Verify filament engaged extruder by extruding reload length and monitoring FPS pressure.

        After OAMS pushes filament to extruder, extrude the configured reload length
        and monitor FPS pressure. If pressure drops below threshold, filament
        engaged successfully. If pressure stays high, filament didn't engage.

        Args:
            fps_name: FPS name (e.g., "fps1")

            fps_state: FPS state object
            fps: FPS object
            lane_name: Lane name (e.g., "lane1")

            oams: OAMS object

        Returns:
            True if filament engaged (pressure dropped), False otherwise
        """
        try:
            # Get engagement parameters from AFC config
            engagement_length, engagement_speed = self._get_engagement_params(lane_name)
            if engagement_length is None or engagement_speed is None:
                self.logger.error(f"Failed to get engagement params for {lane_name}, cannot verify engagement")
                return True  # Assume success to avoid false failures
            post_length, post_speed = self._get_reload_params(lane_name)
            post_length_display = f"{post_length:.2f}" if post_length is not None else "None"
            post_speed_display = f"{post_speed:.0f}" if post_speed is not None else "None"
            self.logger.debug(
                f"Load params for {lane_name}: "
                f"engagement_length={engagement_length:.2f}mm "
                f"engagement_speed={engagement_speed:.0f}mm/min "
                f"reload_length={post_length_display}mm reload_speed={post_speed_display}mm/min"
            )

            self.logger.debug(
                f"Verifying filament engagement for {lane_name}: "
                f"extruding {engagement_length:.1f}mm at {engagement_speed:.0f}mm/min"
            )

            # Set flag to suppress stuck spool detection during engagement verification
            # High FPS pressure during engagement extrusion is NORMAL and expected
            fps_state.engagement_in_progress = True
            try:
                if fps_state.current_oams is not None and fps_state.current_spool_idx is not None:
                    oams_obj = self._get_oams_object(fps_state.current_oams)
                    if oams_obj is not None:
                        self._enable_follower(
                            fps_name,
                            fps_state,
                            oams_obj,
                            1,
                            "engagement verification extrusion",
                        )

                # Get extruder object
                extruder = getattr(fps, 'extruder', None)
                if extruder is None:
                    self.logger.error(f"No extruder found for {fps_name}")
                    return True  # Assume success

                # Record encoder position BEFORE engagement extrusion
                # Encoder movement during extrusion proves filament engaged successfully
                try:
                    encoder_before = oams.encoder_clicks
                except Exception:
                    self.logger.warning(f"Could not read encoder before engagement for {lane_name}")
                    encoder_before = None

                # Get gcode object for running extrusion command
                gcode = self.printer.lookup_object('gcode')
                gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_engagement")
                try:
                    # Run the engagement extrusion using gcode command
                    # M83: relative extrusion, G92 E0: reset position, G1: extrude
                    prime_length = 5.0
                    remaining_length = max(0.0, engagement_length - prime_length)
                    gcode.run_script_from_command("M83")  # Relative extrusion mode
                    gcode.run_script_from_command("G92 E0")  # Reset extruder position
                    gcode.run_script_from_command(f"G1 E{prime_length:.2f} F{engagement_speed:.0f}")  # Prime to help engagement
                    gcode.run_script_from_command("M400")  # Wait for moves to complete
                    self.reactor.pause(self.reactor.monotonic() + 0.2)
                    if remaining_length > 0.0:
                        gcode.run_script_from_command(f"G1 E{remaining_length:.2f} F{engagement_speed:.0f}")  # Extrude to nozzle tip
                        gcode.run_script_from_command("M400")  # Wait for moves to complete
                finally:
                    gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_engagement")

                # Small pause to let encoder reading settle
                self.reactor.pause(self.reactor.monotonic() + 0.2)

                # Check encoder movement - most reliable indicator of successful engagement
                # If encoder moved, follower tracked filament being pulled through buffer ? engaged!
                try:
                    encoder_after = oams.encoder_clicks

                    # Record engagement result and timestamp for stuck spool suppression
                    now = self.reactor.monotonic()
                    fps_state.engagement_checked_at = now

                    if encoder_before is not None:
                        encoder_delta = abs(encoder_after - encoder_before)
                        # Expect encoder movement for at least 50% of the post-prime extrusion,
                        # then allow a small fixed slack to reduce false failures.
                        # engagement_length is tool_stn / 2, so this checks for tool_stn / 4,
                        # minus the prime length. Allow 3 clicks of slack.
                        expected_movement = max(1.0, remaining_length * 0.5)
                        min_encoder_movement = max(1.0, expected_movement - 3.0)

                        if encoder_delta >= min_encoder_movement:
                            fps_pressure = oams.fps_value
                            if fps_pressure >= self.engagement_pressure_threshold:
                                fps_state.engaged_with_extruder = False
                                self.logger.warning(
                                    f"Filament failed to engage for {lane_name} "
                                    f"(encoder moved {encoder_delta} clicks but FPS pressure stayed high at "
                                    f"{fps_pressure:.2f})"
                                )
                                return False

                            # Encoder moved and pressure dropped - filament engaged successfully.
                            fps_state.engaged_with_extruder = True
                            self.logger.debug(
                                f"Filament engagement verified for {lane_name} "
                                f"(encoder moved {encoder_delta} clicks during {engagement_length:.1f}mm extrusion)"
                            )
                            if post_length is not None and post_speed is not None and post_length > 0:
                                self.logger.debug(
                                    f"Completing load for {lane_name}: extruding {post_length:.1f}mm "
                                    f"at {post_speed:.0f}mm/min"
                                )
                                gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_post_load")
                                gcode.run_script_from_command("M83")  # Relative extrusion mode
                                gcode.run_script_from_command(f"G1 E{post_length:.2f} F{post_speed:.0f}")
                                gcode.run_script_from_command("M400")
                                gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_post_load")
                            return True
                        else:
                            # Encoder didn't move enough - re-check after a short delay
                            self.reactor.pause(self.reactor.monotonic() + 0.3)
                            try:
                                encoder_retry = oams.encoder_clicks
                            except Exception:
                                encoder_retry = encoder_after
                            encoder_delta = abs(encoder_retry - encoder_before)
                            if encoder_delta >= min_encoder_movement:
                                fps_pressure = oams.fps_value
                                if fps_pressure >= self.engagement_pressure_threshold:
                                    fps_state.engaged_with_extruder = False
                                    self.logger.warning(
                                        f"Filament failed to engage for {lane_name} "
                                        f"(encoder moved {encoder_delta} clicks after retry but FPS pressure stayed high at "
                                        f"{fps_pressure:.2f})"
                                    )
                                    return False
                                fps_state.engaged_with_extruder = True
                                self.logger.debug(
                                    f"Filament engagement verified for {lane_name} "
                                    f"(encoder moved {encoder_delta} clicks after retry during "
                                    f"{engagement_length:.1f}mm extrusion)"
                                )
                                if post_length is not None and post_speed is not None and post_length > 0:
                                    self.logger.debug(
                                        f"Completing load for {lane_name}: extruding {post_length:.1f}mm "
                                        f"at {post_speed:.0f}mm/min"
                                    )
                                    gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_post_load")
                                    gcode.run_script_from_command("M83")  # Relative extrusion mode
                                    gcode.run_script_from_command(f"G1 E{post_length:.2f} F{post_speed:.0f}")
                                    gcode.run_script_from_command("M400")
                                    gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_post_load")
                                return True

                            # Encoder didn't move enough - filament not engaged
                            fps_state.engaged_with_extruder = False
                            self.logger.info(
                                f"Filament failed to engage extruder for {lane_name} "
                                f"(encoder only moved {encoder_delta} clicks, expected >={min_encoder_movement:.1f})"
                            )
                            return False
                    else:
                        # Couldn't read encoder before - fall back to pressure check
                        fps_pressure = oams.fps_value
                        if fps_pressure < self.engagement_pressure_threshold:
                            fps_state.engaged_with_extruder = True
                            self.logger.debug(
                                f"Filament engagement verified for {lane_name} "
                                f"(FPS pressure {fps_pressure:.2f}, encoder unavailable)"
                            )
                            if post_length is not None and post_speed is not None and post_length > 0:
                                self.logger.debug(
                                    f"Completing load for {lane_name}: extruding {post_length:.1f}mm "
                                    f"at {post_speed:.0f}mm/min"
                                )
                                gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_post_load")
                                gcode.run_script_from_command("M83")  # Relative extrusion mode
                                gcode.run_script_from_command(f"G1 E{post_length:.2f} F{post_speed:.0f}")
                                gcode.run_script_from_command("M400")
                                gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_post_load")
                            return True
                        else:
                            fps_state.engaged_with_extruder = False
                            self.logger.warning(
                                f"Filament failed to engage for {lane_name} "
                                f"(FPS pressure {fps_pressure:.2f}, encoder unavailable)"
                            )
                            return False

                except Exception:
                    self.logger.error(f"Failed to check encoder for engagement verification on {fps_name}")
                    return False
            finally:
                fps_state.engagement_in_progress = False  # Clear flag now that verification is complete

        except Exception:
            self.logger.error(
                f"Failed to verify engagement for {lane_name}\n{traceback.format_exc()}"
            )
            return False

    def _handle_successful_reload(self, fps_name: str, fps_state: 'FPSState', target_lane: str,
                                   target_lane_map: str, source_lane_name: Optional[str],
                                   active_oams: str, monitor) -> None:
        """Handle successful reload after same-FPS runout swap completes.

        This function:
        1. Logs success
        2. Notifies AFC that target lane is loaded (updates virtual sensors)
        3. Ensures follower is enabled for new lane
        4. Clears source lane state in AFC
        5. Resets runout monitor
        """
        self.logger.info(f"Successfully loaded lane {target_lane} on {fps_name} after infinite runout")

        # Always notify AFC that target lane is loaded to update virtual sensors
        # This ensures AMS_Extruder# sensors show correct state after same-FPS runouts
        if target_lane:
            handled = False
            if AMSRunoutCoordinator is not None:
                try:
                    handled = AMSRunoutCoordinator.notify_lane_tool_state(
                        self.printer, fps_state.current_oams or active_oams, target_lane,
                        loaded=True, spool_index=fps_state.current_spool_idx, eventtime=fps_state.since
                    )
                    if handled:
                        self.logger.info(f"Notified AFC that lane {target_lane} is loaded via AMSRunoutCoordinator (updates virtual sensor state)")
                    else:
                        self.logger.info(f"AMSRunoutCoordinator.notify_lane_tool_state returned False for lane {target_lane}, trying fallback")
                except Exception as e:
                    self.logger.error(f"Failed to notify AFC lane {target_lane} after infinite runout on {fps_name}: {e}")
                    handled = False
            else:
                # AMSRunoutCoordinator not available - call AFC methods directly
                self.logger.info(f"AMSRunoutCoordinator not available, updating virtual sensor directly for lane {target_lane}")
                try:
                    afc = self._get_afc()
                    if afc and hasattr(afc, 'lanes'):
                        lane_obj = afc.lanes.get(target_lane)
                        if lane_obj:
                            # Update virtual sensor using AFC's direct method
                            if hasattr(afc, '_mirror_lane_to_virtual_sensor'):
                                afc._mirror_lane_to_virtual_sensor(lane_obj, self.reactor.monotonic())
                                self.logger.info(f"Updated virtual sensor for lane {target_lane} via AFC._mirror_lane_to_virtual_sensor")
                                handled = True
                            else:
                                self.logger.warning("AFC doesn't have _mirror_lane_to_virtual_sensor method")

                        else:
                            self.logger.warning(f"Lane object not found for {target_lane} in AFC")
                    else:
                        self.logger.warning("AFC not available or has no lanes attribute")

                except Exception as e:
                    self.logger.error(f"Failed to update virtual sensor directly for lane {target_lane}: {e}")

            if not handled:
                try:
                    gcode = self.printer.lookup_object("gcode")

                    gcode.run_script(f"SET_LANE_LOADED LANE={target_lane}")

                    self.logger.info(f"Marked lane {target_lane} as loaded via SET_LANE_LOADED after infinite runout on {fps_name}")
                except Exception as e:
                    self.logger.error(f"Failed to mark lane {target_lane} as loaded after infinite runout on {fps_name}: {e}")

        # Ensure follower is enabled after successful reload
        # Follower should stay enabled throughout same-FPS runouts (never disabled)
        # This is a safety check to ensure follower is active for new lane
        if fps_state.current_oams and fps_state.current_spool_idx is not None:
            # Use _get_oams_object for proper name fallback (handles "oams1" vs "oams oams1" mismatch)
            oams = self._get_oams_object(fps_state.current_oams)
            if oams:
                self._ensure_forward_follower(fps_name, fps_state, "after infinite runout reload")

                # CRITICAL: Force update target lane's loaded_to_hub to match current hub sensor state
                # During same-FPS runout, hub sensor may stay True throughout (Lane A unload ? Lane B load)
                # The hub_changed event won't fire because hardware value didn't change
                # So new lane's loaded_to_hub won't get updated, causing Mainsail to show stale status
                try:
                    hub_hes_values = getattr(oams, "hub_hes_value", None)
                    if hub_hes_values is not None and fps_state.current_spool_idx < len(hub_hes_values):
                        hub_sensor_state = bool(hub_hes_values[fps_state.current_spool_idx])
                        afc = self._get_afc()
                        if afc and hasattr(afc, 'lanes') and target_lane:
                            target_lane_obj = afc.lanes.get(target_lane)
                            if target_lane_obj:
                                # Force update loaded_to_hub to match actual hub sensor
                                target_lane_obj.loaded_to_hub = hub_sensor_state
                                self.logger.info(f"Force updated loaded_to_hub={hub_sensor_state} for {target_lane} after same-FPS runout")
                except Exception as e:
                    self.logger.warning(f"Failed to force update loaded_to_hub for {target_lane}: {e}")

        # Clear the source lane's state in AFC so it shows as EMPTY and can detect new filament
        # Note: LED will clear automatically when lane state updates to empty
        # FPS state stays LOADED with the new target lane, but old lane needs to be cleared
        if source_lane_name:
            try:
                if AMSRunoutCoordinator is not None:
                    # Notify AFC that source lane is now unloaded
                    AMSRunoutCoordinator.notify_lane_tool_state(
                        self.printer,
                        fps_state.current_oams or active_oams,
                        source_lane_name,
                        loaded=False,
                        spool_index=None,
                        eventtime=self.reactor.monotonic()
                    )
                    self.logger.info(f"Cleared source lane {source_lane_name} state in AFC after successful reload to {target_lane}")
                else:
                    # Fallback to gcode command if coordinator not available
                    gcode = self.printer.lookup_object("gcode")

                    gcode.run_script(f"SET_LANE_UNLOADED LANE={source_lane_name}")

                    self.logger.info(f"Cleared source lane {source_lane_name} via SET_LANE_UNLOADED after reload to {target_lane}")

                # Clear the same-FPS runout flag on source lane after successful reload
                afc = self._get_afc()
                if afc and hasattr(afc, 'lanes'):
                    source_lane_obj = afc.lanes.get(source_lane_name)
                    if source_lane_obj and hasattr(source_lane_obj, '_oams_same_fps_runout'):
                        source_lane_obj._oams_same_fps_runout = False
                        self.logger.info(f"Cleared same-FPS runout flag on lane {source_lane_name} after successful reload")
            except Exception:
                self.logger.error(f"Failed to clear source lane {source_lane_name} state after reload to {target_lane}")

        # CRITICAL: Reset detection trackers after successful reload to prevent false positives
        # The clog/stuck spool trackers may have stale encoder data from the old lane
        # which would immediately trigger false clog detection on the new lane
        fps_state.reset_clog_tracker()
        fps_state.reset_stuck_spool_state()
        fps_state.clear_encoder_samples()
        # Record lane change time for clog/stuck detection grace period
        fps_state.last_lane_change_time = self.reactor.monotonic()
        self.logger.debug(f"Reset clog/stuck spool trackers after successful reload on {fps_name}")

        fps_state.reset_runout_positions()
        if monitor:
            monitor.reset()
            monitor.start()

    def _start_async_reload(self, fps_name: str, fps_state: 'FPSState', target_lane: str,
                           target_lane_map: str, source_lane_name: Optional[str],
                           active_oams: str, monitor) -> None:
        """Start non-blocking reload using OAMS hardware with async completion polling.

        This avoids blocking wait loops that don't work from timer context.
        Instead, we start the operation and poll for completion with a timer.
        """
        # Get OAMS object for unload
        if fps_name not in self.fpss:
            self.logger.error(f"FPS {fps_name} not found for async reload")
            self._pause_printer_message(f"FPS {fps_name} not found", active_oams)
            if monitor:
                monitor.paused()
            return

        fps_state_obj = self.current_state.fps_state[fps_name]
        if fps_state_obj.current_oams is None:
            self.logger.error(f"No OAMS loaded on {fps_name} for async reload")
            self._pause_printer_message(f"No OAMS on {fps_name}", active_oams)
            if monitor:
                monitor.paused()
            return

        oams_unload = self._get_oams_object(fps_state_obj.current_oams)
        if oams_unload is None:
            self.logger.error(f"OAMS {fps_state_obj.current_oams} not found for unload")
            self._pause_printer_message(f"OAMS {fps_state_obj.current_oams} not found", active_oams)
            if monitor:
                monitor.paused()
            return

        # Start unload operation (non-blocking - just sends MCU command)
        self.logger.info(f"Starting async unload for same-FPS reload on {fps_name}")

        # CRITICAL: Enable follower in reverse direction BEFORE starting unload
        # The follower motor must be set to pull filament back to the spool
        # before the BLDC motor starts rewinding (following user's requirement:
        # "we have to first tell the follower what direction to be going in")
        try:
            self._set_follower_state(
                fps_name,
                fps_state_obj,
                oams_unload,
                1,
                0,
                "async unload",
                force=True,
            )
            self.logger.debug(f"Set follower reverse before async unload on {fps_name}")
        except Exception:
            self.logger.warning(f"Failed to set follower reverse before async unload on {fps_name}")

        if OAMSStatus is None:
            self.logger.error("CRITICAL: OAMSStatus not available (oams.py import failed)")

            self._pause_printer_message("OAMS module error", active_oams)
            if monitor:
                monitor.paused()
            return

        oams_unload.action_status = OAMSStatus.UNLOADING
        oams_unload.oams_unload_spool_cmd.send()
        unload_start_time = self.reactor.monotonic()

        # Create polling timer to check unload completion
        def _check_unload_complete(eventtime):
            # Check if unload completed
            if oams_unload.action_status is None:
                # Unload finished - check result
                if OAMSOpCode is None:
                    self.logger.error("CRITICAL: OAMSOpCode not available (oams.py import failed)")

                    self._pause_printer_message("OAMS module error", active_oams)
                    if monitor:
                        monitor.paused()
                    return self.reactor.NEVER

                if oams_unload.action_status_code == OAMSOpCode.SUCCESS:
                    self.logger.info(f"Async unload completed successfully, starting load for {target_lane}")
                    # Update FPS state
                    fps_state_obj.state = FPSLoadState.UNLOADED
                    fps_state_obj.following = False
                    fps_state_obj.direction = 0
                    fps_state_obj.since = self.reactor.monotonic()
                    oams_unload.current_spool = None

                    # Now start load operation
                    self._start_async_load(fps_name, fps_state, target_lane, target_lane_map,
                                         source_lane_name, active_oams, monitor)
                else:
                    self.logger.error(f"Async unload failed with code {oams_unload.action_status_code}")
                    self._pause_printer_message(f"Failed to unload on {fps_name}", active_oams)
                    if monitor:
                        monitor.paused()
                return self.reactor.NEVER  # Stop polling

            # Check timeout (30 seconds)
            if eventtime - unload_start_time > 30.0:
                self.logger.error("Async unload timed out after 30s")

                oams_unload.action_status = None
                self._pause_printer_message(f"Unload timeout on {fps_name}", active_oams)
                if monitor:
                    monitor.paused()
                return self.reactor.NEVER  # Stop polling

            # Not done yet - keep polling every 100ms
            return eventtime + 0.1

        # Start polling timer
        self.reactor.register_timer(_check_unload_complete, self.reactor.NOW)

    def _start_async_load(self, fps_name: str, fps_state: 'FPSState', target_lane: str,
                         target_lane_map: str, source_lane_name: Optional[str],
                         active_oams: str, monitor) -> None:
        """Start non-blocking load operation with async completion polling."""
        # Get target lane info to determine OAMS and bay
        afc = self._get_afc()
        if afc is None:
            self.logger.error("AFC not available for async load")

            self._pause_printer_message("AFC not available", active_oams)
            if monitor:
                monitor.paused()
            return

        lane = afc.lanes.get(target_lane)
        if lane is None:
            self.logger.error(f"Lane {target_lane} not found for async load")
            self._pause_printer_message(f"Lane {target_lane} not found", active_oams)
            if monitor:
                monitor.paused()
            return

        # Get unit and slot to find OAMS and bay index
        unit_str = getattr(lane, "unit", None)
        if not unit_str:
            self.logger.error(f"Lane {target_lane} has no unit")
            self._pause_printer_message(f"Lane {target_lane} has no unit", active_oams)
            if monitor:
                monitor.paused()
            return

        # Parse slot number
        slot_number = None
        if isinstance(unit_str, str) and ':' in unit_str:
            base_unit_name, slot_str = unit_str.split(':', 1)
            try:
                slot_number = int(slot_str)
            except ValueError:
                self.logger.error(f"Invalid slot in unit {unit_str}")
                self._pause_printer_message(f"Invalid slot in {unit_str}", active_oams)
                if monitor:
                    monitor.paused()
                return
        else:
            base_unit_name = str(unit_str)
            slot_number = getattr(lane, "index", None)

        if slot_number is None:
            self.logger.error(f"Could not determine slot for lane {target_lane}")
            self._pause_printer_message(f"No slot for {target_lane}", active_oams)
            if monitor:
                monitor.paused()
            return

        bay_index = slot_number - 1
        if bay_index < 0:
            self.logger.error(f"Invalid slot {slot_number}")
            self._pause_printer_message(f"Invalid slot {slot_number}", active_oams)
            if monitor:
                monitor.paused()
            return

        # Get OAMS object
        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None:
            units = getattr(afc, "units", {})
            unit_obj = units.get(base_unit_name)

        if unit_obj is None:
            self.logger.error(f"Unit {base_unit_name} not found")
            self._pause_printer_message(f"Unit {base_unit_name} not found", active_oams)
            if monitor:
                monitor.paused()
            return

        oams_name = getattr(unit_obj, "oams_name", None)
        if not oams_name:
            self.logger.error(f"Unit {base_unit_name} has no oams_name")
            self._pause_printer_message(f"Unit {base_unit_name} has no OAMS", active_oams)
            if monitor:
                monitor.paused()
            return

        oam_load = self.oams.get(oams_name)
        if oam_load is None:
            oam_load = self.oams.get(f"oams {oams_name}")

        if oam_load is None:
            self.logger.error(f"OAMS {oams_name} not found for load")
            self._pause_printer_message(f"OAMS {oams_name} not found", active_oams)
            if monitor:
                monitor.paused()
            return

        # Enable follower before load
        fps_state_obj = self.current_state.fps_state[fps_name]
        self._enable_follower(fps_name, fps_state_obj, oam_load, 1, "before async load")


        # Update FPS state for loading
        fps_state_obj.state = FPSLoadState.LOADING
        fps_state_obj.encoder = oam_load.encoder_clicks
        fps_state_obj.current_oams = oam_load.name
        fps_state_obj.current_spool_idx = bay_index
        fps_state_obj.since = self.reactor.monotonic()
        fps_state_obj.clear_encoder_samples()

        # Note: Stuck spool retry logic uses nested loops now (2 attempts per engagement try)
        # No need to track persistent counter - each engagement attempt gets fresh stuck retries

        # Start load operation (non-blocking - just sends MCU command)
        self.logger.info(f"Starting async load for lane {target_lane} (bay {bay_index}) on {oams_name}")
        if OAMSStatus is None:
            self.logger.error("CRITICAL: OAMSStatus not available (oams.py import failed)")

            self._pause_printer_message("OAMS module error", active_oams)
            if monitor:
                monitor.paused()
            return

        oam_load.action_status = OAMSStatus.LOADING
        oam_load.oams_load_spool_cmd.send([bay_index])
        load_start_time = self.reactor.monotonic()

        # Create polling timer to check load completion
        def _check_load_complete(eventtime):
            # Check if load completed
            if oam_load.action_status is None:
                # Load finished - check result
                if OAMSOpCode is None:
                    self.logger.error("CRITICAL: OAMSOpCode not available (oams.py import failed)")

                    self._pause_printer_message("OAMS module error", active_oams)
                    if monitor:
                        monitor.paused()
                    return self.reactor.NEVER

                if oam_load.action_status_code == OAMSOpCode.SUCCESS:
                    self.logger.info(f"Async load completed successfully for lane {target_lane}")
                    # Update state
                    fps_state_obj.state = FPSLoadState.LOADED
                    fps_state_obj.current_lane = target_lane
                    fps_state_obj.since = self.reactor.monotonic()
                    oam_load.current_spool = bay_index

                    # Call success handler with AFC notifications
                    self._handle_successful_reload(fps_name, fps_state, target_lane, target_lane_map,
                                                   source_lane_name, active_oams, monitor)
                else:
                    self.logger.error(f"Async load failed with code {oam_load.action_status_code}")
                    fps_state_obj.state = FPSLoadState.UNLOADED
                    self._pause_printer_message(f"Failed to load {target_lane}", active_oams)
                    if monitor:
                        monitor.paused()
                return self.reactor.NEVER  # Stop polling

            # Check timeout (30 seconds)
            if eventtime - load_start_time > 30.0:
                self.logger.error("Async load timed out after 30s")

                oam_load.action_status = None
                fps_state_obj.state = FPSLoadState.UNLOADED
                self._pause_printer_message(f"Load timeout for {target_lane}", active_oams)
                if monitor:
                    monitor.paused()
                return self.reactor.NEVER  # Stop polling

            # Not done yet - keep polling every 100ms
            return eventtime + 0.1

        # Start polling timer
        self.reactor.register_timer(_check_load_complete, self.reactor.NOW)

    def _get_infinite_runout_target_lane(self, fps_name: str, fps_state: 'FPSState') -> Tuple[Optional[str], Optional[str], bool, Optional[str]]:
        """Get target lane for infinite runout.

        Returns: (target_lane_map, target_lane_name, delegate_to_afc, source_lane_name)
        - target_lane_map: Lane map attribute or lane name (for display)
        - target_lane_name: Actual target lane name to load
        - delegate_to_afc: True if AFC should handle (always True to preserve coasting/timing logic)
        - source_lane_name: Current source lane name
        """
        current_lane = fps_state.current_lane
        if not current_lane:
            return None, None, False, None

        afc = self._get_afc()
        if afc is None:
            return None, None, False, None

        lane_name, _ = self._resolve_lane_for_state(fps_state, current_lane, afc)

        if not lane_name:
            return None, None, False, None

        lanes = getattr(afc, "lanes", {})
        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None, None, False, lane_name

        raw_runout_lane = getattr(lane, "runout_lane", None)
        runout_lane_name = self._resolve_afc_lane_name(afc, raw_runout_lane)
        if not runout_lane_name:
            return None, None, False, lane_name

        if raw_runout_lane and runout_lane_name != raw_runout_lane:
            try:
                lane.runout_lane = runout_lane_name
                self.logger.debug(f"Normalized runout lane for {lane_name} from {raw_runout_lane} to {runout_lane_name}")
            except Exception:
                self.logger.debug(f"Could not persist normalized runout lane {runout_lane_name} on {lane_name}")

        target_lane = afc.lanes.get(runout_lane_name)
        if target_lane is None:
            self.logger.warning(f"Runout lane {runout_lane_name} for {lane_name} on {fps_name} is not available; deferring to AFC")
            return None, runout_lane_name, True, lane_name

        # Check if source and target lanes are on the same FPS/extruder
        # If SAME FPS: OpenAMS handles reload internally (coast, PTFE calc, load new spool)
        # If DIFFERENT FPS: AFC handles via CHANGE_TOOL (full runout, then switch tools)
        source_extruder = self._get_lane_extruder_name(lane)
        target_extruder = self._get_lane_extruder_name(target_lane)

        same_fps = bool(source_extruder and target_extruder and source_extruder == target_extruder)
        delegate_to_afc = not same_fps  # Only delegate if different FPS

        target_lane_map = getattr(target_lane, "map", runout_lane_name)

        if same_fps:
            self.logger.info(f"Infinite runout: {lane_name} -> {runout_lane_name} on same FPS {fps_name} (OpenAMS handling internally)")

        else:
            self.logger.info(f"Infinite runout: {lane_name} -> {runout_lane_name} on different FPS (delegating to AFC for tool change)")


        return target_lane_map, runout_lane_name, delegate_to_afc, lane_name

    def _delegate_runout_to_afc(self, fps_name: str, fps_state: 'FPSState', source_lane_name: Optional[str], target_lane_name: Optional[str]) -> bool:
        afc = self._get_afc()
        if afc is None:
            return False

        if not source_lane_name:
            return False

        lane = afc.lanes.get(source_lane_name)
        if lane is None:
            self.logger.warning(f"AFC lane {source_lane_name} not found while delegating infinite runout for {fps_name}")
            return False

        raw_runout_target = getattr(lane, "runout_lane", None)
        resolved_request = target_lane_name or raw_runout_target
        runout_target = self._resolve_afc_lane_name(afc, raw_runout_target) or self._resolve_afc_lane_name(afc, target_lane_name)
        if not runout_target:
            self.logger.warning(
                "AFC lane %s has no runout target while delegating infinite runout for %s (raw=%s, requested=%s)",
                source_lane_name,
                fps_name,
                raw_runout_target,
                resolved_request,
            )
            return False

        if raw_runout_target and runout_target != raw_runout_target:
            try:
                lane.runout_lane = runout_target
                self.logger.info(f"Delegating runout for {source_lane_name} using normalized target {runout_target} (from {raw_runout_target})")
            except Exception:
                self.logger.debug(f"Failed to persist normalized runout target {runout_target} on {source_lane_name}")
        elif not raw_runout_target:
            try:
                lane.runout_lane = runout_target
                self.logger.debug(f"Delegating runout for {source_lane_name} using resolved target {runout_target} (no raw target set)")
            except Exception:
                self.logger.debug(f"Failed to persist resolved runout target {runout_target} on {source_lane_name}")

        if getattr(lane, "runout_lane", None) != runout_target:
            self.logger.warning(
                "Resolved runout target %s could not be stored on lane %s (current=%s)",
                runout_target,
                source_lane_name,
                getattr(lane, "runout_lane", None),
            )
            return False

        now = self.reactor.monotonic()
        if fps_state.afc_delegation_active and now < fps_state.afc_delegation_until:
            return True

        if runout_target not in afc.lanes:
            self.logger.warning(
                "AFC runout lane %s referenced by %s is unavailable (requested=%s)",
                runout_target,
                source_lane_name,
                resolved_request,
            )
            return False

        try:
            current_lane = getattr(afc, "current", None)
            if current_lane != source_lane_name:
                if current_lane not in afc.lanes:
                    self.logger.debug(
                        "AFC current lane %s invalid during runout for %s; overriding to %s",
                        current_lane,
                        source_lane_name,
                        source_lane_name,
                    )
                try:
                    afc.current = source_lane_name
                except Exception:
                    self.logger.debug(f"Failed to set AFC current lane to {source_lane_name} before delegation")

            self.logger.debug(
                "Delegating infinite runout via AFC: fps=%s source=%s target=%s (resolved_request=%s)",
                fps_name,
                source_lane_name,
                runout_target,
                resolved_request,
            )
            lane._perform_infinite_runout()
        except Exception:
            err_trace = traceback.format_exc()
            self.logger.error(
                "AFC infinite runout failed for lane %s -> %s: %s",
                source_lane_name,
                runout_target,
                err_trace,
            )
            fps_state.afc_delegation_active = False
            fps_state.afc_delegation_until = 0.0
            return False

        fps_state.afc_delegation_active = True
        fps_state.afc_delegation_until = now + AFC_DELEGATION_TIMEOUT
        self.logger.info(f"Delegated infinite runout for {fps_name} via AFC lane {source_lane_name} -> {runout_target}")
        return True

    def _unload_filament_for_fps(self, fps_name: str) -> Tuple[bool, str]:
        if fps_name not in self.fpss:
            return False, f"FPS {fps_name} does not exist"

        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.UNLOADED:
            return False, f"FPS {fps_name} is already unloaded"
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            return False, f"FPS {fps_name} is busy ({fps_state.state.name}), cannot unload"
        if fps_state.state != FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is in unexpected state {fps_state.state.name}"

        if fps_state.current_oams is None:
            oams_from_lane = self._resolve_oams_for_lane(fps_state.current_lane)
            if oams_from_lane is not None:
                fps_state.current_oams = oams_from_lane.name
            else:
                return False, f"FPS {fps_name} has no OAMS loaded"

        oams = self._get_oams_object(fps_state.current_oams)
        if oams is None:
            oams = self._resolve_oams_for_lane(fps_state.current_lane)
            if oams is not None:
                fps_state.current_oams = oams.name
            else:
                return False, f"OAMS {fps_state.current_oams} not found for FPS {fps_name}"

        if oams.current_spool is None:
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.current_lane = None
            fps_state.current_spool_idx = None
            fps_state.since = self.reactor.monotonic()
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, "Spool already unloaded"

        lane_name: Optional[str] = None
        spool_index = fps_state.current_spool_idx
        if AMSRunoutCoordinator is not None:
            try:
                afc = self._get_afc()
                if afc is not None:
                    lane_name, _ = self._resolve_lane_for_state(fps_state, fps_state.current_lane, afc)
            except Exception:
                self.logger.error(f"Failed to resolve AFC lane for unload on {fps_name}")
                lane_name = None

        # Clear any lingering stuck flags so retries can proceed without manual resets
        if fps_state.stuck_spool.active:
            fps_state.reset_stuck_spool_state(preserve_restore=True)
            self.logger.info(f"Cleared stuck-spool flag before retrying unload on {fps_name}")

        # Capture state BEFORE changing fps_state.state to avoid getting stuck
        try:
            encoder = oams.encoder_clicks
            current_time = self.reactor.monotonic()
            current_oams_name = oams.name
            current_spool = oams.current_spool
        except Exception:
            self.logger.error(f"Failed to capture unload state for {fps_name}")
            return False, f"Failed to prepare unload on {fps_name}"

        # Only set state after all preliminary operations succeed
        fps_state.state = FPSLoadState.UNLOADING
        fps_state.encoder = encoder
        fps_state.since = current_time
        fps_state.current_oams = current_oams_name
        fps_state.current_spool_idx = current_spool
        fps_state.clear_encoder_samples()  # Clear stale encoder samples

        # Cancel post-load pressure check to prevent false positive clog detection during unload
        self._cancel_post_load_pressure_check(fps_state)

        # Ensure follower is set to reverse before starting unload
        try:
            self._set_follower_state(
                fps_name,
                fps_state,
                oams,
                1,
                0,
                "before unload",
                force=True,
            )
        except Exception:
            self.logger.warning(f"Failed to set follower reverse before unload on {fps_name}")

        # REFACTOR: Use AFC's retry config for unload attempts
        afc = self._get_afc()
        max_retries = None
        if afc is not None and hasattr(afc, 'tool_max_unload_attempts'):
            max_retries = afc.tool_max_unload_attempts
            self.logger.debug(f"Using AFC retry config for unload: {max_retries} attempts")

        try:
            success, message = oams.unload_spool_with_retry(max_retries=max_retries)
        except Exception:
            success = False
            message = f"Exception unloading filament on {fps_name}"
            self.logger.error(message)

        if not success:
            self.logger.warning(f"Unload failed on {fps_name}, preparing retry: {message}")
            try:
                unload_lane = fps_state.current_lane or lane_name
                unload_length, unload_speed = (
                    self._get_unload_params(unload_lane) if unload_lane else (None, None)
                )
                retract_feed = unload_speed if unload_speed is not None else 1200.0
                self._set_follower_if_changed(
                    fps_state.current_oams,
                    oams,
                    1,
                    1,
                    "unload retry recovery",
                    force=True,
                )
                fps_state.following = True
                fps_state.direction = 1
                self.reactor.pause(self.reactor.monotonic() + 1.0)
                self._set_follower_state(
                    fps_name,
                    fps_state,
                    oams,
                    1,
                    0,
                    "unload retry recovery",
                    force=True,
                )
                gcode = self._gcode_obj
                if gcode is None:
                    gcode = self.printer.lookup_object("gcode")
                    self._gcode_obj = gcode
                gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_unload_retry")
                try:
                    gcode.run_script_from_command("M83")  # Relative extrusion mode
                    gcode.run_script_from_command("G92 E0")  # Reset extruder position
                    gcode.run_script_from_command(f"G1 E-5.00 F{retract_feed:.0f}")
                    gcode.run_script_from_command("M400")
                finally:
                    gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_unload_retry")
                self.reactor.pause(self.reactor.monotonic() + 1.0)
                success, message = oams.unload_spool_with_retry()
            except Exception:
                self.logger.error(f"Exception while retrying unload on {fps_name}")

        if success:
            since_time = self.reactor.monotonic()

            # Notify AFC that lane is unloaded from toolhead using the normal AFC process
            # This triggers AFC's _apply_lane_sensor_state() which handles everything properly:
            # - Unsyncs lane from extruder
            # - Handles shared prep/load lanes via _update_shared_lane()
            # - Updates virtual sensor via _mirror_lane_to_virtual_sensor()
            # - Calls lane.unit_obj.lane_unloaded() for proper cleanup
            #
            # Runout monitors keep their own cached spool index and lane name,
            # so same-FPS runout coasting/reload logic remains unaffected.
            lane_notified = False
            if lane_name and AMSRunoutCoordinator is not None:
                try:
                    AMSRunoutCoordinator.notify_lane_tool_state(
                        self.printer,
                        fps_state.current_oams or oams.name,
                        lane_name,
                        loaded=False,
                        spool_index=spool_index,
                        eventtime=since_time
                    )
                    lane_notified = True
                    self.logger.debug(f"Notified AFC coordinator that lane {lane_name} unloaded from toolhead")
                except Exception:
                    self.logger.error(f"Failed to notify AFC that lane {lane_name} unloaded on {fps_name}")

            # Fallback: If lane_name wasn't available above, try to resolve it from location
            # This ensures AFC tracking clears even if lane_name was None
            if not lane_notified:
                afc = self._get_afc()
                if afc is not None and AMSRunoutCoordinator is not None:
                    try:
                        afc_lane_name = self._lane_by_location.get((fps_state.current_oams, spool_index)) if spool_index is not None else None
                        if afc_lane_name:
                            lane_obj = afc.lanes.get(afc_lane_name)
                            if lane_obj is not None:
                                try:
                                    AMSRunoutCoordinator.notify_lane_tool_state(
                                        self.printer,
                                        fps_state.current_oams,
                                        afc_lane_name,
                                        loaded=False,
                                        spool_index=spool_index,
                                        eventtime=since_time
                                    )
                                    self.logger.debug(f"Notified AFC coordinator (via location lookup) that lane {afc_lane_name} unloaded")
                                except Exception:
                                    self.logger.error(f"Failed to notify AFC coordinator about lane {afc_lane_name} unload")
                    except Exception:
                        self.logger.error(f"Failed to clear AFC tool tracking during unload cleanup for {fps_name}")

            # Clear LED error state if stuck spool was active before resetting state
            if fps_state.stuck_spool.active and oams is not None and spool_index is not None:
                self._clear_error_led(oams, spool_index, fps_name, "successful unload")

            fps_state.state = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.since = since_time
            fps_state.current_lane = None
            fps_state.current_spool_idx = None
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, message

        # Failed unload: return to LOADED state so another attempt can be issued
        fps_state.state = FPSLoadState.LOADED
        fps_state.since = self.reactor.monotonic()
        fps_state.reset_stuck_spool_state(preserve_restore=True)
        if fps_state.current_oams:
            oams_obj = self._get_oams_object(fps_state.current_oams)
            if oams_obj is not None:
                self._set_follower_if_changed(
                    fps_state.current_oams,
                    oams_obj,
                    1,
                    0,
                    "unload retry",
                    force=True,
                )

        return False, message

    def _clear_lane_on_runout(self, fps_name: str, fps_state: "FPSState", lane_name: Optional[str]) -> None:
        """
        Clear lane state from toolhead and OAMS when runout occurs without infinite spool target.

        This ensures proper cleanup when a lane runs out and has no runout_lane configured.
        Called after the runout sequence completes:
        1. F1S sensor detects empty spool
        2. 60mm PAUSE_DISTANCE allows hub to clear
        3. Follower motor stopped when entering COASTING (hub already cleared)
        4. PTFE amount coasts to toolhead
        5. This method is called to clear state

        Actions:
        - Clears FPS state in OAMS manager (sets UNLOADED, updates follower state flag)
        - Notifies AFC that lane is unloaded from toolhead
        - Resets stuck spool and clog tracking

        Note: Follower motor is already stopped by this point (stopped after hub cleared
        during COASTING transition). This method just updates the state flags.

        Args:
            fps_name: Name of the FPS (e.g., "fps fps1")

            fps_state: Current FPS state object
            lane_name: Name of the lane that ran out (e.g., "lane7")

        """
        if not lane_name:
            self.logger.warning(f"Cannot clear lane on runout - no lane name provided for {fps_name}")
            return

        # Save spool index before clearing for AFC notification
        spool_index = fps_state.current_spool_idx
        oams_name = fps_state.current_oams

        # Clear FPS state (matching _unload_filament_for_fps and cross-extruder clear logic)
        fps_state.state = FPSLoadState.UNLOADED
        fps_state.following = False
        fps_state.direction = 0
        fps_state.since = self.reactor.monotonic()
        fps_state.current_lane = None
        fps_state.current_spool_idx = None
        fps_state.current_oams = None
        fps_state.reset_stuck_spool_state()
        fps_state.reset_clog_tracker()
        fps_state.reset_runout_positions()

        self.logger.debug(f"Cleared FPS state for {fps_name} (was lane {lane_name}, spool {spool_index})")

        # Notify AFC that lane is unloaded from toolhead
        # This triggers AFC's _apply_lane_sensor_state() which:
        # - Handles shared prep/load lanes properly via _update_shared_lane()
        # - Updates virtual sensor via _mirror_lane_to_virtual_sensor()
        # - Calls lane.unit_obj.lane_unloaded() for proper cleanup
        if AMSRunoutCoordinator is not None and oams_name and lane_name:
            try:
                AMSRunoutCoordinator.notify_lane_tool_state(
                    self.printer,
                    oams_name,
                    lane_name,
                    loaded=False,
                    spool_index=spool_index,
                    eventtime=fps_state.since
                )
                self.logger.info(f"Notified AFC coordinator that lane {lane_name} unloaded from toolhead after runout")
            except Exception:
                self.logger.error(f"Failed to notify AFC coordinator about lane {lane_name} unload after runout")
    def _clear_error_led(
        self,
        oam: object,
        spool_idx: int,
        fps_name: str,
        context: str = "operation"
    ) -> bool:
        """Centralized LED clearing with consistent error handling.

        Args:
            oam: OAMS object with set_led_error method
            spool_idx: Spool index (0-3) to clear LED for
            fps_name: FPS name for logging
            context: Context description for logging (e.g., "stuck spool retry", "successful load")

        Returns:
            True if LED cleared successfully, False otherwise
        """
        if oam is None or spool_idx is None:
            return False

        try:
            if hasattr(oam, "set_led_error"):
                oam.set_led_error(spool_idx, 0)
                self.logger.info(f"Cleared LED for {fps_name} spool {spool_idx} after {context}")
                return True
            else:
                self.logger.warning(f"OAMS {getattr(oam, 'name', 'unknown')} does not have set_led_error method")
                return False
        except Exception as e:
            self.logger.error(f"Failed to clear LED for {fps_name} spool {spool_idx} after {context}: {e}")
            return False

    def _perform_stuck_spool_recovery(
        self,
        fps_name: str,
        fps_state: 'FPSState',
        oam: object,
        lane_name: str,
        stuck_attempt: int
    ) -> bool:
        """Perform stuck spool recovery sequence: reverse follower, retract, abort, unload.

        Returns True if recovery completed successfully, False otherwise.
        """
        self.logger.warning(f"Stuck spool detected for {lane_name}, unloading before retry")

        # STEP 1: Set follower to reverse BEFORE extruder retraction
        try:
            self._set_follower_state(fps_name, fps_state, oam, 1, 0, "stuck spool retry", force=True)
            self.logger.info(f"Set follower to reverse before stuck spool retry on {fps_name}")
        except Exception:
            self.logger.warning(f"Failed to set follower reverse before stuck spool retry on {fps_name}")

        # STEP 2: Retract extruder in case filament barely engaged
        try:
            engagement_length, engagement_speed = self._get_engagement_params(lane_name)
            if engagement_length is not None and engagement_speed is not None:
                gcode = self.printer.lookup_object('gcode')
                gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_stuck_retract")
                try:
                    gcode.run_script_from_command("M83")  # Relative extrusion mode
                    gcode.run_script_from_command(f"G1 E-{engagement_length:.2f} F{engagement_speed:.0f}")
                    gcode.run_script_from_command("M400")  # Wait for moves to complete
                    gcode.run_script_from_command(f"G1 E-10.00 F{engagement_speed:.0f}")  # Overlap retract
                finally:
                    gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_stuck_retract")
        except Exception:
            self.logger.error(f"Failed to retract extruder after stuck spool detection for {lane_name}")

        # STEP 3: Abort the stuck load operation
        try:
            oam.abort_current_action()
            self.logger.info(f"Aborted stuck load operation for {lane_name} before unload")
        except Exception:
            self.logger.error(f"Failed to abort stuck load operation for {lane_name}")

        # STEP 4: Unload the stuck filament
        try:
            oam.unload_spool_with_retry()
            # Clear error LED after successful unload
            if fps_state.stuck_spool.active and fps_state.current_spool_idx is not None:
                self._clear_error_led(oam, fps_state.current_spool_idx, fps_name, "stuck spool retry unload")
        except Exception:
            self.logger.error(f"Exception during unload after stuck spool detection for {lane_name}")
            return False

        # Brief cooldown after unload
        try:
            self.reactor.pause(self.reactor.monotonic() + 0.5)
        except Exception:
            pass

        # Notify AFC that lane is unloaded
        if fps_state.current_lane and AMSRunoutCoordinator is not None:
            try:
                AMSRunoutCoordinator.notify_lane_tool_state(
                    self.printer,
                    fps_state.current_oams or oam.name,
                    fps_state.current_lane,
                    loaded=False,
                    spool_index=fps_state.current_spool_idx,
                    eventtime=self.reactor.monotonic()
                )
                self.logger.info(f"Notified AFC that {fps_state.current_lane} is unloaded after stuck spool retry")
            except Exception:
                self.logger.error(f"Failed to notify AFC about unload for {fps_state.current_lane}")

        # Clear fps_state so retry starts fresh
        fps_state.state = FPSLoadState.UNLOADED
        fps_state.current_spool_idx = None
        fps_state.current_oams = None
        fps_state.current_lane = None
        fps_state.since = self.reactor.monotonic()
        fps_state.stuck_spool.active = False
        fps_state.stuck_spool.start_time = None

        return True

    def _attempt_oams_load_with_stuck_retry(
        self,
        fps_name: str,
        fps_state: 'FPSState',
        fps: object,
        oam: object,
        oams_name: str,
        bay_index: int,
        lane_name: str,
        encoder: object,
        current_time: float
    ) -> Tuple[bool, Optional[str]]:
        """Attempt OAMS load with stuck spool retry logic (up to self.stuck_spool_max_attempts).

        Returns (success, error_message)
        """
        for stuck_attempt in range(self.stuck_spool_max_attempts):
            if stuck_attempt > 0:
                self.logger.info(f"Stuck spool retry {stuck_attempt + 1}/{self.stuck_spool_max_attempts} for {lane_name}")

            # Set up FPS state for this load attempt
            fps_state.state = FPSLoadState.LOADING
            fps_state.encoder = encoder
            fps_state.current_oams = oam.name
            fps_state.current_spool_idx = bay_index
            fps_state.current_lane = lane_name
            fps_state.since = current_time
            fps_state.clear_encoder_samples()
            fps_state.reset_engagement_tracking()

            # Enable follower forward before load
            try:
                self._set_follower_state(fps_name, fps_state, oam, 1, 1, "before load", force=True)
            except Exception as e:
                self.logger.warning(f"Failed to set follower forward before load on {fps_name}: {e}")

            # Try to load filament into buffer
            try:
                success, message = oam.load_spool_with_retry(bay_index)
            except Exception:
                success = False
                message = f"Failed to load bay {bay_index} on {oams_name}"
                self.logger.error(message)
                fps_state.state = FPSLoadState.UNLOADED
                self._pause_on_critical_failure(message, oams_name)
                fps_state.engagement_retry_active = False
                return False, message

            if success:
                # OAMS load succeeded!
                self.logger.debug(f"OAMS load succeeded for {lane_name} on stuck attempt {stuck_attempt + 1}")
                return True, None

            # Load failed (stuck spool detected)
            self.logger.warning(f"Stuck spool detected on attempt {stuck_attempt + 1}/{self.stuck_spool_max_attempts} for {lane_name}")

            # Check if we've exhausted stuck spool retries
            if stuck_attempt + 1 >= self.stuck_spool_max_attempts:
                error_msg = (
                    f"Stuck spool detected on {lane_name} after {self.stuck_spool_max_attempts} attempts. "
                    f"Filament may be tangled or spool not feeding properly. "
                    f"Please manually correct the issue, then run: SET_LANE_LOADED LANE={lane_name}, followed by OAMSM_CLEAR_ERRORS"
                )
                self.logger.error(error_msg)
                self._pause_printer_message(error_msg, oams_name)

                # Attempt recovery before pausing
                if self._is_oams_mcu_ready(oam):
                    try:
                        self._set_follower_state(fps_name, fps_state, oam, 1, 0, "stuck spool max retry", force=True)
                        fps_param = fps_name.replace("fps ", "", 1)
                        gcode = self._gcode_obj or self.printer.lookup_object("gcode")
                        self._gcode_obj = gcode
                        gcode.run_script_from_command(f"OAMSM_UNLOAD_FILAMENT FPS={fps_param}")
                    except Exception:
                        self.logger.error(f"Failed to unwind stuck spool before pausing on {fps_name}")
                else:
                    self.logger.error(f"Skipping stuck spool recovery commands for {fps_name} because OAMS MCU is not ready")

                fps_state.engagement_retry_active = False
                return False, error_msg

            # Not max retries yet - perform recovery sequence
            if not self._perform_stuck_spool_recovery(fps_name, fps_state, oam, lane_name, stuck_attempt):
                return False, f"Failed to recover from stuck spool on {lane_name}"

        return False, f"Failed to load {lane_name} after {self.stuck_spool_max_attempts} stuck spool attempts"

    def _perform_engagement_retry_cleanup(
        self,
        fps_name: str,
        fps_state: 'FPSState',
        oam: object,
        lane_name: str,
        engagement_attempt: int
    ) -> bool:
        """Perform cleanup after engagement failure: retract, abort, unload.

        Returns True if cleanup completed successfully, False otherwise.
        """
        self.logger.info(
            f"Filament engagement failed for {lane_name}, unloading before retry "
            f"(engagement attempt {engagement_attempt + 1})"
        )

        # Suppress stuck detection during entire cleanup phase
        # Retraction, abort, and unload all involve low/no encoder movement - don't trigger false stuck detection
        fps_state.engagement_in_progress = True

        try:
            # Retract extruder to back out the filament extruded during engagement
            try:
                engagement_length, engagement_speed = self._get_engagement_params(lane_name)
                if engagement_length is not None and engagement_speed is not None:
                    self.logger.info(f"Retracting extruder {engagement_length:.1f}mm to reverse engagement extrusion for {lane_name}")
                    gcode = self.printer.lookup_object('gcode')
                    gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_engagement_fail")
                    try:
                        gcode.run_script_from_command("M83")  # Relative extrusion mode
                        gcode.run_script_from_command(f"G1 E-{engagement_length:.2f} F{engagement_speed:.0f}")
                        gcode.run_script_from_command("M400")  # Wait for moves to complete
                        gcode.run_script_from_command(f"G1 E-10.00 F{engagement_speed:.0f}")  # Overlap retract
                    finally:
                        gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_engagement_fail")
                else:
                    self.logger.warning(f"Could not get engagement params for {lane_name}, skipping extruder retraction")
            except Exception:
                self.logger.error(f"Failed to retract extruder after engagement failure for {lane_name}")

            # Abort any lingering load operation
            try:
                oam.abort_current_action()
            except Exception:
                self.logger.error(f"Failed to abort load operation before engagement retry for {lane_name}")

            # Unload the filament
            try:
                oam.unload_spool_with_retry()
            except Exception:
                self.logger.error(f"Exception during unload after engagement failure for {lane_name}")
                return False

            # Brief cooldown
            try:
                self.reactor.pause(self.reactor.monotonic() + 0.5)
            except Exception:
                pass

            # Notify AFC that lane is unloaded
            if fps_state.current_lane and AMSRunoutCoordinator is not None:
                try:
                    AMSRunoutCoordinator.notify_lane_tool_state(
                        self.printer,
                        fps_state.current_oams or oam.name,
                        fps_state.current_lane,
                        loaded=False,
                        spool_index=fps_state.current_spool_idx,
                        eventtime=self.reactor.monotonic()
                    )
                    self.logger.info(f"Notified AFC that {fps_state.current_lane} is unloaded after engagement retry")
                except Exception:
                    self.logger.error(f"Failed to notify AFC about unload for {fps_state.current_lane}")

            # Clear fps_state for next retry
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.current_spool_idx = None
            fps_state.current_oams = None
            fps_state.current_lane = None
            fps_state.since = self.reactor.monotonic()

            return True
        finally:
            # Always clear the suppression flag, even if cleanup fails
            fps_state.engagement_in_progress = False

    def _load_filament_for_lane(self, lane_name: str) -> Tuple[bool, str]:
        """Load filament for a lane by deriving OAMS and bay from the lane's unit configuration.

        This eliminates the need for [filament_group] configs by directly using:
        - lane.unit (e.g., "AMS_1:1") to get bay number
        - AFC_OpenAMS unit config to get OAMS name
        """
        afc = self._get_afc()
        if afc is None:
            return False, "AFC not available"

        lane = afc.lanes.get(lane_name)
        if lane is None:
            return False, f"Lane {lane_name} does not exist"

        # Get the unit string and slot/index
        # AFC stores "unit: AMS_1:1" as unit="AMS_1" and index stored separately
        unit_str = getattr(lane, "unit", None)
        if not unit_str:
            return False, f"Lane {lane_name} has no unit defined"

        # Try to get slot number from different possible attributes
        slot_number = None

        # Method 1: If unit_str contains ':', parse it directly (e.g., "AMS_1:1")

        if isinstance(unit_str, str) and ':' in unit_str:
            base_unit_name, slot_str = unit_str.split(':', 1)
            try:
                slot_number = int(slot_str)
            except ValueError:
                return False, f"Invalid slot number in unit {unit_str}"
        else:
            # Method 2: unit_str is just the unit name (e.g., "AMS_1"), get slot from lane.index
            base_unit_name = str(unit_str)
            slot_number = getattr(lane, "index", None)

            if slot_number is None:
                # Method 3: Try to get it from the lane's name if it follows a pattern
                # This is a fallback - lane names might not always have indices
                return False, f"Lane {lane_name} unit '{unit_str}' doesn't specify slot number"

        if slot_number is None:
            return False, f"Could not determine slot number for lane {lane_name}"

        # Convert slot number to 0-indexed bay number
        bay_index = slot_number - 1
        if bay_index < 0:
            return False, f"Invalid slot number {slot_number} (must be >= 1)"

        # Look up the AFC unit object to get OAMS name
        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None:
            units = getattr(afc, "units", {})
            unit_obj = units.get(base_unit_name)

        if unit_obj is None:
            return False, f"AFC unit {base_unit_name} not found"

        # Get the OAMS name from the unit (e.g., "oams1")

        oams_name = getattr(unit_obj, "oams_name", None)
        if not oams_name:
            return False, f"Unit {base_unit_name} has no oams_name defined"

        # Find the OAMS object
        # OAMS objects are stored with full name like "oams oams1", not just "oams1"
        oam = self.oams.get(oams_name)
        if oam is None:
            # Try with "oams " prefix
            oam = self.oams.get(f"oams {oams_name}")

        if oam is None:
            return False, f"OAMS {oams_name} not found"

        # Find which FPS has this OAMS
        fps_name = None
        fps = None
        for fps_name_candidate, fps_candidate in self.fpss.items():
            if hasattr(fps_candidate, "oams"):
                fps_oams = fps_candidate.oams
                if isinstance(fps_oams, list):
                    if oam in fps_oams:
                        fps_name = fps_name_candidate
                        fps = fps_candidate
                        break
                else:
                    if fps_oams == oam:
                        fps_name = fps_name_candidate
                        fps = fps_candidate
                        break

        if not fps_name or fps is None:
            return False, f"No FPS found for OAMS {oams_name}"

        fps_state = self.current_state.fps_state[fps_name]

        # Synchronize with actual loaded lane before deciding how to handle the request
        detected_lane, detected_oams, detected_spool_idx = self.determine_current_loaded_lane(fps_name)

        # Get tool operation status to suppress false positive state clearing messages
        afc = self.printer.lookup_object("AFC", None)
        is_tool_operation = getattr(afc, 'in_toolchange', False) if afc else False

        if detected_lane is not None:
            fps_state.current_lane = detected_lane
            fps_state.current_oams = detected_oams.name if detected_oams else None
            fps_state.current_spool_idx = detected_spool_idx
            fps_state.state = FPSLoadState.LOADED
            fps_state.since = self.reactor.monotonic()
            hub_hes_values = getattr(oam, "hub_hes_value", None)
            hub_has_filament = any(hub_hes_values) if hub_hes_values is not None else False
            if oam.current_spool is None and not hub_has_filament:
                # Skip clearing state during tool operations - AFC state machine handles it
                if not is_tool_operation:
                    self.logger.info(
                        f"Clearing stale AFC lane_loaded state for {detected_lane} on {fps_name} "
                        f"(no spool detected in {oams_name})"
                    )
                if not is_tool_operation and AMSRunoutCoordinator is not None:
                    try:
                        AMSRunoutCoordinator.notify_lane_tool_state(
                            self.printer,
                            fps_state.current_oams or oam.name,
                            detected_lane,
                            loaded=False,
                            spool_index=detected_spool_idx,
                            eventtime=fps_state.since,
                        )
                    except Exception:
                        self.logger.error(f"Failed to clear AFC lane_loaded for {detected_lane} on {fps_name}")
                if not is_tool_operation:
                    fps_state.state = FPSLoadState.UNLOADED
                    fps_state.current_lane = None
                    fps_state.current_oams = None
                    fps_state.current_spool_idx = None
                    detected_lane = None

            if detected_lane is not None:
                # Check what AFC thinks is loaded for this extruder
                # This determines if we need auto-unload or if AFC is handling the sequence
                afc_lane_loaded = None
                try:
                    extruder_obj = getattr(lane, 'extruder_obj', None)
                    if extruder_obj is not None:
                        afc_lane_loaded = getattr(extruder_obj, 'lane_loaded', None)
                except Exception:
                    pass

                # "Already loaded" check - skip during tool changes or if same lane
                if detected_lane == lane_name:
                    if not is_tool_operation:
                        return False, f"Lane {lane_name} is already loaded to {fps_name}"
                    # During tool change: same lane detected, just proceed (timing issue)
                    self.logger.debug(
                        f"Detected {lane_name} already on {fps_name} during tool change - "
                        f"proceeding (likely timing/state lag)"
                    )
                else:
                    # Different lane detected (detected_lane != lane_name)
                    # We must auto-unload the lane currently in the toolhead before loading a new lane.
                    if afc_lane_loaded is None:
                        self.logger.info(
                            f"AFC thinks {fps_name} is empty, but sensors detect {detected_lane} - "
                            f"auto-unloading (stale state from empty shuttle start)"
                        )
                    else:
                        self.logger.info(
                            f"Sensors detect {detected_lane} loaded on {fps_name} while loading {lane_name} - "
                            f"auto-unloading to clear the toolhead"
                        )
                    try:
                        gcode = self._gcode_obj
                        if gcode is None:
                            gcode = self.printer.lookup_object("gcode")
                            self._gcode_obj = gcode

                        self.logger.info(
                            f"Auto-unloading {detected_lane} from {fps_name} before loading {lane_name} "
                            f"(using TOOL_UNLOAD for proper cut/form_tip/retract sequence)"
                        )
                        gcode.run_script_from_command(f"TOOL_UNLOAD LANE={detected_lane}")
                        gcode.run_script_from_command("M400")
                    except Exception as e:
                        return False, f"Failed to unload existing lane {detected_lane} from {fps_name}: {e}"
        else:
            # No lane detected as loaded - clear fps_state if it thinks it's loaded
            # This handles cases where fps_state is stale (e.g., load failed with clog)
            # Skip during tool operations - trust AFC state machine to manage transitions
            if not is_tool_operation and fps_state.state == FPSLoadState.LOADED:
                self.logger.info(f"Clearing stale LOADED state for {fps_name} - no lane detected by AFC")
                fps_state.state = FPSLoadState.UNLOADED
                fps_state.current_lane = None
                fps_state.current_oams = None
                fps_state.current_spool_idx = None

        # Final check: if fps_state still thinks something is loaded, error
        # Skip during tool operations since we already handled auto-unload above
        if not is_tool_operation and fps_state.state == FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is already loaded"

        self._cancel_post_load_pressure_check(fps_state)

        # Check if the bay is ready
        try:
            is_ready = oam.is_bay_ready(bay_index)
        except Exception:
            self.logger.error(f"Failed to check bay {bay_index} readiness on {oams_name}")
            return False, f"Failed to check bay {bay_index} readiness on {oams_name}"

        if not is_ready:
            return False, f"Bay {bay_index} on {oams_name} is not ready (no spool detected)"

        # Load the filament
        self.logger.debug(f"Loading lane {lane_name}: {oams_name} bay {bay_index} via {fps_name}")

        if getattr(oam, "dock_load", False):
            gcode = self._gcode_obj
            if gcode is None:
                try:
                    gcode = self.printer.lookup_object("gcode")
                except Exception:
                    gcode = None
                self._gcode_obj = gcode
            if gcode is not None:
                if not self._run_tool_crash_detection(False):
                    self.logger.warning("Failed to stop tool crash detection before dock unload")
                try:
                    gcode.run_script_from_command("AFC_UNSELECT_TOOL")
                except Exception:
                    self.logger.warning(f"Failed to dock tool before loading {lane_name}")
            else:
                self.logger.warning(f"Failed to dock tool before loading {lane_name}")

        # Capture state BEFORE changing fps_state.state to avoid getting stuck
        try:
            encoder = oam.encoder_clicks
            current_time = self.reactor.monotonic()
            oam_name = oam.name
        except Exception:
            self.logger.error(f"Failed to capture load state for lane {lane_name} bay {bay_index}")
            return False, f"Failed to capture load state for lane {lane_name}"

        # =======================================================================================
        # CONFIGURATION PRECEDENCE: Engagement Retry Count
        # =======================================================================================
        # Uses AFC's tool_max_unload_attempts for engagement retries (semantic repurposing)
        #
        # Priority Order:
        #   1. AFC.tool_max_unload_attempts (if AFC available) - USED FOR ENGAGEMENT RETRIES
        #   2. OAMS.load_retry_max (fallback if AFC not available)
        #
        # Note: AFC's "tool_max_unload_attempts" is semantically an unload config but
        # repurposed here for load engagement retries to maintain AFC as single source of truth.
        # This creates some semantic confusion but ensures consistent retry counts across system.
        #
        # Stuck Spool Retries (separate pool):
        #   - Configured via OAMSM.stuck_spool_max_attempts (default: 2)
        #   - Independent from engagement retries (nested loop structure)
        #   - Each engagement attempt gets fresh stuck spool retry budget
        # =======================================================================================
        afc = self._get_afc()
        if afc is not None and hasattr(afc, 'tool_max_unload_attempts'):
            max_engagement_retries = afc.tool_max_unload_attempts
            self.logger.debug(f"Using AFC retry config for engagement: {max_engagement_retries} attempts (from AFC.tool_max_unload_attempts)")
        else:
            # Fallback to OAMS config if AFC not available
            max_engagement_retries = getattr(oam, "load_retry_max", 3)
            self.logger.debug(f"Using OAMS retry config for engagement: {max_engagement_retries} attempts (AFC config not available)")

        load_success = False
        last_error = None

        self.logger.debug(
            f"Starting load attempts for {lane_name} (max {max_engagement_retries} engagement attempts, "
            f"{self.stuck_spool_max_attempts} stuck spool attempts per engagement try)"
        )

        # Outer loop: Engagement retries (uses configured max_engagement_retries)
        fps_state.engagement_retry_active = True
        for engagement_attempt in range(max_engagement_retries):
            self.logger.debug(
                f"Engagement attempt {engagement_attempt + 1}/{max_engagement_retries} for {lane_name}"
            )

            # Attempt OAMS load with stuck spool retry logic (inner loop)
            oams_load_succeeded, error_msg = self._attempt_oams_load_with_stuck_retry(
                fps_name=fps_name,
                fps_state=fps_state,
                fps=fps,
                oam=oam,
                oams_name=oams_name,
                bay_index=bay_index,
                lane_name=lane_name,
                encoder=encoder,
                current_time=current_time
            )

            if not oams_load_succeeded:
                # Stuck spool max retries reached or critical failure
                last_error = error_msg
                break

            # OAMS load succeeded - now verify filament engaged extruder
            engagement_ok = self._verify_engagement_with_extrude(fps_name, fps_state, fps, lane_name, oam)
            if engagement_ok:
                load_success = True
                break

            # Filament reached extruder but didn't engage - perform cleanup and retry
            if not self._perform_engagement_retry_cleanup(fps_name, fps_state, oam, lane_name, engagement_attempt):
                last_error = f"Failed to clean up after engagement failure for {lane_name}"
                break

            if engagement_attempt + 1 >= max_engagement_retries:
                last_error = f"Filament failed to engage extruder for {lane_name} after {max_engagement_retries} attempts"

        fps_state.engagement_retry_active = False

        if not load_success:
            return False, last_error or f"Failed to load lane {lane_name}"

        old_lane = fps_state.current_lane
        fps_state.current_lane = lane_name  # Store lane name (e.g., "lane8") not map (e.g., "T0")

        fps_state.current_oams = oam.name
        fps_state.current_spool_idx = bay_index

        # If lane actually changed (runout recovery), track the time
        if old_lane is not None and old_lane != lane_name:
            fps_state.last_lane_change_time = self.reactor.monotonic()

        # CRITICAL: Set fps_state.since to the successful load time BEFORE changing state
        successful_load_time = oam.get_last_successful_load_time(bay_index)
        fps_state.since = successful_load_time if successful_load_time is not None else self.reactor.monotonic()

        # Now set state to LOADED after timestamp is correct
        fps_state.state = FPSLoadState.LOADED
        fps_state.direction = 1

        # OPTIMIZATION: Enable follower immediately before cleanup operations
        self._ensure_forward_follower(fps_name, fps_state, "load filament")

        # WORKAROUND: Fix AFC runout helper min_event_systime after load
        # AFC's handle_load_runout() doesn't update this, causing Klipper crashes during manual loads
        self._fix_afc_runout_helper_time(lane_name)

        # Clear LED error state if stuck spool was active
        if fps_state.stuck_spool.active:
            self._clear_error_led(oam, bay_index, fps_name, "successful load")

        fps_state.reset_stuck_spool_state()
        fps_state.reset_clog_tracker()

        # Update virtual tool sensor state after successful load
        # This ensures the virtual sensor reflects that filament is loaded at the toolhead
        try:
            afc = self._get_afc()
            if afc is not None:
                lane = afc.lanes.get(lane_name)
                if lane is not None:
                    unit_obj = getattr(lane, "unit_obj", None)
                    if unit_obj is None:
                        base_unit_name = getattr(lane, "unit", "").split(":")[0] if getattr(lane, "unit", None) else None
                        units = getattr(afc, "units", {})
                        unit_obj = units.get(base_unit_name)

                    if unit_obj is not None and hasattr(unit_obj, '_set_virtual_tool_sensor_state'):
                        eventtime = self.reactor.monotonic()
                        # Call _set_virtual_tool_sensor_state directly with force=True
                        # This is the same approach used in SET_LANE_LOADED wrapper
                        unit_obj._set_virtual_tool_sensor_state(True, eventtime, lane_name, force=True, lane_obj=lane)
                        self.logger.debug(
                            f"Updated virtual tool sensor to LOADED for {lane_name} after successful load"
                        )

                        # CRITICAL: Notify AFC that lane is loaded to toolhead
                        # This calls handle_openams_lane_tool_state which sets extruder.lane_loaded
                        # Without this, determine_current_loaded_lane() can't detect what's loaded
                        if AMSRunoutCoordinator is not None:
                            try:
                                AMSRunoutCoordinator.notify_lane_tool_state(
                                    self.printer, oam_name, lane_name,
                                    loaded=True, spool_index=bay_index, eventtime=eventtime
                                )
                                self.logger.debug(f"Notified AFC that lane {lane_name} is loaded to toolhead")
                            except Exception:
                                self.logger.error(f"Failed to notify AFC that lane {lane_name} loaded")
                    else:
                        self.logger.debug(f"Virtual tool sensor update not available for {lane_name}")
        except Exception:
            self.logger.warning(f"Failed to update virtual tool sensor for {lane_name} after load")

        # Force-update loaded_to_hub from hardware hub sensor after successful load.
        # During engagement retries, set_unloaded() clears loaded_to_hub but the hub_changed
        # event may not re-fire if the hardware sensor didn't change between polling cycles
        # (edge-triggered polling misses software-only state changes).
        try:
            afc = self._get_afc()
            if afc is not None:
                lane_obj = afc.lanes.get(lane_name)
                if lane_obj is not None:
                    hub_hes_values = getattr(oam, "hub_hes_value", None)
                    if hub_hes_values is not None and bay_index < len(hub_hes_values):
                        hub_sensor_state = bool(hub_hes_values[bay_index])
                        if lane_obj.loaded_to_hub != hub_sensor_state:
                            lane_obj.loaded_to_hub = hub_sensor_state
                            self.logger.debug(
                                f"Force-updated loaded_to_hub={hub_sensor_state} for {lane_name} "
                                f"from hardware after successful load"
                            )
        except Exception:
            pass

        # Monitors are already running globally, no need to restart them
        if getattr(oam, "dock_load", False):
            extruder_obj = getattr(lane, "extruder_obj", None)
            extruder_name = getattr(extruder_obj, "name", None) if extruder_obj else None
            purge_length = getattr(oam, "post_load_purge", 0.0) or 0.0
            if purge_length > 0:
                _, purge_speed = self._get_reload_params(lane_name)
                purge_feed = purge_speed if purge_speed is not None else 1200.0
                try:
                    gcode = self._gcode_obj
                    if gcode is None:
                        gcode = self.printer.lookup_object("gcode")
                        self._gcode_obj = gcode
                    gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_post_purge")
                    try:
                        gcode.run_script_from_command("M83")  # Relative extrusion mode
                        gcode.run_script_from_command("G92 E0")  # Reset extruder position
                        gcode.run_script_from_command(f"G1 E{purge_length:.2f} F{purge_feed:.0f}")
                        gcode.run_script_from_command("M400")
                    finally:
                        gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_post_purge")
                except Exception:
                    self.logger.warning(f"Failed to run post-load purge for {lane_name}")
            if extruder_name:
                try:
                    gcode = self._gcode_obj
                    if gcode is None:
                        gcode = self.printer.lookup_object("gcode")
                        self._gcode_obj = gcode
                    if not self._run_tool_crash_detection(True):
                        self.logger.warning("Failed to start tool crash detection before tool select")
                    gcode.run_script_from_command(f"AFC_SELECT_TOOL TOOL={extruder_name}")
                except Exception:
                    self.logger.warning(f"Failed to select tool {extruder_name} after loading {lane_name}")
            else:
                self.logger.warning(f"No extruder name available to select tool after loading {lane_name}")
        return True, f"Loaded lane {lane_name} ({oam_name} bay {bay_index})"

        # Fallback - should not be hit, but return a failure tuple instead of None
        return False, f"Failed to load lane {lane_name}"


    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps_param = gcmd.get('FPS')
        fps_name = "fps " + fps_param
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")

            return

        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.UNLOADED:
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")

            return
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            self.logger.debug(f"OAMSM_UNLOAD_FILAMENT ignored because {fps_name} is busy")

            return

        extra_retract_raw = gcmd.get('EXTRA_RETRACT', None)
        if extra_retract_raw is not None:
            try:
                extra_retract = float(extra_retract_raw)
            except Exception:
                raise gcmd.error("EXTRA_RETRACT must be a number")
        else:
            oams_obj = self._get_oams_object(fps_state.current_oams) if fps_state.current_oams else None
            extra_retract = getattr(oams_obj, "extra_retract", None)
            if extra_retract is None:
                extra_retract = self.extra_retract_default

        # Queue a small extra retract move to overlap with the unload sequence
        extra_retract_lane = fps_state.current_lane
        if extra_retract_lane is not None:
            _, reload_speed = self._get_reload_params(extra_retract_lane)
            unload_length, unload_speed = self._get_unload_params(extra_retract_lane)
            extra_retract_feed_rate = (
                unload_speed
                if unload_speed is not None
                else (reload_speed if reload_speed is not None else 1500.0)
            )
            unload_length_display = f"{unload_length:.2f}" if unload_length is not None else "None"
            unload_speed_display = f"{unload_speed:.0f}" if unload_speed is not None else "None"
            self.logger.debug(
                f"Unload params for {extra_retract_lane} on {fps_name}: "
                f"unload_length={unload_length_display}mm unload_speed={unload_speed_display}mm/min "
                f"extra_retract={extra_retract:.2f}mm feed_rate={extra_retract_feed_rate:.0f}mm/min"
            )
            reverse_direction = 0  # Pull back during unload overlap

            # Ensure follower is enabled in reverse before the initial unload retract
            try:
                oams_obj = self._get_oams_object(fps_state.current_oams) if fps_state.current_oams else None
                if oams_obj is None:
                    oams_obj = self._resolve_oams_for_lane(extra_retract_lane)
                    if oams_obj is not None and fps_state.current_oams is None:
                        fps_state.current_oams = oams_obj.name
                self._set_follower_state(
                    fps_name,
                    fps_state,
                    oams_obj,
                    1,
                    reverse_direction,
                    "unload extra retract",
                    force=True,
                )
            except Exception:
                self.logger.warning(f"Unable to set follower reverse before extra retract on {fps_name}")

            try:
                gcode = self._gcode_obj
                if gcode is None:
                    gcode = self.printer.lookup_object("gcode")
                    self._gcode_obj = gcode
                gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_extra_retract")
                try:
                    gcode.run_script_from_command("M83")  # Relative extrusion mode
                    gcode.run_script_from_command("G92 E0")  # Reset extruder position

                    # First retract by the configured unload length (if available)
                    if unload_length is not None:
                        unload_feed = unload_speed if unload_speed is not None else extra_retract_feed_rate
                        gcode.run_script_from_command(f"G1 E-{unload_length:.2f} F{unload_feed:.0f}")
                        gcode.run_script_from_command("M400")

                    # Wait for retract moves to complete before extra retract/unload
                    gcode.run_script_from_command("M400")

                    # Then issue the extra retract before unload
                    gcode.run_script_from_command(f"G1 E{extra_retract:.2f} F{extra_retract_feed_rate:.0f}")
                finally:
                    gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_extra_retract")
            except Exception:
                self.logger.warning(f"Skipping extra retract before unload on {fps_name}: unable to queue gcode")
        else:
            self.logger.info(f"Skipping extra retract before unload on {fps_name}: no lane resolved")

        success, message = self._unload_filament_for_fps(fps_name)

        if not success or (message and message != "Spool unloaded successfully"):
            gcmd.respond_info(message)

    cmd_LOAD_FILAMENT_help = "Load a spool from a specific AFC lane (LANE=name)"
    def cmd_LOAD_FILAMENT(self, gcmd):
        lane_name = gcmd.get('LANE', None)

        if not lane_name:
            raise gcmd.error("LANE parameter is required (e.g., LANE=lane4)")

        # Load directly from lane configuration
        success, message = self._load_filament_for_lane(lane_name)
        if not success:
            # Raise to halt any enclosing macro/script so we don't continue with a bad state
            raise gcmd.error(message or f"Failed to load {lane_name}")
        gcmd.respond_info(message or f"Loaded {lane_name}")

    def _pause_on_critical_failure(self, error_message: str, oams_name: Optional[str] = None) -> None:
        """
        Pause the printer if a critical failure occurs during printing.

        This prevents catastrophic failures like:
        - Printing without filament loaded
        - Attempting to extrude with no material
        - Nozzle damage from dry printing

        The pause is scheduled asynchronously to avoid deadlocks when called
        from within gcode commands (like custom load macros).

        Only pauses if printer is actively printing. Manual operations during idle
        will just report the error without pausing.

        Args:
            error_message: Description of the failure
            oams_name: Name of the OAMS unit that encountered the error
        """
        # OPTIMIZATION: Use cached idle_timeout object
        idle_timeout = self._idle_timeout_obj
        if idle_timeout is None:
            try:
                idle_timeout = self.printer.lookup_object("idle_timeout")

                self._idle_timeout_obj = idle_timeout
            except Exception:
                self.logger.warning("Cannot check printer state for pause decision")

                # Schedule async pause to avoid deadlock
                self._schedule_async_pause(error_message, oams_name)
                return

        # Check if printer is currently printing
        try:
            eventtime = self.reactor.monotonic()
            status = idle_timeout.get_status(eventtime)
            is_printing = status.get("state") == "Printing"
        except Exception:
            self.logger.error("Failed to get printer state")

            # Schedule async pause to avoid deadlock
            self._schedule_async_pause(error_message, oams_name)
            return

        if is_printing:
            # Critical: printer is trying to print without filament loaded
            self.logger.error(f"CRITICAL FAILURE during printing: {error_message} - PAUSING PRINTER")
            # Schedule pause asynchronously to avoid deadlock when called from gcode command
            self._schedule_async_pause(error_message, oams_name)
        else:
            # Not printing, just log the error - user can retry manually
            self.logger.warning(f"Load failed while idle: {error_message}")
    def _schedule_async_pause(self, message: str, oams_name: Optional[str] = None) -> None:
        """
        Schedule a pause to happen asynchronously via reactor timer.

        This prevents deadlocks when pause is triggered from within a gcode command.
        The timer will fire after the current command completes, allowing the pause
        to execute in a clean context.
        """
        def _do_pause(eventtime):
            try:
                self._pause_printer_message(message, oams_name)
            except Exception:
                self.logger.error("Failed to execute async pause")

            return self.reactor.NEVER

        # Schedule pause to happen ASAP (0.05s delay to let current command finish)
        self.reactor.register_timer(_do_pause, self.reactor.monotonic() + 0.05)

    def _pause_printer_message(self, message, oams_name: Optional[str] = None, notify_afc: bool = True):
        self.logger.info(message)

        if notify_afc and AMSRunoutCoordinator is not None and oams_name:
            try:
                AMSRunoutCoordinator.notify_afc_error(self.printer, oams_name, message, pause=False)
            except Exception:
                self.logger.error("Failed to forward OAMS pause message to AFC")


        # OPTIMIZATION: Use cached gcode object
        gcode = self._gcode_obj
        if gcode is None:
            try:
                gcode = self.printer.lookup_object("gcode")

                self._gcode_obj = gcode
            except Exception:
                self.logger.error("Failed to look up gcode object for pause message")

                return

        pause_message = f"Print has been paused: {message}"
        try:
            gcode.run_script(f"M118 {pause_message}")

            gcode.run_script(f"M114 {pause_message}")

        except Exception:
            self.logger.error("Failed to send pause notification gcode")


        # OPTIMIZATION: Use cached toolhead object
        toolhead = self._toolhead_obj
        if toolhead is None:
            try:
                toolhead = self.printer.lookup_object("toolhead")

                self._toolhead_obj = toolhead
            except Exception:
                self.logger.error("Failed to query toolhead state during pause handling")

                return

        try:
            homed_axes = toolhead.get_status(self.reactor.monotonic()).get("homed_axes", "")

        except Exception:
            self.logger.error("Failed to query toolhead state during pause handling")

            return

        try:
            state_message = self.printer.get_state_message()
        except Exception:
            state_message = None

        if isinstance(state_message, (list, tuple)) and state_message:
            printer_state_text = state_message[0]
        else:
            printer_state_text = state_message

        printer_state_text = printer_state_text if isinstance(printer_state_text, str) else None

        if printer_state_text:
            lowered_state = printer_state_text.lower()
            if "lost communication" in lowered_state or "mcu" in lowered_state:
                self.logger.warning(
                    "Printer reported an error state during pause handling: %s",
                    printer_state_text,
                )
                gcode.respond_info(
                    f"Pause notification may fail because printer reported: {printer_state_text}"
                )

        already_paused = False
        try:
            pause_resume = self.printer.lookup_object("pause_resume")

        except Exception:
            pause_resume = None

        if pause_resume is not None:
            try:
                already_paused = bool(getattr(pause_resume, "is_paused", False))
            except Exception:
                already_paused = False

        if already_paused:
            self.logger.debug("Skipping PAUSE command because printer is already paused")

            return

        if all(axis in homed_axes for axis in ("x", "y", "z")):
            pause_attempted = False
            pause_successful = False
            try:
                gcode.run_script("PAUSE")

                pause_attempted = True

                # Verify pause state after attempting to pause
                if pause_resume is not None:
                    try:
                        pause_successful = bool(getattr(pause_resume, "is_paused", False))
                    except Exception:
                        self.logger.error("Failed to verify pause state after PAUSE command")

            except Exception:
                self.logger.error("Failed to run PAUSE script")


            if pause_attempted and not pause_successful:
                self.logger.error(
                    "CRITICAL: Failed to pause printer for critical error: %s. "
                    "Print may continue despite error condition!",
                    message
                )
        else:
            self.logger.warning(f"Skipping PAUSE command because axes are not homed (homed_axes={homed_axes})")
    def _cancel_post_load_pressure_check(self, fps_state: "FPSState") -> None:
        timer = getattr(fps_state, "post_load_pressure_timer", None)
        if timer is not None:
            try:
                self.reactor.unregister_timer(timer)
            except Exception:
                self.logger.error("Failed to cancel post-load pressure timer")

        fps_state.post_load_pressure_timer = None
        fps_state.post_load_pressure_start = None

    def _schedule_post_load_pressure_check(self, fps_name: str, fps_state: "FPSState") -> None:
        self._cancel_post_load_pressure_check(fps_state)

        def _monitor_pressure(self, eventtime):
            tracked_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)

            if tracked_state is None or fps is None:
                if tracked_state is not None:
                    self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            if tracked_state.state != FPSLoadState.LOADED:
                self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            pressure = float(getattr(fps, "fps_value", 0.0))
            if pressure <= POST_LOAD_PRESSURE_THRESHOLD:
                self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            now = self.reactor.monotonic()
            if tracked_state.post_load_pressure_start is None:
                tracked_state.post_load_pressure_start = now
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            if now - tracked_state.post_load_pressure_start < self.post_load_pressure_dwell:
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            oams_obj = None
            if tracked_state.current_oams is not None:
                oams_obj = self._get_oams_object(tracked_state.current_oams)
            if (oams_obj is not None and tracked_state.current_spool_idx is not None):
                try:
                    oams_obj.set_led_error(tracked_state.current_spool_idx, 1)
                except Exception:
                    self.logger.error(f"Failed to set clog LED on {fps_name} spool {tracked_state.current_spool_idx} after loading")

            message = f"Possible clog detected after loading {tracked_state.current_lane or fps_name}: FPS pressure {pressure:.2f} remained above {POST_LOAD_PRESSURE_THRESHOLD:.2f}"

            # Pause printer with error message
            self._pause_printer_message(message, tracked_state.current_oams)

            tracked_state.clog.active = True

            self._cancel_post_load_pressure_check(tracked_state)
            return self.reactor.NEVER

        timer = self.reactor.register_timer(partial(_monitor_pressure, self), self.reactor.NOW)
        fps_state.post_load_pressure_timer = timer
        fps_state.post_load_pressure_start = None

    def _enable_follower(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], direction: int, context: str) -> None:
        """
        Enable the OAMS follower motor to track filament movement.

        The follower motor maintains proper tension on the filament by following its movement
        through the buffer tube. This is essential for accurate encoder tracking and preventing
        filament tangles.

        Args:
            fps_name: Name of the FPS (Filament Pressure Sensor) being controlled
            fps_state: Current state object for the FPS
            oams: OAMS object controlling the hardware (can be None, will be looked up)
            direction: Follower direction (0=reverse, 1=forward)
            context: Description of why follower is being enabled (for logging)

        State Updates:
            - fps_state.following: Set to True on success
            - fps_state.direction: Updated to match requested direction

        Notes:
            - Direction defaults to forward (1) if invalid value provided
            - Fails silently if no spool is loaded (current_spool_idx is None)
            - Logs exceptions but doesn't raise them to avoid disrupting workflow
            - Uses state-tracked helper to avoid redundant MCU commands
        """
        if fps_state.current_spool_idx is None:
            return

        if oams is None and fps_state.current_oams is not None:
            oams = self._get_oams_object(fps_state.current_oams)
        if oams is None:
            self.logger.warning("Cannot enable follower: OAMS not found")

            return

        if fps_state.current_oams != oams.name:
            fps_state.current_oams = oams.name

        direction = direction if direction in (0, 1) else 1

        # Use state-tracked helper to avoid overwhelming MCU with redundant commands
        self._set_follower_if_changed(fps_state.current_oams, oams, 1, direction, context)

        # Update FPS state to reflect follower is now enabled
        # Note: _set_follower_if_changed updates follower_last_state tracking
        fps_state.following = True
        fps_state.direction = direction

    def _set_follower_state(
        self,
        fps_name: str,
        fps_state: "FPSState",
        oams: Optional[Any],
        enable: int,
        direction: int,
        context: str,
        force: bool = False,
    ) -> None:
        """Set follower state directly and update cached FPS state."""
        if oams is None and fps_state.current_oams is not None:
            oams = self._get_oams_object(fps_state.current_oams)
        if oams is None:
            self.logger.warning(f"Cannot set follower state for {fps_name}: OAMS not found")
            return

        if fps_state.current_oams != oams.name:
            fps_state.current_oams = oams.name

        oams_name = fps_state.current_oams or getattr(oams, "name", None)
        if not oams_name:
            self.logger.warning(f"Cannot set follower state for {fps_name}: OAMS name unavailable")
            return

        direction = direction if direction in (0, 1) else 1
        oams_name = self._normalize_oams_name(oams_name, oams)
        self._set_follower_if_changed(oams_name, oams, enable, direction, context, force=force)
        fps_state.following = bool(enable)
        fps_state.direction = direction if enable else 0

    def _ensure_forward_follower(self, fps_name: str, fps_state: "FPSState", context: str) -> None:
        """Ensure follower is enabled forward when filament is present."""
        if fps_state.current_spool_idx is None:
            self.logger.debug(f"_ensure_forward_follower skipped for {fps_name}: current_spool_idx is None ({context})")
            return

        oams = None
        if fps_state.current_oams is not None:
            oams = self._get_oams_object(fps_state.current_oams)
        if oams is None:
            self.logger.warning(
                f"_ensure_forward_follower failed for {fps_name}: OAMS '{fps_state.current_oams}' not found ({context})"
            )
            return

        self._set_follower_state(
            fps_name,
            fps_state,
            oams,
            1,
            1,
            context,
            force=True,
        )

    def _rate_limited_mcu_command(self, oams_name: str, command_fn: Callable, *args, **kwargs) -> None:
        """
        Execute MCU command with completion-aware queuing to prevent queue overflow.

        Waits for previous command to complete (action_status becomes None) before
        sending next command. Queues commands if OAMS is busy.

        Args:
            oams_name: Name of OAMS unit
            command_fn: Function to call (e.g., oams.set_led_error)
            *args, **kwargs: Arguments to pass to command_fn
        """
        # Get OAMS object to check status
        oams = self.oams.get(oams_name)
        if oams is None:
            self.logger.error(f"Cannot send command to {oams_name} - OAMS not found")
            return
        if not self._is_oams_mcu_ready(oams):
            self.logger.debug(f"Skipping command for {oams_name} - MCU not ready")
            return

        # Initialize queue if needed
        if oams_name not in self._mcu_command_queue:
            self._mcu_command_queue[oams_name] = []
            self._mcu_command_in_flight[oams_name] = False

        # Add command to queue
        self._mcu_command_queue[oams_name].append((command_fn, args, kwargs))

        # Try to process queue
        self._process_mcu_command_queue(oams_name)

    def _process_mcu_command_queue(self, oams_name: str) -> None:
        """
        Process queued MCU commands for an OAMS, waiting for each to complete.

        Args:
            oams_name: Name of OAMS unit
        """
        oams = self.oams.get(oams_name)
        if oams is None:
            return

        # Skip if command already in flight
        if self._mcu_command_in_flight.get(oams_name, False):
            return

        # Skip if no commands queued
        queue = self._mcu_command_queue.get(oams_name, [])
        if not queue:
            return

        # Check if OAMS is busy with a load/unload operation
        # action_status is None when idle, set to LOADING/UNLOADING/etc when busy
        if getattr(oams, "action_status", None) is not None:
            # OAMS is busy - schedule retry after operation completes
            # Use short polling interval (100ms) to detect completion quickly
            def _retry_queue(eventtime):
                self._mcu_command_in_flight[oams_name] = False
                self._process_mcu_command_queue(oams_name)
                return self.reactor.NEVER

            # Cancel existing poll timer if any
            if oams_name in self._mcu_command_poll_timers:
                try:
                    self.reactor.unregister_timer(self._mcu_command_poll_timers[oams_name])
                except Exception:
                    pass

            # Register new poll timer
            timer = self.reactor.register_timer(_retry_queue, self.reactor.NOW + 0.1)
            self._mcu_command_poll_timers[oams_name] = timer
            return

        # OAMS is idle - send next command
        command_fn, args, kwargs = queue.pop(0)
        self._mcu_command_in_flight[oams_name] = True

        try:
            command_fn(*args, **kwargs)
            self.logger.debug(f"Sent MCU command to {oams_name} ({len(queue)} remaining in queue)")
        except Exception as e:
            self.logger.error(f"MCU command failed for {oams_name}: {e}")

        # Schedule completion check after short delay (50ms) to allow command to process
        # Then check if action_status changed (command started processing)
        def _check_completion(eventtime):
            # Check if OAMS MCU is still available
            current_oams = self.oams.get(oams_name)
            if current_oams is None or not self._is_oams_mcu_ready(current_oams):
                self.logger.debug(f"OAMS {oams_name} no longer available, clearing command queue")
                self._mcu_command_in_flight[oams_name] = False
                self._mcu_command_queue.pop(oams_name, None)
                return self.reactor.NEVER

            # If action_status is still None, command completed immediately (like set_led)
            # If action_status is set, need to wait for it to return to None
            if getattr(current_oams, "action_status", None) is None:
                # Command completed - mark as not in flight and process next
                self._mcu_command_in_flight[oams_name] = False
                self._process_mcu_command_queue(oams_name)
                return self.reactor.NEVER
            else:
                # Command started an operation - poll for completion
                return eventtime + 0.1  # Poll every 100ms

        # Cancel existing poll timer if any
        if oams_name in self._mcu_command_poll_timers:
            try:
                self.reactor.unregister_timer(self._mcu_command_poll_timers[oams_name])
            except Exception:
                pass

        # Register completion check timer
        timer = self.reactor.register_timer(_check_completion, self.reactor.NOW + 0.05)
        self._mcu_command_poll_timers[oams_name] = timer

    def _set_led_error_if_changed(self, oams: Any, oams_name: str, spool_idx: int, error_state: int, context: str = "") -> None:
        """
        Send LED error command only if state has changed to avoid overwhelming MCU with redundant commands.

        Args:
            oams: OAMS object
            oams_name: Name of the OAMS
            spool_idx: Spool index (0-based)
            error_state: 0 to clear, 1 to set error
            context: Description for logging (optional)
        """
        led_key = f"{oams_name}:{spool_idx}"
        last_state = self.led_error_state.get(led_key, None)

        # Only send command if state changed or this is the first command
        if last_state != error_state:
            # Use rate limiting to prevent MCU queue overflow
            self._rate_limited_mcu_command(oams_name, oams.set_led_error, spool_idx, error_state)
            self.led_error_state[led_key] = error_state
            if context:
                self.logger.debug(f"LED error {'set' if error_state else 'cleared'} for {oams_name} spool {spool_idx} ({context})")

    def _is_oams_mcu_ready(self, oams: Any) -> bool:
        """True when the OAMS MCU is reachable and not shutdown."""

        mcu = getattr(oams, "mcu", None)
        try:
            if mcu is None:
                return False

            if hasattr(mcu, "is_shutdown") and mcu.is_shutdown():
                return False

            serial = getattr(mcu, "serial", None)
            if serial is not None:
                if hasattr(serial, "is_shutdown") and serial.is_shutdown():
                    return False

            if hasattr(mcu, "is_connected"):
                if not mcu.is_connected():
                    return False
            elif serial is not None and hasattr(serial, "is_connected"):
                return bool(serial.is_connected())
        except Exception:
            self.logger.debug(f"Could not read MCU state for {getattr(oams, 'name', '<unknown>')}")
            return False

        return True

    def _set_follower_if_changed(
        self,
        oams_name: str,
        oams: Any,
        enable: int,
        direction: int,
        context: str = "",
        force: bool = False,
    ) -> None:
        """
        Send follower command only if state has changed to avoid overwhelming MCU with redundant commands.

        Args:
            oams_name: Name of the OAMS
            oams: OAMS object
            enable: 1 to enable, 0 to disable
            direction: 0 for reverse, 1 for forward
            context: Description for logging (optional)
        """
        oams_name = self._normalize_oams_name(oams_name, oams)
        state = self._get_follower_state(oams_name)
        desired_state = (enable, direction)
        self._log_follower_request(oams_name, desired_state, context, force=force)

        if not self._is_oams_mcu_ready(oams):
            if force:
                try:
                    self._log_follower_change(oams_name, enable, direction, context, forced=True)
                    oams.set_oams_follower(enable, direction)
                    state.last_state = desired_state
                    self.logger.debug(
                        f"Forced follower change for {oams_name} ({context or 'no context'}) while MCU not ready"
                    )
                except Exception:
                    self.logger.error(
                        f"Failed to force follower change for {oams_name} ({context or 'no context'}) while MCU not ready"
                    )
                return
            self.logger.debug(f"Skipping follower change for {oams_name} ({context or 'no context'}) because MCU is not ready")
            return

        # Only send command if state changed or this is the first command
        if force or state.last_state != desired_state:
            try:
                if oams_name not in self.oams and oams is not None:
                    self.oams[oams_name] = oams
                self._log_follower_change(oams_name, enable, direction, context, forced=force)
                oams.set_oams_follower(enable, direction)
                state.last_state = desired_state
                # Log follower disable at INFO level to track what's disabling it during clogs
                if enable:
                    self.logger.debug(f"Follower enabled for {oams_name} ({context or 'no context'})")
                else:
                    self.logger.info(f"Follower DISABLED for {oams_name} ({context or 'no context'})")

            except Exception:
                self.logger.error(f"Failed to {'enable' if enable else 'disable'} follower for {oams_name}{f' ({context})' if context else ''}")

    def _log_follower_request(
        self,
        oams_name: str,
        desired_state: Tuple[int, int],
        context: str,
        *,
        force: bool = False,
    ) -> None:
        """Debug log follower requests only when state changes or forced."""
        state = self._get_follower_state(oams_name)
        if force or state.last_state != desired_state:
            self.logger.debug(
                f"Follower request for {oams_name}: enable={desired_state[0]} "
                f"direction={desired_state[1]} context={context or 'no context'} force={force}"
            )

    def _log_follower_change(
        self,
        oams_name: str,
        enable: int,
        direction: int,
        context: str,
        *,
        forced: bool = False,
    ) -> None:
        """Debug log actual follower commands sent."""
        self.logger.debug(
            f"Follower command for {oams_name}: enable={enable} direction={direction} "
            f"context={context or 'no context'} forced={forced}"
        )

    def _update_follower_for_oams(self, oams_name: str, oams: Any) -> None:
        """Follower is manually controlled; no automatic updates."""
        return
    def _ensure_followers_for_loaded_hubs(self) -> None:
        """
        Ensure followers are enabled forward for any OAMS unit that has:
        1. Any hub sensor showing filament loaded, OR
        2. Any lane on that unit showing loaded to toolhead

        Called after OAMSM_CLEAR_ERRORS and other state changes.
        User requirement: Keep follower feeding filament towards extruder when filament is present.
        """
        afc = self._get_afc()
        if afc is None or not hasattr(afc, 'units'):
            return

        for unit_name, unit_obj in afc.units.items():
            # Only process OpenAMS units
            if not hasattr(unit_obj, 'oams_name'):
                continue

            oams_name = unit_obj.oams_name
            oams = self.oams.get(oams_name)
            if oams is None or not self._is_oams_mcu_ready(oams):
                continue

            should_enable_follower = False

            # Check 1: Does any hub sensor show filament loaded?
            hub_hes_values = getattr(oams, "hub_hes_value", None)
            if hub_hes_values is not None and any(hub_hes_values):
                should_enable_follower = True
                self.logger.debug(f"Hub sensors on {oams_name} show filament loaded - enabling follower forward")

            # Check 2: Does any lane on this unit show loaded to toolhead?
            if not should_enable_follower and hasattr(unit_obj, 'lanes'):
                for lane_name, lane in unit_obj.lanes.items():
                    if getattr(lane, 'tool_loaded', False):
                        should_enable_follower = True
                        self.logger.debug(f"Lane {lane_name} loaded to toolhead - enabling follower forward on {oams_name}")
                        break

            # Enable follower forward if conditions met
            if should_enable_follower:
                # Find the FPS for this OAMS - use first loaded bay
                fps_name = None
                for fps_name_candidate, fps_state in self.current_state.fps_state.items():
                    if fps_state.current_oams == oams_name and fps_state.current_spool_idx is not None:
                        fps_name = fps_name_candidate
                        break

                if fps_name:
                    fps_state = self.current_state.fps_state.get(fps_name)
                    if fps_state:
                        try:
                            self._enable_follower(fps_name, fps_state, oams, 1, "CLEAR_ERRORS with filament present")
                            self.logger.info(f"Enabled follower forward on {oams_name} {fps_name} - filament present in hub/toolhead")
                        except Exception as e:
                            self.logger.error(f"Failed to enable follower on {oams_name} {fps_name}: {e}")

    def _force_enable_followers(self, ready_oams: Dict[str, Any]) -> None:
        """
        Force followers on forward for all ready OAMS controllers.
        This ensures manual extrusion is available during error recovery.
        """
        for oams_name, oams in ready_oams.items():
            # Find first FPS associated with this OAMS
            fps_name = None
            for fps_name_candidate, fps_state in self.current_state.fps_state.items():
                if fps_state.current_oams == oams_name:
                    fps_name = fps_name_candidate
                    break

            if fps_name:
                fps_state = self.current_state.fps_state.get(fps_name)
                if fps_state and fps_state.current_spool_idx is not None:
                    try:
                        self._enable_follower(fps_name, fps_state, oams, 1, "CLEAR_ERRORS force enable")
                        self.logger.debug(f"Force enabled follower forward on {oams_name} {fps_name}")
                    except Exception as e:
                        self.logger.error(f"Failed to force enable follower on {oams_name} {fps_name}: {e}")

    def _find_fps_for_oams_bay(self, oams_name: str, bay_idx: int) -> Optional[str]:
        """Find the FPS name that corresponds to a specific OAMS bay."""
        # Query AFC to find which lane uses this OAMS bay
        afc = self._get_afc()
        if afc is None or not hasattr(afc, 'lanes'):
            return None

        # Look through AFC lanes to find one that matches this OAMS and bay
        for lane_name, lane in afc.lanes.items():
            # Get lane's unit and slot
            unit_str = getattr(lane, "unit", None)
            if not unit_str:
                continue

            # Parse unit name and slot
            if isinstance(unit_str, str) and ':' in unit_str:
                base_unit_name, slot_str = unit_str.split(':', 1)
                try:
                    slot_number = int(slot_str)
                except ValueError:
                    continue
            else:
                base_unit_name = str(unit_str)
                slot_number = getattr(lane, "index", None)
                if slot_number is None:
                    continue

            # Convert slot to bay index
            lane_bay_idx = slot_number - 1
            if lane_bay_idx != bay_idx:
                continue

            # Check if this lane's unit uses this OAMS
            unit_obj = getattr(lane, "unit_obj", None)
            if unit_obj is None:
                units = getattr(afc, "units", {})
                unit_obj = units.get(base_unit_name)
            if unit_obj is None:
                continue

            lane_oams_name = getattr(unit_obj, "oams_name", None)
            if lane_oams_name != oams_name and f"oams {lane_oams_name}" != oams_name:
                continue

            # Found matching lane - return its FPS
            return self.get_fps_for_afc_lane(lane_name)

        return None

    def _restore_follower_if_needed(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], context: str) -> None:
        if not fps_state.stuck_spool.restore_follower:
            return

        if fps_state.current_oams is None:
            fps_state.stuck_spool.restore_follower = False
            return

        if oams is None:
            oams = self._get_oams_object(fps_state.current_oams)
        if oams is None:
            return

        direction = fps_state.stuck_spool.restore_direction
        self._enable_follower(fps_name, fps_state, oams, direction, context)
        if fps_state.following:
            fps_state.stuck_spool.restore_follower = False
            self.logger.info(f"Restarted follower for {fps_name} spool {fps_state.current_spool_idx} after {context}.")
    def _handle_printing_resumed(self, _eventtime):
        # CRITICAL: Check if printer is actually still paused before processing resume
        # The pause:resume event can fire during TOOL_UNLOAD operations while printer is paused
        # Processing resume logic while paused causes conflicts (e.g., enabling follower during AFC_Cut)
        pause_resume = self._pause_resume_obj
        if pause_resume is None:
            try:
                pause_resume = self.printer.lookup_object("pause_resume")
                self._pause_resume_obj = pause_resume
            except Exception:
                pause_resume = None

        if pause_resume:
            try:
                is_paused = bool(getattr(pause_resume, "is_paused", False))
                if is_paused:
                    self.logger.debug("Skipping resume processing - printer is still paused (likely TOOL_UNLOAD operation)")
                    return
            except Exception:
                pass  # If we can't determine pause state, proceed with resume logic

        # Check if monitors were stopped and need to be restarted
        if not self.monitor_timers:
            self.logger.info("Restarting monitors after pause/intervention")

            self.start_monitors()

        now = self.reactor.monotonic()

        # Clear any error LEDs on resume (error flags already cleared when pause was triggered)
        for fps_name, fps_state in self.current_state.fps_state.items():
            oams = self._get_oams_object(fps_state.current_oams) if fps_state.current_oams else None

            # Clear stuck_spool_active on resume to allow follower to restart
            if fps_state.stuck_spool.active:
                fps_state.reset_stuck_spool_state(preserve_restore=True)
                self.logger.info(f"Cleared stuck spool state for {fps_name} on print resume")
                # Clear the error LED if we have an OAMS and spool index
                if oams is not None and fps_state.current_spool_idx is not None:
                    self._clear_error_led(oams, fps_state.current_spool_idx, fps_name, "print resume (stuck spool)")

            # Clear clog_active on resume and reset tracker
            if fps_state.clog.active:
                fps_state.reset_clog_tracker()
                self.logger.info(f"Cleared clog state for {fps_name} on print resume")
                if fps_state.current_lane is not None:
                    fps_state.engaged_with_extruder = True
                    fps_state.engagement_checked_at = now
                # Clear the error LED if we have an OAMS and spool index
                if oams is not None and fps_state.current_spool_idx is not None:
                    self._clear_error_led(oams, fps_state.current_spool_idx, fps_name, "print resume (clog)")

            if fps_state.stuck_spool.restore_follower:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "print resume")
            elif fps_state.current_oams is not None and fps_state.current_spool_idx is not None:
                if not fps_state.following or fps_state.direction != 1:
                    self._ensure_forward_follower(fps_name, fps_state, "print resume")


        # Update all followers based on hub sensors
        # Simple: if hub has filament, enable follower; if all empty, disable
        self._ensure_followers_for_loaded_hubs()

    def _trigger_stuck_spool_retry_gcode(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], message: str) -> None:
        """
        Trigger automatic unload + retry for stuck spool detected BEFORE engagement.

        Uses G-code commands instead of direct Python calls to properly queue commands
        through Klipper's G-code system without blocking the event loop.

        Args:
            fps_name: Name of the FPS where stuck spool was detected
            fps_state: Current state of the FPS
            oams: OAMS object (can be None, will be looked up)
            message: Error message describing the stuck spool condition
        """
        spool_idx = fps_state.current_spool_idx
        oams_name = fps_state.current_oams
        lane_name = fps_state.current_lane

        if oams is None and oams_name is not None:
            oams = self.oams.get(oams_name)

        if oams is None:
            self.logger.error(f"Cannot retry stuck spool on {fps_name} - OAMS not found")
            return

        oams_name = oams_name or getattr(oams, "name", oams_name)
        if not oams_name:
            self.logger.error(f"Cannot retry stuck spool on {fps_name} - OAMS name unavailable")
            return

        if spool_idx is None:
            self.logger.error(f"Cannot retry stuck spool on {fps_name} - spool index unknown")
            return

        if lane_name is None:
            self.logger.error(f"Cannot retry stuck spool on {fps_name} - lane name unknown")
            return

        self.logger.info(f"Starting stuck spool G-code retry sequence for {fps_name} lane {lane_name} spool {spool_idx}")

        # Get G-code object for running commands
        try:
            gcode = self.printer.lookup_object("gcode")
        except Exception as e:
            self.logger.error(f"Cannot get gcode object for retry: {e}")
            return

        # Extract FPS parameter (remove "fps " prefix if present)
        # fps_name is like "fps fps1", but G-code commands need just "fps1"
        fps_param = fps_name.replace("fps ", "", 1)

        # Execute stuck spool retry sequence with reactor blocking
        # CRITICAL: Use reactor.pause() between commands to prevent MCU overwhelm
        # The OAMS MCU needs time to process each command before receiving the next one
        try:
            self.logger.info(f"Starting blocked retry sequence for {fps_name} lane {lane_name}")

            # Step 1: Retract extruder to relieve pressure
            self.logger.debug(f"{fps_name}: Retracting extruder")
            gcode.run_script_from_command("SAVE_GCODE_STATE NAME=oams_blocked_retry")
            try:
                gcode.run_script_from_command("G91")  # Relative positioning
                gcode.run_script_from_command("M83")  # Relative extrusion mode
                gcode.run_script_from_command("G1 E-10 F300")
                gcode.run_script_from_command("M400")  # Wait for moves to finish
            finally:
                gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=oams_blocked_retry")
            self.reactor.pause(self.reactor.monotonic() + 0.5)

            # Step 2: Ensure follower is reverse for unload
            self.logger.debug(f"{fps_name}: Setting follower to reverse")
            self._set_follower_if_changed(
                oams_name,
                oams,
                1,
                0,
                "stuck spool retry recovery",
                force=True,
            )
            fps_state.following = True
            fps_state.direction = 0
            self.reactor.pause(self.reactor.monotonic() + 0.5)

            # Step 3: Unload the stuck filament
            self.logger.debug(f"{fps_name}: Unloading stuck filament")
            gcode.run_script_from_command(f"OAMSM_UNLOAD_FILAMENT FPS={fps_param}")
            gcode.run_script_from_command("M400")
            self.reactor.pause(self.reactor.monotonic() + 2.0)  # Longer wait for unload

            # Step 4: Ensure follower is forward for load
            self.logger.debug(f"{fps_name}: Setting follower to forward")
            self._set_follower_if_changed(
                oams_name,
                oams,
                1,
                1,
                "stuck spool retry recovery",
                force=True,
            )
            fps_state.following = True
            fps_state.direction = 1
            self.reactor.pause(self.reactor.monotonic() + 0.5)

            # Step 5: Retry load
            self.logger.debug(f"{fps_name}: Retrying load")
            gcode.run_script_from_command(f"OAMSM_LOAD_FILAMENT FPS={fps_param} LANE={lane_name}")
            gcode.run_script_from_command("M400")

            self.logger.info(f"Completed blocked retry sequence for {fps_name}")
        except Exception as e:
            self.logger.error(f"Failed to execute stuck spool retry for {fps_name}: {e}")
            # Clear stuck flag so detection can retrigger
            fps_state.stuck_spool.active = False

    def _trigger_stuck_spool_pause(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], message: str) -> None:
        if fps_state.stuck_spool.active:
            return

        # Set active flag BEFORE pause to prevent retriggering if pause fails or is slow
        fps_state.stuck_spool.active = True

        spool_idx = fps_state.current_spool_idx
        if oams is None and fps_state.current_oams is not None:
            oams = self._get_oams_object(fps_state.current_oams)

        # Set LED to red to indicate error
        if oams is not None and spool_idx is not None:
            try:
                oams.set_led_error(spool_idx, 1)
                # Note: Removed reactor.pause() - cannot block in timer callback during printing
            except Exception:
                self.logger.error(f"Failed to set stuck spool LED on {fps_name} spool {spool_idx}")

        # Abort current action (unload/load)
        if oams is not None:
            try:
                oams.abort_current_action(wait=False)
            except Exception:
                self.logger.error(f"Failed to abort current action on {fps_name} during stuck spool pause")

        # Pause printer with error message
        # CRITICAL: Do NOT notify AFC during stuck spool pause - AFC will try to unload which fails
        # during printing. Just pause and let user manually fix the stuck spool.
        # SAFETY: Wrap pause in try/except to prevent crash if pause logic fails
        try:
            self._pause_printer_message(message, fps_state.current_oams, notify_afc=False)
            # Note: Removed reactor.pause() - cannot block in timer callback during printing
        except Exception:
            self.logger.error(f"Failed to pause printer during stuck spool on {fps_name} - continuing with error state")
            # If pause failed, keep active=True to prevent retriggering until user intervention
            return

        # Keep the follower running during stuck spool pauses so manual extrusion
        # remains available without requiring OAMSM_CLEAR_ERRORS.
        # Enable follower directly during stuck spool pause, bypassing state checks
        # NOTE: Unlike clog detection, stuck spool can happen during LOADING before follower is enabled
        if oams is not None and spool_idx is not None:
            # ALWAYS use forward direction (1) during error states so user can manually extrude to fix the issue
            # Save the current direction for restore on RESUME
            current_direction = fps_state.direction if fps_state.following else 1
            fps_state.stuck_spool.restore_follower = True
            fps_state.stuck_spool.restore_direction = current_direction

            self._enable_follower(
                fps_name,
                fps_state,
                oams,
                1,  # Always forward during stuck spool so user can manually extrude
                "stuck spool pause - keep follower forward for manual recovery",
            )
            self.logger.info(f"Follower enabled on {fps_name} during stuck spool pause")

        self.logger.info(f"Stuck spool pause triggered for {fps_name} (LED stays red, active flag set, follower enabled)")
    def _unified_monitor_for_fps(self, fps_name):
        """Consolidated monitor handling all FPS checks in a single timer (OPTIMIZED)."""
        def _unified_monitor(self, eventtime):
            try:
                fps_state = self.current_state.fps_state.get(fps_name)
                fps = self.fpss.get(fps_name)

                if fps_state is None or fps is None:
                    return eventtime + MONITOR_ENCODER_PERIOD_IDLE

                oams = self._get_oams_object(fps_state.current_oams) if fps_state.current_oams else None

                # OPTIMIZATION: Use cached idle_timeout object
                is_printing = False
                if self._idle_timeout_obj is not None:
                    try:
                        is_printing = self._idle_timeout_obj.get_status(eventtime)["state"] == "Printing"
                    except Exception:
                        is_printing = False

                # OPTIMIZATION: Skip sensor reads if idle and no state changes
                state = fps_state.state
                if not is_printing and state == FPSLoadState.LOADED:
                    fps_state.consecutive_idle_polls += 1
                    if fps_state.consecutive_idle_polls > IDLE_POLL_THRESHOLD:
                        # Exponential backoff for idle polling
                        if fps_state.consecutive_idle_polls % 5 == 0:
                            fps_state.idle_backoff_level = min(fps_state.idle_backoff_level + 1, 3)
                        backoff_multiplier = 2 ** fps_state.idle_backoff_level
                        return eventtime + (MONITOR_ENCODER_PERIOD_IDLE * backoff_multiplier)

                # OPTIMIZATION: Skip sensor reads in UNLOADED state (only needed every 2 polls for auto-detect)
                # This reduces MCU communication by 50% when UNLOADED while keeping detection fast (4 seconds)
                skip_sensor_read = (state == FPSLoadState.UNLOADED and fps_state.consecutive_idle_polls % 2 != 0)

                # Get OAMS name for cache lookup
                oams_name = fps_state.current_oams or getattr(oams, "name", None)

                # Read sensors from AMSHardwareService cache (skip if UNLOADED and not on 2-poll boundary)
                # This consolidates sensor polling through a single source, reducing duplicate MCU reads
                encoder_value = None
                pressure = None
                hes_values = None

                if not skip_sensor_read:
                    if not oams_name and not oams:
                        return eventtime + MONITOR_ENCODER_PERIOD_IDLE
                    try:
                        # Use cached sensor data from AMSHardwareService when available
                        if oams_name:
                            cached_data = self._get_cached_sensor_data(oams_name, oams)
                            encoder_value = cached_data.get("encoder_clicks")
                            hes_values = cached_data.get("hub_hes_value")
                            # Get FPS pressure - prefer cache, fall back to direct
                            pressure = self._get_cached_fps_value(fps, oams_name)
                            if pressure is None:
                                pressure = 0.0
                        elif oams:
                            # Fallback to direct read if no oams_name
                            encoder_value = oams.encoder_clicks
                            pressure = float(getattr(fps, "fps_value", 0.0))
                            hes_values = oams.hub_hes_value
                        else:
                            return eventtime + MONITOR_ENCODER_PERIOD_IDLE
                    except Exception:
                        self.logger.error(f"Failed to read sensors for {fps_name}")
                        return eventtime + MONITOR_ENCODER_PERIOD_IDLE

                monitor = self.runout_monitors.get(fps_name)
                is_runout_active = monitor and monitor.state != OAMSRunoutState.MONITORING
                allow_f1s_updates = not (
                    monitor and monitor.state in (OAMSRunoutState.DETECTED, OAMSRunoutState.COASTING)
                )
                allow_lane_clear = (
                    not is_runout_active
                    and fps_state.state not in (FPSLoadState.LOADING, FPSLoadState.UNLOADING)
                )
                if oams_name:
                    self._sync_openams_sensors_for_oams(
                        oams_name,
                        eventtime,
                        allow_f1s_updates=allow_f1s_updates,
                        allow_lane_clear=allow_lane_clear,
                    )

                now = self.reactor.monotonic()
                state_changed = False

                # SAFETY: Check fps_state.since is not None before subtraction to prevent crash
                # Also skip if encoder_value is None (cache not yet populated)
                if state == FPSLoadState.UNLOADING and fps_state.since is not None and encoder_value is not None and now - fps_state.since > MONITOR_ENCODER_SPEED_GRACE:
                    self._check_unload_speed(fps_name, fps_state, oams, encoder_value, now)
                    state_changed = True
                elif state == FPSLoadState.LOADING and fps_state.since is not None and encoder_value is not None and now - fps_state.since > MONITOR_ENCODER_SPEED_GRACE:
                    self._check_load_speed(fps_name, fps_state, fps, oams, encoder_value, pressure, now)
                    state_changed = True
                elif state == FPSLoadState.UNLOADED:
                    # When UNLOADED, periodically check if filament was newly inserted
                    # AFC updates lane_loaded when filament is detected, so we need to check determine_state()
                    # to pick up the new lane_loaded and update current_spool_idx
                    # Skip auto-detect if:
                    # - Active runout in progress (avoid interference)
                    # - Tool change in progress (hub sensor hasn't cleared yet during unload)
                    is_tool_operation = False
                    try:
                        afc = self._get_afc()
                        is_tool_operation = getattr(afc, 'in_toolchange', False) if afc else False
                    except Exception:
                        pass

                    recently_unloaded = (
                        fps_state.since is not None
                        and now - fps_state.since < AUTO_DETECT_INSERT_GRACE
                    )
                    if (
                        not is_runout_active
                        and not is_tool_operation
                        and not recently_unloaded
                        and fps_state.consecutive_idle_polls % 2 == 0
                    ):  # Check every 2 polls (4 seconds)
                        old_lane = fps_state.current_lane
                        old_spool_idx = fps_state.current_spool_idx
                        (
                            fps_state.current_lane,
                            current_oams,
                            fps_state.current_spool_idx,
                        ) = self.determine_current_loaded_lane(fps_name)

                        if current_oams is not None:
                            fps_state.current_oams = current_oams.name

                        # If lane was newly detected, transition to LOADED
                        if fps_state.current_lane and fps_state.current_spool_idx is not None:
                            fps_state.state = FPSLoadState.LOADED
                            fps_state.since = now
                            fps_state.reset_stuck_spool_state()
                            fps_state.reset_clog_tracker()
                            self._ensure_forward_follower(fps_name, fps_state, "auto-detect new filament")

                            self.logger.info(f"Auto-detected newly inserted filament: {fps_state.current_lane} (spool {fps_state.current_spool_idx})")

                            state_changed = True
                elif state == FPSLoadState.LOADED:
                    if is_printing:
                        # CRITICAL: Skip stuck spool/clog detection during runout states FIRST
                        # This must be checked BEFORE active extruder check because during
                        # cross-extruder runouts, the old lane may no longer be "active" but
                        # still needs protection while COASTING/DETECTED
                        monitor = self.runout_monitors.get(fps_name)
                        in_runout_state = monitor and monitor.state in (OAMSRunoutState.DETECTED, OAMSRunoutState.COASTING)

                        if not in_runout_state:
                            # Only run stuck spool/clog detection for the ACTIVE extruder
                            # Otherwise we trigger false positives on inactive lanes
                            is_active_extruder = False
                            try:
                                afc = self._get_afc()
                                if afc and hasattr(afc, 'function'):
                                    current_extruder = afc.function.get_current_extruder()
                                    if current_extruder and hasattr(afc, 'tools'):
                                        extruder_obj = afc.tools.get(current_extruder)
                                        if extruder_obj:
                                            loaded_lane = getattr(extruder_obj, 'lane_loaded', None)
                                            is_active_extruder = (loaded_lane == fps_state.current_lane)
                            except Exception:
                                # If we can't determine, assume active (safer to check than skip)
                                is_active_extruder = True

                            if is_active_extruder:
                                # Call stuck spool check with encoder value to prevent false positives
                                # Requires BOTH low pressure AND encoder stopped (not just pressure alone)
                                # Skip detection if sensor values are None (cache not yet populated)
                                if encoder_value is not None and pressure is not None:
                                    self._check_stuck_spool(fps_name, fps_state, fps, oams, encoder_value, pressure, hes_values, now)
                                    self._check_clog(fps_name, fps_state, fps, oams, encoder_value, pressure, now)

                        state_changed = True
                # Update follower only when state changes or periodically during idle
                # No need to run every cycle since we never auto-disable anymore
                # Follower gets enabled when filament detected, stays enabled until manual disable
                if oams and fps_state.current_oams:
                    # Run on state changes or every 10th idle poll to catch new filament insertions
                    if state_changed or fps_state.consecutive_idle_polls % 10 == 0:
                        self._update_follower_for_oams(fps_state.current_oams, oams)

                # OPTIMIZATION: Adaptive polling interval with exponential backoff
                if state_changed or is_printing:
                    fps_state.consecutive_idle_polls = 0
                    fps_state.idle_backoff_level = 0
                    fps_state.last_state_change = now
                    return eventtime + MONITOR_ENCODER_PERIOD

                fps_state.consecutive_idle_polls += 1
                if fps_state.consecutive_idle_polls > IDLE_POLL_THRESHOLD:
                    # Exponential backoff: increase backoff level every 5 idle polls
                    if fps_state.consecutive_idle_polls % 5 == 0:
                        fps_state.idle_backoff_level = min(fps_state.idle_backoff_level + 1, 3)

                    # Calculate backoff multiplier (1x, 2x, 4x, 8x)
                    backoff_multiplier = 2 ** fps_state.idle_backoff_level
                    interval = MONITOR_ENCODER_PERIOD_IDLE * backoff_multiplier
                    return eventtime + interval

                return eventtime + MONITOR_ENCODER_PERIOD
            except Exception as e:
                self.logger.error(
                    f"Monitor loop crashed for {fps_name}; retrying after backoff: {e}",
                    traceback=traceback.format_exc(),
                )
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

        return partial(_unified_monitor, self)

    def _check_unload_speed(self, fps_name, fps_state, oams, encoder_value, now):
        """Detect stalled unloads from encoder movement during active prints."""
        # Check if stuck spool detection is disabled in config
        if not self.enable_stuck_spool_detection:
            return

        # Skip check if already handling a stuck spool
        if fps_state.stuck_spool.active:
            return

        # Only treat slow unloads as stuck during active prints. Standby/manual unloads
        # can legitimately pause movement, so skip the detection entirely when not printing.
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        if not is_printing:
            fps_state.clear_encoder_samples()
            return

        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        if encoder_diff is None:
            return

        self.logger.debug(f"OAMS[{getattr(oams, 'oams_idx', -1)}] Unload Monitor: Encoder diff {encoder_diff}")

        if encoder_diff < MIN_ENCODER_DIFF:
            lane_label = fps_state.current_lane or fps_name
            spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"

            # Abort the current unload operation cleanly
            try:
                oams.abort_current_action(wait=False)
                self.logger.info(f"Aborted stuck spool unload operation on {fps_name}")
                # NOTE: Cannot use reactor.pause() in timer callback - it doesn't work properly
            except Exception:
                self.logger.error(f"Failed to abort unload operation on {fps_name}")

            # Set LED error using state-tracked helper
            if fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 1, "unload stuck detected")


            # CRITICAL: Keep follower enabled even during stuck unload
            # User needs follower running to manually fix issues or re-attempt unload
            # Follower doesn't interfere with stuck detection (encoder based)
            if fps_state.current_oams:
                self._ensure_forward_follower(fps_name, fps_state, "stuck unload - keep follower active")


            # Transition to LOADED state cleanly (unload failed during print, so still loaded)
            fps_state.state = FPSLoadState.LOADED
            fps_state.clear_encoder_samples()

            # Set the stuck flag to trigger pause
            fps_state.stuck_spool.active = True
            fps_state.stuck_spool.start_time = None

            # NOTE: Do NOT call retry sequence from this timer callback context!
            # Timer callbacks cannot use reactor.pause() properly - it causes commands
            # to fire all at once without waiting for MCU, overwhelming it.
            # For unload during print, this will trigger a pause for user intervention.
            self.logger.info(f"Spool appears stuck while unloading {lane_label} spool {spool_label} - pausing for user intervention")
    def _check_load_speed(self, fps_name, fps_state, fps, oams, encoder_value, pressure, now):
        """Detect stalled loads by combining encoder deltas with FPS pressure feedback."""
        # Check if stuck spool detection is disabled in config
        if not self.enable_stuck_spool_detection:
            return

        if fps_state.stuck_spool.active:
            return

        # Skip check if we don't have a valid since timestamp
        if fps_state.since is None:
            return

        # Skip check if we're still in grace period
        if now - fps_state.since <= MONITOR_ENCODER_SPEED_GRACE:
            return

        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        if encoder_diff is None:
            return

        self.logger.debug(f"OAMS[{getattr(oams, 'oams_idx', -1)}] Load Monitor: Encoder diff {encoder_diff}, FPS pressure {pressure:.2f}")

        # Track if pressure has dropped during load - proves filament is moving
        if pressure < self.load_fps_stuck_threshold and not fps_state.load_pressure_dropped:
            fps_state.load_pressure_dropped = True
            self.logger.debug(f"FPS pressure dropped to {pressure:.2f} during load - filament moving through buffer")

        # Suppress stuck detection DURING engagement verification
        # High FPS pressure during engagement extrusion is NORMAL and expected
        if fps_state.engagement_in_progress:
            self.logger.debug("Suppressing stuck detection during engagement verification (high pressure is normal)")
            return

        # Suppress stuck detection after successful engagement verification
        # Prevents false triggers when filament is actually loaded correctly
        if fps_state.engaged_with_extruder and fps_state.engagement_checked_at is not None:
            time_since_engagement = now - fps_state.engagement_checked_at
            if time_since_engagement < ENGAGEMENT_SUPPRESSION_WINDOW:
                self.logger.debug(f"Suppressing stuck detection for {ENGAGEMENT_SUPPRESSION_WINDOW - time_since_engagement:.1f}s after successful engagement")
                return

        # Check for stuck spool conditions:
        # Both conditions require encoder not moving + high pressure
        # If encoder IS moving, filament is flowing regardless of pressure spikes
        stuck_detected = False
        stuck_reason = ""

        if encoder_diff < MIN_ENCODER_DIFF:
            # Encoder not moving during load = stuck spool (follower not tracking)
            # Pressure level tells us WHY it's stuck, but it's stuck regardless
            stuck_detected = True
            if not fps_state.load_pressure_dropped:
                # Pressure never dropped - spool never engaged buffer
                stuck_reason = "encoder not moving and pressure never dropped"
            elif pressure >= self.load_fps_stuck_threshold:
                # Pressure high - filament not flowing through buffer
                stuck_reason = f"encoder not moving and FPS pressure {pressure:.2f} >= {self.load_fps_stuck_threshold:.2f}"
            else:
                # Pressure low but encoder still not moving - spool not feeding filament
                stuck_reason = f"encoder not moving (only {encoder_diff} clicks) and FPS pressure low ({pressure:.2f})"
        else:
            # Encoder IS moving - filament is flowing, not stuck
            # Pressure spikes during engagement extrusion are normal when filament is moving
            if pressure >= self.load_fps_stuck_threshold:
                self.logger.debug(f"High pressure ({pressure:.2f}) but encoder moving ({encoder_diff}) - filament flowing correctly")

        if stuck_detected:
            lane_label = fps_state.current_lane or fps_name
            spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"

            # User requirement: Stuck detection should trigger after couple seconds, not full timeout
            # CRITICAL: Use wait=False because this runs in timer callback context
            # Timer callbacks cannot use reactor.pause() properly (causes Klipper crashes)
            # wait=False queues the abort without blocking - MCU will complete asynchronously
            # This makes the load fail after ~3-5 seconds instead of waiting 30 seconds for timeout
            try:
                self.logger.info(f"Detected stuck spool on {fps_name}: {stuck_reason}")
                if fps_state.current_oams and oams is not None:
                    oams.abort_current_action(wait=False)
                    self.logger.info(f"Requested abort for stuck load operation on {fps_name}")
            except Exception:
                self.logger.error(f"Failed to abort load operation on {fps_name}")

            # Set LED error using state-tracked helper
            if fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 1, "load stuck detected")


            # Transition to UNLOADED state cleanly
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.clear_encoder_samples()

            # Set the stuck flag but DON'T pause - let the OAMS retry logic handle it
            # The retry logic will clear this flag if the retry succeeds
            fps_state.stuck_spool.active = True
            fps_state.stuck_spool.start_time = None

            # CRITICAL: Keep follower enabled even during stuck load
            # User needs follower running to manually fix clogs or re-attempt load
            # Follower doesn't interfere with stuck detection (encoder + FPS based)
            # Enable follower directly during stuck load, bypassing state checks
            if fps_state.current_oams and fps_state.current_spool_idx is not None:
                oams_obj = self._get_oams_object(fps_state.current_oams)
                if oams_obj is not None:
                    self._enable_follower(
                        fps_name,
                        fps_state,
                        oams_obj,
                        1,
                        "stuck load - keep follower active",
                    )

            # NOTE: Do NOT call retry sequence from this timer callback context!
            # Timer callbacks cannot use reactor.pause() properly - it causes commands
            # to fire all at once without waiting for MCU, overwhelming it.
            # The retry logic in the main load loop (lines 3628-3668) will handle retries
            # properly in a command context where reactor blocking works correctly.
            self.logger.info(f"Spool appears stuck while loading {lane_label} spool {spool_label} ({stuck_reason}) - load will fail and retry in main loop")

    def _check_stuck_spool(self, fps_name, fps_state, fps, oams, encoder_value, pressure, hes_values, now):
        """Check for stuck spool conditions during printing.

        Requires BOTH conditions to trigger:
        1. FPS pressure low (< threshold) for sustained time
        2. Encoder not moving (< MIN_ENCODER_DIFF clicks)

        This prevents false positives during print stalls where pressure may fluctuate
        but encoder naturally stops due to toolhead buffer underrun.
        """
        # Check if stuck spool detection is disabled in config
        if not self.enable_stuck_spool_detection:
            return

        # Skip stuck spool detection if clog is active
        # Clog detection handles follower control during clog conditions
        if fps_state.clog.active:
            if fps_state.stuck_spool.active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "clog active - deferred to clog handling")

            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool.restore_follower)
            return

        # OPTIMIZATION: Use cached idle_timeout object
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        # Skip stuck spool detection if ANY runout is active
        # During runout recovery (same-FPS or cross-FPS), we're actively swapping lanes
        # and should not interfere with follower control until the swap completes
        monitor = self.runout_monitors.get(fps_name)
        if monitor is not None and monitor.state not in (OAMSRunoutState.MONITORING, OAMSRunoutState.STOPPED):
            if fps_state.stuck_spool.active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, f"runout active on this FPS ({monitor.state})")

            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool.restore_follower)
            return

        # Also check if ANY other FPS has an active runout (for cross-FPS scenarios)
        for other_fps_name, other_monitor in self.runout_monitors.items():
            if other_fps_name != fps_name and other_monitor.state not in (OAMSRunoutState.MONITORING, OAMSRunoutState.STOPPED):
                if fps_state.stuck_spool.active and oams is not None and fps_state.current_spool_idx is not None:
                    self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, f"runout active on {other_fps_name} ({other_monitor.state})")

                fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool.restore_follower)
                return

        pause_resume = self._pause_resume_obj
        if pause_resume is None:
            try:
                pause_resume = self.printer.lookup_object("pause_resume")

                self._pause_resume_obj = pause_resume
            except Exception:
                pause_resume = False

        is_paused = False
        if pause_resume:
            try:
                is_paused = bool(getattr(pause_resume, "is_paused", False))
            except Exception:
                is_paused = False

        if not is_printing or is_paused:
            if fps_state.stuck_spool.active and oams is not None and fps_state.current_spool_idx is not None:
                reason = "printer idle" if not is_paused else "printer paused"
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, reason)
            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool.restore_follower)
            return

        if (
            is_printing
            and fps_state.state == FPSLoadState.LOADED
            and fps_state.current_lane is not None
            and not fps_state.engaged_with_extruder
        ):
            fps_state.engaged_with_extruder = True
            fps_state.engagement_checked_at = now

        # Skip stuck spool detection if AFC bypass is enabled
        # User is manually controlling filament, FPS pressure will be abnormal
        try:
            afc = self._get_afc()
            if afc is not None and hasattr(afc, '_get_bypass_state'):
                if afc._get_bypass_state():
                    if fps_state.stuck_spool.active and oams is not None and fps_state.current_spool_idx is not None:
                        self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "AFC bypass enabled")

                    fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool.restore_follower)
                    self.logger.debug(f"Skipping stuck spool detection on {fps_name} - AFC bypass enabled")
                    return
        except Exception:
            # Don't crash if bypass check fails, just continue with detection
            pass

        if fps_state.since is not None and now - fps_state.since < self.stuck_spool_load_grace:
            fps_state.stuck_spool.start_time = None
            # Clear stuck spool flag during grace period after successful load
            if fps_state.stuck_spool.active:
                if oams is not None and fps_state.current_spool_idx is not None:
                    self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "grace period")

                fps_state.reset_stuck_spool_state(preserve_restore=True)
            return

        # Extended grace period for lane transitions during runout recovery
        # Same-FPS runouts don't pause - new filament loads while print continues
        # Give 30 seconds for new filament to position and develop back-pressure
        if fps_state.last_lane_change_time is not None and now - fps_state.last_lane_change_time < 30.0:
            fps_state.stuck_spool.start_time = None
            if fps_state.stuck_spool.active:
                if oams is not None and fps_state.current_spool_idx is not None:
                    elapsed = now - fps_state.last_lane_change_time
                    self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, f"lane transition grace ({elapsed:.1f}s)")

                fps_state.reset_stuck_spool_state(preserve_restore=True)
            return

        # Skip stuck spool detection if NO lane is synced to extruder/toolhead
        # This prevents false positives when filament is only in the hub but not loaded to toolhead
        # (e.g., after manual unload from toolhead but filament still in AMS)
        # CRITICAL: Also skip if THIS fps_state's lane is NOT the one actively loaded to toolhead
        # This prevents monitoring inactive lanes (e.g., when changing spools on a different lane)
        if fps_state.current_lane is not None:
            try:
                afc = self._get_afc()
                if afc is not None:
                    extruder_obj = getattr(afc, 'extruder', None)
                    if extruder_obj is not None:
                        lane_loaded = getattr(extruder_obj, 'lane_loaded', None)
                        if lane_loaded is None:
                            # No lane synced to extruder but filament detected in hub - skip stuck detection
                            # This is the specific case of manual unload from toolhead with filament still in AMS
                            fps_state.stuck_spool.start_time = None
                            if fps_state.stuck_spool.active and oams is not None and fps_state.current_spool_idx is not None:
                                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "no lane synced to extruder")

                            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool.restore_follower)
                            return
                        elif lane_loaded != fps_state.current_lane:
                            # CRITICAL: This lane is NOT the one loaded to extruder - skip detection
                            # Prevents monitoring inactive lanes during spool changes
                            fps_state.stuck_spool.start_time = None
                            if fps_state.stuck_spool.active and oams is not None and fps_state.current_spool_idx is not None:
                                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, f"different lane loaded to extruder ({lane_loaded})")

                            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool.restore_follower)
                            return
            except Exception:
                # If we can't determine sync state, proceed with detection to avoid masking real issues
                pass

        if not fps_state.following or fps_state.direction != 1:
            fps_state.stuck_spool.start_time = None
            # Auto-enable follower if we have a spool loaded but follower is disabled
            if is_printing and oams is not None and not fps_state.following:
                self._ensure_forward_follower(fps_name, fps_state, "auto-enable after manual load")

            elif fps_state.stuck_spool.restore_follower and is_printing and oams is not None:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")

            return

        # Check encoder movement to differentiate stuck spool from normal print pause
        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        encoder_moving = encoder_diff is not None and encoder_diff >= MIN_ENCODER_DIFF

        # =======================================================================================
        # HYSTERESIS STATE MACHINE: Stuck Spool Detection
        # =======================================================================================
        # Uses dual-threshold hysteresis to prevent flapping between stuck/unstuck states:
        #
        # State Transitions:
        #   IDLE ? TIMER_RUNNING: pressure <= stuck_spool_pressure_threshold (0.08) AND encoder stopped
        #   TIMER_RUNNING ? ACTIVE: timer exceeds STUCK_SPOOL_DWELL (5.0s)
        #   ACTIVE ? IDLE: pressure >= stuck_spool_pressure_clear_threshold (0.12) OR encoder moving
        #   TIMER_RUNNING ? IDLE: encoder moving OR pressure restored
        #
        # Hysteresis prevents oscillation:
        #   - Lower threshold (0.08) to START detection
        #   - Upper threshold (0.12) to CLEAR detection
        #   - Gap of 0.04 provides ~50% margin to prevent rapid state changes
        #
        # Why Both Conditions Required:
        #   During normal printing pauses/retracts, pressure may drop but encoder naturally stops.
        #   Requiring BOTH conditions prevents false positives from these normal printer states.
        # =======================================================================================

        # Hysteresis logic: Use lower threshold to start timer, upper threshold to clear
        # CRITICAL: Require BOTH low pressure AND encoder stopped to prevent false positives
        if pressure <= self.stuck_spool_pressure_threshold and not encoder_moving:
            # Pressure is low AND encoder not moving - start or continue stuck spool timer
            if fps_state.stuck_spool.start_time is None:
                fps_state.stuck_spool.start_time = now
                self.logger.info(
                    f"{fps_name}: STUCK SPOOL TIMER STARTED - pressure {pressure:.2f} <= {self.stuck_spool_pressure_threshold:.2f}, "
                    f"encoder_diff={encoder_diff}, threshold={MIN_ENCODER_DIFF}, dwell={STUCK_SPOOL_DWELL}s"
                )
            elif not fps_state.stuck_spool.active:
                # Timer is running - show countdown
                elapsed = now - fps_state.stuck_spool.start_time
                remaining = STUCK_SPOOL_DWELL - elapsed
                if remaining > 0:
                    self.logger.debug(
                        f"{fps_name}: Stuck spool countdown - {remaining:.1f}s remaining "
                        f"(pressure={pressure:.2f}, encoder_diff={encoder_diff})"
                    )
                else:
                    # Detection triggered!
                    message = "Spool appears stuck"
                    if fps_state.current_lane is not None:
                        message = f"Spool appears stuck on {fps_state.current_lane} spool {fps_state.current_spool_idx}"

                    self.logger.info(
                        f"{fps_name}: STUCK SPOOL DETECTED! Conditions: pressure={pressure:.2f} (threshold={self.stuck_spool_pressure_threshold:.2f}), "
                        f"encoder_diff={encoder_diff} (threshold={MIN_ENCODER_DIFF}), dwell_time={elapsed:.1f}s (threshold={STUCK_SPOOL_DWELL}s)"
                    )

                    # CRITICAL: Check if filament has engaged with extruder yet
                    # If NOT engaged, this is a load failure during initial load
                    # If ENGAGED, this is a stuck spool during printing - pause for user intervention
                    if not fps_state.engaged_with_extruder:
                        # Pre-engagement stuck spool - filament failed to load properly
                        # NOTE: Cannot trigger retry from timer callback - just pause
                        self.logger.info(f"{fps_name}: Stuck spool detected BEFORE engagement - pausing")
                        fps_state.stuck_spool.active = True  # Set flag to prevent retriggering

                        # Pause for user intervention
                        try:
                            self._trigger_stuck_spool_pause(fps_name, fps_state, oams, message)
                        except Exception as e:
                            self.logger.error(f"Failed to trigger stuck spool pause for {fps_name}: {e}")
                    else:
                        # Post-engagement stuck spool - filament is loaded but spool is stuck/tangled
                        # This requires user intervention - pause the print
                        self.logger.info(f"{fps_name}: Stuck spool detected AFTER engagement - pausing for user intervention")

                        # SAFETY: Wrap pause trigger in try/except to prevent crash
                        try:
                            self._trigger_stuck_spool_pause(fps_name, fps_state, oams, message)
                        except Exception:
                            self.logger.error(f"Failed to trigger stuck spool pause for {fps_name} - error state may be inconsistent")
        elif encoder_moving:
            # Encoder is moving - clear timer even if pressure is low (print stall with active extrusion)
            if fps_state.stuck_spool.start_time is not None and not fps_state.stuck_spool.active:
                fps_state.stuck_spool.start_time = None
                self.logger.debug(f"{fps_name}: Stuck spool timer cleared - encoder moving ({encoder_diff} clicks)")
        elif pressure >= self.stuck_spool_pressure_clear_threshold:
            # Pressure is definitively high - clear stuck spool state
            if fps_state.stuck_spool.active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "pressure restored")


                # Clear the stuck_spool_active flag BEFORE trying to restore follower
                fps_state.reset_stuck_spool_state(preserve_restore=True)
                self.logger.info(f"Cleared stuck spool state for {fps_name}, pressure restored to {pressure:.2f}")

            # Also clear timer if it was running but not yet triggered
            if fps_state.stuck_spool.start_time is not None and not fps_state.stuck_spool.active:
                fps_state.stuck_spool.start_time = None

            # Now restore/enable follower
            # SAFETY: Wrap follower operations in try/except to prevent crash during recovery
            try:
                if fps_state.stuck_spool.restore_follower and is_printing:
                    self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")

                elif is_printing and not fps_state.following:
                    self._ensure_forward_follower(fps_name, fps_state, "stuck spool recovery")

            except Exception:
                self.logger.error(f"Failed to restore/enable follower during stuck spool recovery for {fps_name}")
        # else: Pressure is in hysteresis band (between thresholds) - maintain current state

    def _check_clog(self, fps_name, fps_state, fps, oams, encoder_value, pressure, now):
        """Check for clog conditions (OPTIMIZED).

        =======================================================================================
        CLOG DETECTION ALGORITHM
        =======================================================================================
        Detects filament path blockages by monitoring extruder motion vs encoder response.

        Detection Window:
          - Configurable per sensitivity level (low/medium/high)
          - Medium: 24mm extruder extrusion window
          - Tracks extruder movement and encoder clicks over this distance

        Detection Criteria (ALL must be true):
          1. Extruder moved >= extrusion_window mm (e.g., 24mm)
          2. Encoder clicks < encoder_slack threshold (e.g., 8 clicks)
          3. FPS pressure within normal band around target (0.50 +/- 0.06)
          4. Sustained for >= dwell time (e.g., 10.0s)

        Pressure Band Check:
          - Prevents false positives from stuck spool (pressure too low)
          - Prevents false positives from active extrusion (pressure too high)
          - Band width varies by sensitivity: low=0.08, medium=0.06, high=0.04
          - Center target: 0.50 (configurable via clog_pressure_target)

        Sensitivity Levels:
          low:    window=48mm, slack=15, band=+/-0.08, dwell=12s (fewest false positives)
          medium: window=24mm, slack=8,  band=+/-0.06, dwell=10s (balanced - default)
          high:   window=12mm, slack=4,  band=+/-0.04, dwell=8s  (earliest detection)

        Rate Limiting:
          - Checks throttled to CLOG_CHECK_INTERVAL (8.0s) when not active
          - Reduces CPU/log churn during normal printing
          - Active clogs checked every monitor cycle for recovery detection

        Suppression Windows (prevent false positives):
          - During engagement verification
          - During engagement retries
          - 5s after lane transitions (allows new filament to engage)
          - When AFC bypass enabled (manual control)
          - When runout monitoring inactive
          - When pressure too low (indicates stuck spool, not clog)
        =======================================================================================
        """
        # Check if clog detection is disabled in config
        if not self.enable_clog_detection:
            return

        # OPTIMIZATION: Use cached idle_timeout object
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        # Check if printer is paused (same logic as stuck spool detection)
        pause_resume = self._pause_resume_obj
        if pause_resume is None:
            try:
                pause_resume = self.printer.lookup_object("pause_resume")
                self._pause_resume_obj = pause_resume
            except Exception:
                pause_resume = False

        is_paused = False
        if pause_resume:
            try:
                is_paused = bool(getattr(pause_resume, "is_paused", False))
            except Exception:
                is_paused = False

        monitor = self.runout_monitors.get(fps_name)
        if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
            if fps_state.clog.active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "runout monitor inactive")

            fps_state.reset_clog_tracker()
            return

        # Skip clog detection when printer is paused (e.g., during manual TOOL_UNLOAD operations)
        # AFC operations like cutting and unloading cause extruder movement that would trigger false clogs
        if is_paused:
            if fps_state.clog.active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "printer paused")

            fps_state.reset_clog_tracker()
            return

        allow_clog_checks = is_printing or (
            fps_state.state == FPSLoadState.LOADING and fps_state.engaged_with_extruder
        )
        if not allow_clog_checks:
            if fps_state.clog.active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "printer idle")

            fps_state.reset_clog_tracker()
            return

        # Skip clog detection if AFC bypass is enabled
        # User is manually controlling filament, extruder/encoder relationship will be abnormal
        try:
            afc = self._get_afc()
            if afc is not None and hasattr(afc, '_get_bypass_state'):
                if afc._get_bypass_state():
                    if fps_state.clog.active and oams is not None and fps_state.current_spool_idx is not None:
                        self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "AFC bypass enabled")

                    fps_state.reset_clog_tracker()
                    self.logger.debug(f"Skipping clog detection on {fps_name} - AFC bypass enabled")
                    return
        except Exception:
            # Don't crash if bypass check fails, just continue with detection
            pass

        # CRITICAL: Skip clog detection if THIS lane is NOT the one actively loaded to toolhead
        # This prevents monitoring inactive lanes (e.g., when changing spools on a different lane)
        if fps_state.current_lane is not None:
            try:
                afc = self._get_afc()
                if afc is not None:
                    extruder_obj = getattr(afc, 'extruder', None)
                    if extruder_obj is not None:
                        lane_loaded = getattr(extruder_obj, 'lane_loaded', None)
                        if lane_loaded is not None and lane_loaded != fps_state.current_lane:
                            # This lane is NOT the one loaded to extruder - skip clog detection
                            if fps_state.clog.active and oams is not None and fps_state.current_spool_idx is not None:
                                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, f"different lane loaded to extruder ({lane_loaded})")
                            fps_state.reset_clog_tracker()
                            return
            except Exception:
                # If we can't determine sync state, proceed with detection to avoid masking real issues
                pass

        # Suppress clog detection during engagement verification to avoid false positives
        # while the extruder is deliberately driving filament for the check.
        if fps_state.engagement_in_progress:
            fps_state.reset_clog_tracker()
            return

        if fps_state.engagement_retry_active:
            fps_state.reset_clog_tracker()
            return

        # Suppress clog detection briefly after lane transitions (e.g., same-FPS runouts)
        # so the new filament has time to engage the extruder before we evaluate encoder motion.
        if fps_state.last_lane_change_time is not None and now - fps_state.last_lane_change_time < 5.0:
            fps_state.reset_clog_tracker()
            return

        if (
            not fps_state.clog.active
            and fps_state.clog.last_check_time is not None
            and now - fps_state.clog.last_check_time < CLOG_CHECK_INTERVAL
        ):
            return

        # Skip clog detection if FPS pressure is very low - indicates stuck spool, not clog
        # During lane loads, stuck spool should trigger retry logic, not clog pause
        # During normal printing, low pressure also indicates stuck spool (separate detection)
        if pressure <= self.stuck_spool_pressure_threshold:
            # Very low FPS pressure indicates stuck spool, not clog - skip clog detection
            fps_state.reset_clog_tracker()
            return

        # Allow clog detection during loading so post-engagement purges still flag true clogs,
        # but keep tracker reset when no lane is present.
        if fps_state.current_spool_idx is None:
            fps_state.reset_clog_tracker()
            return
        # During load purge: extruder advances + encoder doesn't move = genuine clog, detect it
        # Before purge starts: extruder not advancing = clog won't trigger (extrusion_delta < threshold)
        # The existing clog logic is already smart enough to handle this correctly

        fps_state.clog.last_check_time = now

        try:
            extruder_pos = float(getattr(fps.extruder, "last_position", 0.0))
        except Exception:
            self.logger.error(f"Failed to read extruder position while monitoring clogs on {fps_name}")
            return

        if fps_state.clog.start_extruder is None:
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        if extruder_pos < (fps_state.clog.last_extruder or extruder_pos):
            # Retraction detected - track retraction density to filter out fast detailed prints
            fps_state.clog.retraction_count += 1
            fps_state.clog.last_retraction_time = now

            # If we see very high retraction density, this is likely a detailed print
            # with lots of small moves/retracts, not a clog - reset tracker to prevent false positive
            # Threshold: 5+ retractions in 10 seconds = very active detailed printing
            if fps_state.clog.retraction_count >= 5 and fps_state.clog.start_time is not None:
                window_duration = now - fps_state.clog.start_time
                if window_duration < 10.0:  # 5+ retractions in 10 seconds = high density activity
                    self.logger.debug(
                        f"{fps_name}: Resetting clog tracker - high retraction density detected "
                        f"({fps_state.clog.retraction_count} retractions in {window_duration:.1f}s)"
                    )
                    fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
                    return

            # Normal retraction - just reset tracker
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        fps_state.clog.last_extruder = extruder_pos
        if fps_state.clog.min_pressure is None or pressure < fps_state.clog.min_pressure:
            fps_state.clog.min_pressure = pressure
        if fps_state.clog.max_pressure is None or pressure > fps_state.clog.max_pressure:
            fps_state.clog.max_pressure = pressure

        extrusion_delta = extruder_pos - (fps_state.clog.start_extruder or extruder_pos)
        encoder_delta = abs(encoder_value - (fps_state.clog.start_encoder or encoder_value))
        pressure_span = (fps_state.clog.max_pressure or pressure) - (fps_state.clog.min_pressure or pressure)

        settings = self.clog_settings
        if extrusion_delta < settings["extrusion_window"]:
            # Not enough extrusion yet to check for clog
            return

        # CRITICAL: Check if average pressure is within the target band
        # This prevents false positives when pressure is way above/below target
        # (e.g., during long retracts, z-hops, or stuck spool conditions)
        pressure_mid = (fps_state.clog.min_pressure + fps_state.clog.max_pressure) / 2.0
        pressure_deviation = abs(pressure_mid - self.clog_pressure_target)

        # If pressure is outside the target band, this is NOT a clog condition
        # Could be stuck spool (too low), active heavy extrusion (too high), or other anomaly
        if pressure_deviation > settings["pressure_band"]:
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        if (encoder_delta > settings["encoder_slack"] or pressure_span > settings["pressure_band"]):
            # Encoder is moving or pressure is varying - filament is flowing
            # If clog was previously active, clear it and ensure follower keeps up
            if fps_state.clog.active:
                self.logger.info(
                    f"{fps_name}: Clog cleared - encoder moving normally "
                    f"(delta={encoder_delta}, pressure_span={pressure_span:.2f})"
                )

                if oams is not None and fps_state.current_spool_idx is not None:
                    self._set_led_error_if_changed(
                        oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "clog auto-recovery"
                    )

                # Don't clear manual follower override on auto-recovery
                # Keep it set so follower stays enabled during manual intervention
                # User can run CLEAR_ERRORS to return to automatic control

                is_printing = False
                if self._idle_timeout_obj is not None:
                    try:
                        is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
                    except Exception:
                        is_printing = False
                if (
                    is_printing
                    and oams is not None
                    and fps_state.current_spool_idx is not None
                    and (not fps_state.following or fps_state.direction != 1)
                ):
                    self._ensure_forward_follower(fps_name, fps_state, "clog auto-recovery")

                fps_state.reset_clog_tracker()

            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        # Conditions met for clog - check dwell timer
        elapsed = now - (fps_state.clog.start_time or now)
        remaining = settings["dwell"] - elapsed

        if remaining > 0:
            # Timer is counting down
            return

        if not fps_state.clog.active:
            # Clog detection triggered!
            pressure_mid = (fps_state.clog.min_pressure + fps_state.clog.max_pressure) / 2.0

            # Comprehensive debug output showing all detection conditions
            self.logger.info(
                f"{fps_name}: CLOG DETECTED! Conditions met: "
                f"extrusion_delta={extrusion_delta:.1f}mm (window={settings['extrusion_window']}mm), "
                f"encoder_delta={encoder_delta} (slack={settings['encoder_slack']}), "
                f"pressure_span={pressure_span:.2f} (band={settings['pressure_band']}), "
                f"pressure_mid={pressure_mid:.2f} (target={self.clog_pressure_target:.2f}), "
                f"dwell_time={elapsed:.1f}s (threshold={settings['dwell']}s)"
            )

            # Set LED error first
            if oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 1, "clog detected")
                # Note: Removed reactor.pause() here - cannot block in timer callback during printing
                # The LED command is asynchronous and will be processed by MCU without blocking

            message = (
                f"Clog suspected on {fps_state.current_lane or fps_name}: "
                f"extruder advanced {extrusion_delta:.1f}mm while encoder moved {encoder_delta} counts "
                f"with FPS {pressure_mid:.2f} near {self.clog_pressure_target:.2f}"
            )

            fps_state.clog.active = True

            # Pause printer with error message
            # CRITICAL: Do NOT notify AFC during clog pause - AFC will try to unload which fails
            # during printing. Just pause and let user manually fix the clog.
            # SAFETY: Wrap pause in try/except to prevent crash if pause logic fails
            try:
                self._pause_printer_message(message, fps_state.current_oams, notify_afc=False)
                # Note: Removed reactor.pause() - cannot block in timer callback
                # System will settle naturally through event loop processing
            except Exception:
                self.logger.error(f"Failed to pause printer during clog on {fps_name} - continuing with error state")
                # Keep active=True to prevent retriggering until user intervention
                return
            # Ensure the lane is ready to accept commands after the pause.
            # If a clog is detected during a transient load/unload state, mark as LOADED
            # so unload/recovery commands are not rejected as "busy."
            if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.afc_delegation_active = False
                fps_state.afc_delegation_until = 0.0
                self.logger.info(
                    f"Reset FPS state to LOADED after clog pause on {fps_name} to allow recovery commands"
                )

            # Enable follower during clog pause so manual extrusion remains available for recovery
            # Similar to stuck spool detection, keep follower forward for user intervention
            if oams is not None and fps_state.current_spool_idx is not None:
                # Save current follower state for potential restore on RESUME
                current_direction = fps_state.direction if fps_state.following else 1
                fps_state.clog.restore_follower = fps_state.following
                fps_state.clog.restore_direction = current_direction

                # Enable follower forward during clog pause for manual recovery
                self._enable_follower(
                    fps_name,
                    fps_state,
                    oams,
                    1,  # Always forward during clog so user can manually extrude
                    "clog pause - keep follower forward for manual recovery",
                )
                self.logger.info(f"Follower enabled forward on {fps_name} during clog pause for manual recovery")

    def start_monitors(self):
        """Start all monitoring timers"""
        # Stop existing monitors first to prevent timer leaks or duplicate timers
        if self.monitor_timers or self.runout_monitors:
            self.stop_monitors()

        self.monitor_timers = []
        self.runout_monitors = {}
        reactor = self.printer.get_reactor()
        
        for fps_name in self.current_state.fps_state.keys():
            self.monitor_timers.append(
                reactor.register_timer(
                    self._unified_monitor_for_fps(fps_name), 
                    reactor.NOW
                )
            )

            def _reload_callback(fps_name=fps_name, fps_state=self.current_state.fps_state[fps_name]):
                monitor = self.runout_monitors.get(fps_name)
                source_lane_name = fps_state.current_lane
                active_oams = fps_state.current_oams

                # Handle cross-extruder runouts using box turtle infinite runout sequence
                # Box turtle sequence: PAUSE -> SAVE_POS -> CHANGE_TOOL -> SET_MAP -> LANE_UNLOAD -> RESTORE_POS -> RESUME
                if getattr(fps_state, 'is_cross_extruder_runout', False):
                    self.logger.info(f"OAMS: Handling cross-extruder runout for {fps_name} (using box turtle sequence)")

                    # Get target lane from AFC runout_lane configuration
                    afc = self._get_afc()
                    target_lane_name = None
                    source_lane = None

                    if afc and source_lane_name:
                        source_lane = afc.lanes.get(source_lane_name)
                        if source_lane:
                            target_lane_name = getattr(source_lane, 'runout_lane', None)
                            if target_lane_name:
                                self.logger.info(f"OAMS: Found runout_lane={target_lane_name} for lane {source_lane_name}")

                    if not target_lane_name:
                        self.logger.error(f"OAMS: No runout lane configured for cross-extruder runout on {source_lane_name or fps_name}")
                        fps_state.reset_runout_positions()
                        fps_state.is_cross_extruder_runout = False
                        self._pause_printer_message(f"No runout lane configured for {source_lane_name or fps_name}", active_oams)
                        if monitor:
                            monitor.paused()
                        return

                    # DON'T update hardware service snapshot for cross-extruder runouts
                    # Lane0 might share the same AMS bay as lane8, and updating the snapshot
                    # to "empty" would cause AFC to reject loading lane0 with "filament detected but not loaded"

                    # Execute cross-extruder runout sequence: Heat target, then use CHANGE_TOOL (like box turtle)
                    try:
                        self.logger.info(f"OAMS: Cross-extruder infinite runout: {source_lane_name} -> {target_lane_name}")
                        gcode = self.printer.lookup_object("gcode")


                        # 1. Pause printer
                        self.logger.info("OAMS: Step 1 - Pausing printer")

                        gcode.run_script("PAUSE")


                        # 2. Save position
                        self.logger.info("OAMS: Step 2 - Saving position")

                        afc.save_pos()

                        # 3. Z-hop to lift nozzle off print
                        self.logger.info("OAMS: Step 3 - Z-hop 5mm")

                        gcode.run_script("SAVE_GCODE_STATE NAME=oams_zhop")
                        try:
                            gcode.run_script("G91")  # Relative positioning
                            gcode.run_script("G1 Z5 F600")  # Lift 5mm
                        finally:
                            gcode.run_script("RESTORE_GCODE_STATE NAME=oams_zhop MOVE=0")

                        # 4. Get target lane and extruder
                        self.logger.info("OAMS: Step 4 - Getting target lane info")

                        target_lane = afc.lanes.get(target_lane_name)
                        if not target_lane:
                            raise Exception(f"Target lane {target_lane_name} not found in AFC")


                        target_extruder_name = getattr(target_lane, 'extruder_name', None)
                        if not target_extruder_name:
                            raise Exception(f"Target lane {target_lane_name} has no extruder_name")


                        # Get source extruder name for turning it off later
                        source_extruder_name = getattr(source_lane, 'extruder_name', None) if source_lane else None

                        # Get current extruder temp to use for target
                        current_extruder = self.printer.lookup_object('toolhead').get_extruder()
                        target_temp = current_extruder.get_heater().target_temp
                        lane_snapshot = self._get_lane_snapshot(
                            target_lane_name,
                            unit_name=getattr(target_lane, "unit", None),
                        )
                        lane_extruder_temp = None
                        if isinstance(lane_snapshot, dict):
                            lane_extruder_temp = lane_snapshot.get("extruder_temp")
                        if lane_extruder_temp is not None:
                            target_temp = lane_extruder_temp
                        try:
                            target_temp_value = float(target_temp)
                        except (TypeError, ValueError):
                            raise Exception(f"Current extruder has invalid target temp: {target_temp}")
                        if target_temp_value <= 0:
                            raise Exception(f"Current extruder has no target temp set")


                        # 5. Set target extruder temp before CHANGE_TOOL (so it knows what temp to heat to)
                        self.logger.info(
                            f"OAMS: Step 5 - Setting target temp for {target_extruder_name} to {target_temp_value:.1f}"
                        )
                        target_extruder_obj = self.printer.lookup_object(target_extruder_name)
                        target_heater = target_extruder_obj.get_heater()
                        target_heater.set_temp(target_temp_value)

                        # 6. Use AFC's CHANGE_TOOL Python method (like box turtle does)
                        # This should handle tool switching, heating, and loading
                        self.logger.info(f"OAMS: Step 6 - Calling afc.CHANGE_TOOL for {target_lane_name}")
                        afc.CHANGE_TOOL(target_lane, restore_pos=False)

                        # 8. Update FPS state - this FPS is now UNLOADED (new lane is on different FPS/tool)
                        # Cross-extruder runout means lane0 is on a different FPS, so this FPS has nothing loaded
                        self.logger.info("OAMS: Step 8 - Setting FPS state to UNLOADED (lane loaded to different FPS)")

                        fps_state.state = FPSLoadState.UNLOADED
                        fps_state.current_lane = None
                        fps_state.current_spool_idx = None
                        fps_state.current_oams = None

                        # 9. Set mapping so T# references use new lane
                        self.logger.info("OAMS: Step 9 - Setting lane mapping")

                        empty_lane_map = getattr(source_lane, 'map', None) or source_lane_name
                        gcode.run_script(f"SET_MAP LANE={target_lane_name} MAP={empty_lane_map}")

                        self.logger.info(f"OAMS: Set mapping {target_lane_name} -> {empty_lane_map}")

                        # 10. Unload empty lane from unit
                        if not afc.error_state:
                            self.logger.info(f"OAMS: Step 10 - Unloading empty lane {source_lane_name}")
                            gcode.run_script(f"LANE_UNLOAD LANE={source_lane_name}")


                            # 11. Restore position (brings Z back down) and resume
                            self.logger.info("OAMS: Step 11 - Restoring position and resuming")

                            afc.restore_pos()
                            gcode.run_script("RESUME")


                            # 12. Turn off the old extruder that ran out (now that we're on the new tool)
                            if source_extruder_name:
                                try:
                                    self.logger.info(f"OAMS: Step 12 - Turning off old extruder {source_extruder_name}")
                                    source_extruder_obj = self.printer.lookup_object(source_extruder_name)
                                    source_heater = source_extruder_obj.get_heater()
                                    source_heater.set_temp(0)
                                    self.logger.info(f"OAMS: Set {source_extruder_name} heater target to 0")
                                except Exception as e:
                                    self.logger.error(f"OAMS: Failed to turn off old extruder {source_extruder_name}: {e}")
                            else:
                                self.logger.warning("OAMS: Cannot turn off old extruder - source_extruder_name not available")

                        else:
                            self.logger.error("OAMS: AFC error_state is set, skipping LANE_UNLOAD and RESUME")


                        # Clear cross-extruder flag on source lane
                        if source_lane_name:
                            try:
                                source_lane_obj = afc.lanes.get(source_lane_name)
                                if source_lane_obj and hasattr(source_lane_obj, '_oams_cross_extruder_runout'):
                                    source_lane_obj._oams_cross_extruder_runout = False
                                    self.logger.info(f"OAMS: Cleared cross-extruder runout flag on lane {source_lane_name}")
                            except Exception as e:
                                self.logger.error(f"OAMS: Failed to clear cross-extruder runout flag on lane {source_lane_name}: {e}")

                        # Reset state and restart monitoring
                        fps_state.reset_runout_positions()
                        fps_state.is_cross_extruder_runout = False
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        self.logger.info("OAMS: Cross-extruder runout sequence completed successfully")

                        return

                    except Exception as e:
                        self.logger.error(f"OAMS: Failed to execute cross-extruder runout sequence - Exception: {str(e)}")

                        # Clear cross-extruder flag on error too
                        if source_lane_name:
                            try:
                                afc = self.printer.lookup_object('AFC')
                                if afc and hasattr(afc, 'lanes'):
                                    source_lane_obj = afc.lanes.get(source_lane_name)
                                    if source_lane_obj and hasattr(source_lane_obj, '_oams_cross_extruder_runout'):
                                        source_lane_obj._oams_cross_extruder_runout = False
                            except Exception:
                                pass

                        fps_state.reset_runout_positions()
                        fps_state.is_cross_extruder_runout = False
                        self._pause_printer_message(f"Failed to change tool for {source_lane_name or fps_name}: {str(e)}", active_oams)
                        if monitor:
                            monitor.paused()
                        return

                # Standard runout handling for same-extruder runouts
                target_lane_map, target_lane, delegate_to_afc, source_lane = self._get_infinite_runout_target_lane(fps_name, fps_state)
                source_lane_name = fps_state.current_lane

                if delegate_to_afc:
                    delegated = self._delegate_runout_to_afc(fps_name, fps_state, source_lane, target_lane)
                    if delegated:
                        fps_state.reset_runout_positions()
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        return

                    self.logger.error(f"Failed to delegate infinite runout for {fps_name} on {source_lane_name or '<unknown>'} via AFC")
                    fps_state.reset_runout_positions()
                    self._pause_printer_message(f"Unable to delegate infinite runout for {source_lane_name or fps_name}", fps_state.current_oams or active_oams)
                    if monitor:
                        monitor.paused()
                    return

                # Load the target lane directly
                if target_lane is None:
                    # No infinite runout target configured - clear the lane and pause
                    self.logger.info(f"No infinite runout target for {source_lane_name or fps_name} on {fps_name} - clearing lane from toolhead and OAMS")


                    # Clear FPS state and notify AFC (similar to cross-extruder runout handling)
                    self._clear_lane_on_runout(fps_name, fps_state, source_lane_name)

                    self.logger.error(f"No lane available to reload on {fps_name}")
                    self._pause_printer_message(f"No lane available to reload on {fps_name}", fps_state.current_oams or active_oams)
                    if monitor:
                        monitor.paused()
                    return

                if target_lane_map:
                    self.logger.info(f"Infinite runout triggered for {fps_name} on {source_lane_name} -> {target_lane}")

                    # CRITICAL FIX: Use async non-blocking reload mechanism
                    # Blocking waits (toolhead.dwell or reactor.pause) don't work from timer context during printing
                    # Instead, start the OAMS hardware operation and poll for completion with timers
                    self._start_async_reload(fps_name, fps_state, target_lane, target_lane_map,
                                           source_lane_name, active_oams, monitor)
                    return

                # Fallback: If target_lane_map is somehow None, use async path anyway
                # This shouldn't normally happen, but ensures we always use working async mechanism
                self.logger.warning(f"target_lane_map is None for {source_lane_name} -> {target_lane}, using async reload anyway")
                self._start_async_reload(fps_name, fps_state, target_lane, target_lane or "unknown",
                                       source_lane_name, active_oams, monitor)

            fps_reload_margin = getattr(self.fpss[fps_name], "reload_before_toolhead_distance", None)
            if fps_reload_margin is None:
                fps_reload_margin = self.reload_before_toolhead_distance

            monitor = OAMSRunoutMonitor(
                self.printer,
                fps_name,
                self.fpss[fps_name],
                self.current_state.fps_state[fps_name],
                self.oams,
                _reload_callback,
                follower_callback=self._set_follower_state,
                logger=self.logger,
                reload_before_toolhead_distance=fps_reload_margin,
                debounce_delay=self.debounce_delay,
            )
            self.runout_monitors[fps_name] = monitor
            monitor.start()

        self.logger.info("All monitors started (runout, FPS, and encoder checks active)")

    # ============================================================================
    # AFC State Change Notifications (Optional Callbacks)
    # ============================================================================
    # These methods can be called by AFC when lane state changes to keep OAMS
    # manager in sync immediately without waiting for state detection polling.
    # AFC integration is optional - these are no-ops if called when AFC is not configured.

    def on_afc_lane_loaded(self, lane_name: str, extruder_name: Optional[str] = None) -> None:
        """Callback for AFC to notify OAMS when a lane is loaded.

        Args:
            lane_name: Name of the lane that was loaded (e.g., "lane4")

            extruder_name: Optional name of the extruder/tool it was loaded to

        This allows OAMS to:
        - Update FPS state immediately (no polling lag)
        - Enable follower motor instantly
        - Sync state for accurate monitoring
        """
        try:
            # Get FPS for this lane (uses cache for speed)
            fps_name = self.get_fps_for_afc_lane(lane_name)
            if not fps_name:
                return  # Lane not on any FPS we manage

            fps_state = self.current_state.fps_state.get(fps_name)
            if fps_state is None:
                return

            # Let state detection handle the full sync
            # We just trigger it to run immediately instead of waiting for next poll
            detected_lane_name, oam, bay_index = self._determine_loaded_lane_for_fps(fps_name, self.fpss[fps_name])

            if detected_lane_name and oam and bay_index is not None:
                # Update FPS state - store lane name (e.g., "lane8") not map (e.g., "T0")

                fps_state.current_lane = detected_lane_name
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.direction = 1

                # Enable follower immediately
                self._ensure_forward_follower(fps_name, fps_state, "AFC lane loaded notification")


                self.logger.info(f"Synced OAMS state from AFC: {detected_lane_name} loaded to {fps_name} (bay {bay_index} on {oam.name})")

        except Exception:
            self.logger.error(f"Error processing AFC lane loaded notification for {lane_name}")
    def on_afc_lane_unloaded(self, lane_name: str, extruder_name: Optional[str] = None) -> None:
        """Callback for AFC to notify OAMS when a lane is unloaded.

        Args:
            lane_name: Name of the lane that was unloaded (e.g., "lane4")

            extruder_name: Optional name of the extruder/tool it was unloaded from

        This allows OAMS to update FPS state immediately while leaving follower
        control to the hub sensor logic so it only disables when all hubs on the
        unit are empty.
        """
        try:
            # Get FPS for this lane (uses cache for speed)
            fps_name = self.get_fps_for_afc_lane(lane_name)
            if not fps_name:
                return  # Lane not on any FPS we manage

            fps_state = self.current_state.fps_state.get(fps_name)
            if fps_state is None:
                return

            # Only update if this lane was actually loaded on this FPS
            if fps_state.state == FPSLoadState.LOADED:
                prev_oams_name = fps_state.current_oams
                fps_state.state = FPSLoadState.UNLOADED
                fps_state.current_lane = None
                fps_state.current_oams = None
                fps_state.current_spool_idx = None
                fps_state.since = self.reactor.monotonic()

                # Let hub sensor control decide whether follower stays on based on
                # remaining filament in the unit
                if prev_oams_name:
                    oam = self.oams.get(prev_oams_name)
                    if oam:
                        self._update_follower_for_oams(prev_oams_name, oam)

                self.logger.info(f"Synced OAMS state from AFC: {lane_name} unloaded from {fps_name}")
        except Exception:
            self.logger.error(f"Error processing AFC lane unloaded notification for {lane_name}")
    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []
        for monitor in self.runout_monitors.values():
            monitor.reset()
        self.runout_monitors = {}

    @contextmanager
    def _monitors_suspended(self, reason: str = "", restart_on_exit: bool = True):
        """Pause monitors while performing coordinated reset work."""
        monitors_were_running = bool(self.monitor_timers)
        if monitors_were_running:
            try:
                self.stop_monitors()
                if reason:
                    self.logger.debug(f"Monitors paused for {reason}")
            except Exception:
                self.logger.error(f"Failed to pause monitors{' for ' + reason if reason else ''}")
                monitors_were_running = False
        try:
            yield
        finally:
            if monitors_were_running and restart_on_exit:
                try:
                    self.start_monitors()
                    if reason:
                        self.logger.debug(f"Monitors resumed after {reason}")
                except Exception:
                    self.logger.error(f"Failed to resume monitors{' after ' + reason if reason else ''}")

def load_config(config):
    return OAMSManager(config)
