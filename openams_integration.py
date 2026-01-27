# Armored Turtle Automated Filament Changer  
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""Shared OpenAMS integration helpers used by AFC and OpenAMS."""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

OPENAMS_VERSION = "0.0.3"

# ============================================================================
# Shared Utility Functions
# ============================================================================

def normalize_extruder_name(name: Optional[str]) -> Optional[str]:
    """Return a case-insensitive token for comparing extruder aliases.

    Strips whitespace and converts to lowercase. Also removes 'ams_' prefix
    if present (for AMS virtual extruder names).

    Args:
        name: Extruder name to normalize (e.g., "extruder", "Extruder4", "ams_extruder")

    Returns:
        Normalized name (e.g., "extruder", "extruder4") or None if invalid
    """
    if not name or not isinstance(name, str):
        return None

    normalized = name.strip()
    if not normalized:
        return None

    lowered = normalized.lower()
    if lowered.startswith("ams_"):
        lowered = lowered[4:]

    return lowered or None


# ============================================================================
# Event System
# ============================================================================

class AMSEventBus:
    """Lightweight event system for lane state changes.
    
    This eliminates polling overhead by publishing events when state changes occur.
    Subscribers register callbacks for specific event types.
    
    Event Types:
        - spool_loaded: When OAMS loads a spool
        - spool_unloaded: When OAMS unloads a spool
        - lane_tool_loaded: When filament reaches the toolhead
        - lane_tool_unloaded: When filament leaves the toolhead
        - lane_hub_loaded: When filament reaches the hub
        - lane_hub_unloaded: When filament leaves the hub
        - fps_state_changed: When FPS sensor state changes
    """
    
    _instance: Optional['AMSEventBus'] = None
    _lock = threading.RLock()
    
    def __init__(self):
        self._subscribers: Dict[str, List[Tuple[Callable, int]]] = {}
        self._event_history: List[Tuple[str, float, Dict[str, Any]]] = []
        self._max_history = 100
        self.logger = logging.getLogger("AMSEventBus")
    
    @classmethod
    def get_instance(cls) -> 'AMSEventBus':
        """Get or create the singleton event bus."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance
    
    def subscribe(self, event_type: str, callback: Callable, *, priority: int = 0) -> None:
        """Register a callback for a specific event type.
        
        Args:
            event_type: Type of event to subscribe to
            callback: Function to call when event occurs (receives **kwargs)
            priority: Higher priority callbacks are called first (default: 0)
        """
        with self._lock:
            if event_type not in self._subscribers:
                self._subscribers[event_type] = []
            
            # Insert based on priority (higher priority first)
            subscribers = self._subscribers[event_type]
            insert_idx = 0
            for i, (existing_callback, existing_priority) in enumerate(subscribers):
                if priority > existing_priority:
                    insert_idx = i
                    break
                insert_idx = i + 1
            
            subscribers.insert(insert_idx, (callback, priority))
            self.logger.debug("Subscribed to '%s' (priority=%d, total=%d)", 
                            event_type, priority, len(subscribers))
    
    def unsubscribe(self, event_type: str, callback: Callable) -> None:
        """Unregister a callback from a specific event type."""
        with self._lock:
            if event_type in self._subscribers:
                self._subscribers[event_type] = [
                    (cb, pri) for cb, pri in self._subscribers[event_type] 
                    if cb != callback
                ]
    
    def publish(self, event_type: str, **kwargs) -> int:
        """Publish an event to all subscribers.
        
        Args:
            event_type: Type of event being published
            **kwargs: Event data to pass to subscribers
            
        Returns:
            Number of subscribers that successfully handled the event
        """
        import time
        eventtime = kwargs.get('eventtime', time.time())
        
        with self._lock:
            # Record event in history
            self._event_history.append((event_type, eventtime, dict(kwargs)))
            if len(self._event_history) > self._max_history:
                self._event_history.pop(0)
            
            subscribers = list(self._subscribers.get(event_type, []))
        
        if not subscribers:
            return 0
        
        success_count = 0
        for callback, priority in subscribers:
            try:
                callback(event_type=event_type, **kwargs)
                success_count += 1
            except Exception:
                self.logger.error("Event handler failed for '%s' (priority=%d)", 
                                    event_type, priority)
        
        return success_count
    
    def get_history(self, event_type: Optional[str] = None, 
                   since: Optional[float] = None) -> List[Tuple[str, float, Dict[str, Any]]]:
        """Get event history, optionally filtered by type and time."""
        with self._lock:
            history = list(self._event_history)
        
        if event_type:
            history = [(et, time, data) for et, time, data in history if et == event_type]
        
        if since is not None:
            history = [(et, time, data) for et, time, data in history if time >= since]
        
        return history


# ============================================================================
# Lane Registry
# ============================================================================

@dataclass
class LaneInfo:
    """Complete identity information for a single lane.

    This is the single source of truth for lane identity across OpenAMS and AFC.
    All lookups (by name, spool index, lane alias, etc.) route through the registry.
    """
    lane_name: str          # "lane4", "lane5", etc.
    unit_name: str          # "AMS_1", "AMS_2", etc.
    spool_index: int        # 0-3 (zero-indexed position in OAMS unit)
    extruder: str           # "extruder4", "extruder5", etc.
    
    # Optional fields
    fps_name: Optional[str] = None          # "fps1", "fps2", etc.
    hub_name: Optional[str] = None          # "Hub_1", "Hub_2", etc.
    led_index: Optional[str] = None         # LED indicator reference
    custom_load_cmd: Optional[str] = None   # Custom load macro
    custom_unload_cmd: Optional[str] = None # Custom unload macro
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            'lane_name': self.lane_name,
            'unit_name': self.unit_name,
            'spool_index': self.spool_index,
            'extruder': self.extruder,
            'fps_name': self.fps_name,
            'hub_name': self.hub_name,
            'led_index': self.led_index,
            'custom_load_cmd': self.custom_load_cmd,
            'custom_unload_cmd': self.custom_unload_cmd,
        }


class LaneRegistry:
    """Single source of truth for lane identity across OpenAMS and AFC.
    
    This registry eliminates multiple redundant lookups by providing O(1) access
    to lane information via any identifier (lane name, spool index, etc.).
    
    All lane tracking should go through this registry instead of maintaining
    separate mappings in multiple files.
    """
    
    _instances: Dict[int, 'LaneRegistry'] = {}
    _lock = threading.RLock()
    
    def __init__(self, printer):
        self.printer = printer
        self.logger = logging.getLogger("LaneRegistry")
        
        # Primary storage
        self._lanes: List[LaneInfo] = []
        
        # Indexed lookups for O(1) access
        self._by_lane_name: Dict[str, LaneInfo] = {}
        self._by_spool: Dict[Tuple[str, int], LaneInfo] = {}
        self._by_extruder: Dict[str, List[LaneInfo]] = {}
        
        # Subscribe to events
        self.event_bus = AMSEventBus.get_instance()
    
    @classmethod
    def for_printer(cls, printer) -> 'LaneRegistry':
        """Get or create the singleton registry for a printer."""
        with cls._lock:
            key = id(printer)
            if key not in cls._instances:
                cls._instances[key] = cls(printer)
            return cls._instances[key]
    
    def register_lane(self,
                     lane_name: str,
                     unit_name: str,
                     spool_index: int,
                     extruder: str,
                     *,
                     fps_name: Optional[str] = None,
                     hub_name: Optional[str] = None,
                     led_index: Optional[str] = None,
                     custom_load_cmd: Optional[str] = None,
                     custom_unload_cmd: Optional[str] = None) -> LaneInfo:
        """Register a lane with all its identifiers.

        This is typically called during AFC_lane initialization.

        Args:
            lane_name: AFC lane identifier (e.g., "lane4")
            unit_name: AMS unit name (e.g., "AMS_1")
            spool_index: Zero-indexed spool position in OAMS unit (0-3)
            extruder: Klipper extruder name (e.g., "extruder4")
            fps_name: Optional FPS identifier (e.g., "fps1")
            hub_name: Optional hub identifier (e.g., "Hub_1")
            led_index: Optional LED indicator reference
            custom_load_cmd: Optional custom load command
            custom_unload_cmd: Optional custom unload command

        Returns:
            The registered LaneInfo object
        """
        with self._lock:
            # Check if already registered
            existing = self._by_lane_name.get(lane_name)
            if existing is not None:
                self.logger.warning("Lane '%s' already registered, updating", lane_name)
                self._unregister_lane(existing)

            # Create lane info
            info = LaneInfo(
                lane_name=lane_name,
                unit_name=unit_name,
                spool_index=spool_index,
                extruder=extruder,
                fps_name=fps_name,
                hub_name=hub_name,
                led_index=led_index,
                custom_load_cmd=custom_load_cmd,
                custom_unload_cmd=custom_unload_cmd,
            )
            
            # Add to storage
            self._lanes.append(info)

            # Build indexes
            self._by_lane_name[lane_name] = info
            self._by_spool[(unit_name, spool_index)] = info

            if extruder not in self._by_extruder:
                self._by_extruder[extruder] = []
            self._by_extruder[extruder].append(info)

            self.logger.info("Registered lane: %s ? %s[%d] (extruder=%s, fps=%s)",
                           lane_name, unit_name, spool_index, extruder, fps_name)
            
            return info
    
    def _unregister_lane(self, info: LaneInfo) -> None:
        """Internal: Remove a lane from all indexes."""
        if info in self._lanes:
            self._lanes.remove(info)
        
        self._by_lane_name.pop(info.lane_name, None)
        self._by_spool.pop((info.unit_name, info.spool_index), None)

        extruder_lanes = self._by_extruder.get(info.extruder, [])
        if info in extruder_lanes:
            extruder_lanes.remove(info)
            # Remove empty list to prevent memory leak
            if not extruder_lanes:
                self._by_extruder.pop(info.extruder, None)
    
    def get_by_lane(self, lane_name: str) -> Optional[LaneInfo]:
        """Get lane info by AFC lane name (e.g., "lane4")."""
        with self._lock:
            return self._by_lane_name.get(lane_name)
    
    def get_by_spool(self, unit_name: str, spool_index: int) -> Optional[LaneInfo]:
        """Get lane info by OAMS unit and spool index (e.g., "AMS_1", 0)."""
        with self._lock:
            return self._by_spool.get((unit_name, spool_index))

    def resolve_lane_token(self, token: str) -> Optional[LaneInfo]:
        """Resolve a lane by name."""
        lowered = token.lower()
        with self._lock:
            for lane_name, info in self._by_lane_name.items():
                if lane_name.lower() == lowered:
                    return info

            return None
    
    def get_by_extruder(self, extruder: str) -> List[LaneInfo]:
        """Get all lanes for an extruder (e.g., "extruder4")."""
        with self._lock:
            return list(self._by_extruder.get(extruder, []))
    
    def get_all_lanes(self) -> List[LaneInfo]:
        """Get all registered lanes."""
        with self._lock:
            return list(self._lanes)
    
    def resolve_lane_name(self, unit_name: str, spool_index: int) -> Optional[str]:
        """Helper: Get lane name from unit and spool index."""
        info = self.get_by_spool(unit_name, spool_index)
        return info.lane_name if info else None
    
    def resolve_spool_index(self, lane_name: str) -> Optional[int]:
        """Helper: Get spool index from lane name."""
        info = self.get_by_lane(lane_name)
        return info.spool_index if info else None
    
    def resolve_extruder(self, lane_name: str) -> Optional[str]:
        """Helper: Get extruder from lane name."""
        info = self.get_by_lane(lane_name)
        return info.extruder if info else None


# ============================================================================

# ============================================================================
# AMSHardwareService (Enhanced with Registry)
# ============================================================================

class AMSHardwareService:
    """Centralised interface for accessing OpenAMS hardware from AFC.

    The service tracks the underlying ``OAMS`` firmware object created by
    ``klipper_openams`` and exposes high level helpers so AFC can interact with
    the same hardware instance without reimplementing any low level MCU
    messaging.
    
    Uses LaneRegistry for all lane lookups.
    """

    _instances: Dict[Tuple[int, str], "AMSHardwareService"] = {}

    def __init__(self, printer, name: str, logger: Optional[logging.Logger] = None):
        self.printer = printer
        self.name = name
        self.logger = logger or logging.getLogger(f"AFC.AMS.{name}")
        self._controller = None
        self._lock = threading.RLock()
        self._status: Dict[str, Any] = {}
        self._lane_snapshots: Dict[str, Dict[str, Any]] = {}
        self._status_callbacks: List[Callable[[Dict[str, Any]], None]] = []

        # Use registry instead of local _lanes_by_spool
        self.registry = LaneRegistry.for_printer(printer)
        self.event_bus = AMSEventBus.get_instance()

        # Cache reactor reference
        self._reactor = None

        # Unified polling with event publishing
        self._polling_timer = None
        self._polling_interval = 2.0  # Active polling interval
        self._polling_interval_idle = 4.0  # Idle polling interval
        self._consecutive_idle_polls = 0
        self._idle_poll_threshold = 3
        self._last_encoder_clicks = None
        self._last_f1s_hes = [None, None, None, None]
        self._last_hub_hes = [None, None, None, None]
        self._last_fps_value = None
        self._polling_enabled = False

    @classmethod
    def for_printer(cls, printer, name: str = "default", logger: Optional[logging.Logger] = None) -> "AMSHardwareService":
        """Return the singleton service for the provided printer/name pair."""
        key = (id(printer), name)
        try:
            service = cls._instances[key]
        except KeyError:
            service = cls(printer, name, logger)
            cls._instances[key] = service
        else:
            if logger is not None:
                service.logger = logger
        return service

    def attach_controller(self, controller: Any) -> None:
        """Attach the low level ``OAMS`` controller to this service."""
        with self._lock:
            self._controller = controller
        if controller is not None:
            try:
                status = controller.get_status(self._monotonic())
            except Exception:
                status = None
            if status:
                self._update_status(status)
            self.logger.debug("Attached OAMS controller %s", controller)

    def resolve_controller(self) -> Optional[Any]:
        """Return the currently attached controller, attempting lookup if needed."""
        with self._lock:
            controller = self._controller
        if controller is not None:
            return controller

        lookup_name = f"oams {self.name}"
        try:
            controller = self.printer.lookup_object(lookup_name, None)
        except Exception:
            controller = None
        if controller is not None:
            self.attach_controller(controller)
        return controller

    def _monotonic(self) -> float:
        """Cache reactor reference for faster lookups."""
        if self._reactor is None:
            reactor = getattr(self.printer, "get_reactor", None)
            if callable(reactor):
                try:
                    self._reactor = reactor()
                except Exception:
                    pass
            if self._reactor is None:
                try:
                    self._reactor = self.printer.get_reactor()
                except Exception:
                    return 0.0

        try:
            return self._reactor.monotonic()
        except Exception:
            return 0.0

    def _log_info(self, message: str) -> None:
        """Helper to log info messages compatible with both AFC_logger and standard logging."""
        try:
            # AFC_logger signature: info(message, console_only=False)
            self.logger.info(message)
        except TypeError:
            # Standard logging signature: info(msg, *args)
            self.logger.info(message)

    def _log_warning(self, message: str) -> None:
        """Helper to log warning messages compatible with both AFC_logger and standard logging."""
        try:
            self.logger.warning(message)
        except TypeError:
            self.logger.warning(message)

    def _log_error(self, message: str) -> None:
        """Helper to log error messages compatible with both AFC_logger and standard logging."""
        try:
            # AFC_logger has error() but not exception()
            if hasattr(self.logger, 'error'):
                self.logger.error(message)
            else:
                self.logger.error(message)
        except Exception:
            # Fallback to print if all else fails
            print(f"AMSHardwareService: {message}")

    def _log_debug(self, message: str) -> None:
        """Helper to log debug messages compatible with both AFC_logger and standard logging."""
        try:
            self.logger.debug(message)
        except Exception:
            pass  # Ignore debug logging failures

    def start_polling(self) -> None:
        """Start the unified hardware polling timer.

        Centralizes all hardware polling in one place and publishes events
        when sensor values change, allowing subscribers to react without polling.
        """
        if self._polling_timer is not None:
            self._log_warning(f"Polling already started for {self.name}")
            return

        if self._reactor is None:
            self._monotonic()  # Initialize reactor

        if self._reactor is None:
            self._log_error("Cannot start polling: reactor not available")
            return

        self._polling_enabled = True
        # Offset timer by 1 second to avoid CPU spike when both polling callbacks run simultaneously
        # This spreads the load: oams_manager runs at 0s,2s,4s... and this runs at 1s,3s,5s...
        self._polling_timer = self._reactor.register_timer(
            self._polling_callback,
            self._reactor.NOW + 1.0
        )
        self._log_info(f"Started unified hardware polling for {self.name} (offset by 1s)")

    def stop_polling(self) -> None:
        """Stop the unified hardware polling timer."""
        self._polling_enabled = False
        if self._polling_timer is not None and self._reactor is not None:
            self._reactor.unregister_timer(self._polling_timer)
            self._polling_timer = None
            self._log_info(f"Stopped unified hardware polling for {self.name}")

    def _polling_callback(self, eventtime: float) -> float:
        """Unified polling callback that detects changes and publishes events.

        Polls OAMS hardware sensors and publishes events to subscribers when
        state changes are detected. Single source of truth for hardware state.
        """
        if not self._polling_enabled:
            return self._reactor.NEVER

        try:
            # Poll hardware to keep sensor cache fresh
            # This ensures oams_manager reads fresh data from oams.hub_hes_value, etc.
            status = self.poll_status()
            if not status:
                return eventtime + self._polling_interval_idle

            # Detect encoder changes (for adaptive polling)
            encoder_changed = False
            encoder_clicks = status.get("encoder_clicks")
            if encoder_clicks is not None:
                if self._last_encoder_clicks is not None:
                    if encoder_clicks != self._last_encoder_clicks:
                        encoder_changed = True
                        self._consecutive_idle_polls = 0
                self._last_encoder_clicks = encoder_clicks

            # Detect f1s_hes changes (spool present in bay)
            f1s_values = status.get("f1s_hes_value", [])
            for bay in range(min(len(f1s_values), 4)):
                new_val = bool(f1s_values[bay])
                old_val = self._last_f1s_hes[bay]

                # Publish on first detection (old_val is None) OR when value changes
                if old_val is None or new_val != old_val:
                    # Publish sensor change event - AFC_OpenAMS subscribes to these
                    self.event_bus.publish(
                        "f1s_changed",
                        unit_name=self.name,
                        bay=bay,
                        value=new_val,
                        eventtime=eventtime
                    )
                    if old_val is None:
                        self._log_info(f"f1s[{bay}] initial state: {new_val}")
                    else:
                        self._log_debug(f"f1s[{bay}] changed: {old_val} -> {new_val}")

                self._last_f1s_hes[bay] = new_val

            # Detect hub_hes changes (filament at hub)
            hub_values = status.get("hub_hes_value", [])
            for bay in range(min(len(hub_values), 4)):
                new_val = bool(hub_values[bay])
                old_val = self._last_hub_hes[bay]

                # Publish on first detection (old_val is None) OR when value changes
                if old_val is None or new_val != old_val:
                    # Publish sensor change event - AFC_OpenAMS subscribes to these
                    self.event_bus.publish(
                        "hub_changed",
                        unit_name=self.name,
                        bay=bay,
                        value=new_val,
                        eventtime=eventtime
                    )
                    if old_val is None:
                        self._log_info(f"hub[{bay}] initial state: {new_val}")
                    else:
                        self._log_debug(f"hub[{bay}] changed: {old_val} -> {new_val}")

                self._last_hub_hes[bay] = new_val

            # Detect fps_value changes (pressure sensor)
            fps_value = status.get("fps_value")
            if fps_value is not None:
                old_fps = self._last_fps_value
                # Only publish significant pressure changes (> 0.05 threshold)
                if old_fps is not None and abs(fps_value - old_fps) > 0.05:
                    self.event_bus.publish(
                        "fps_changed",
                        unit_name=self.name,
                        value=fps_value,
                        old_value=old_fps,
                        eventtime=eventtime
                    )
                self._last_fps_value = fps_value

            # Adaptive polling interval
            if encoder_changed:
                return eventtime + self._polling_interval

            self._consecutive_idle_polls += 1
            if self._consecutive_idle_polls > self._idle_poll_threshold:
                return eventtime + self._polling_interval_idle

            return eventtime + self._polling_interval

        except Exception:
            import traceback
            self._log_error(f"Error in unified polling callback for {self.name}: {traceback.format_exc()}")
            return eventtime + self._polling_interval_idle

    def poll_status(self) -> Optional[Dict[str, Any]]:
        """Query the controller for its latest status snapshot."""
        controller = self.resolve_controller()
        if controller is None:
            return None

        eventtime = self._monotonic()
        try:
            status = controller.get_status(eventtime)
        except Exception:
            #  Direct attribute access fallback
            status = {
                "current_spool": getattr(controller, "current_spool", None),
                "f1s_hes_value": list(getattr(controller, "f1s_hes_value", []) or []),
                "hub_hes_value": list(getattr(controller, "hub_hes_value", []) or []),
                "fps_value": getattr(controller, "fps_value", 0.0),
            }
            encoder = getattr(controller, "encoder_clicks", None)
            if encoder is not None:
                status["encoder_clicks"] = encoder
        self._update_status(status)
        return status

    def _update_status(self, status: Dict[str, Any]) -> None:
        """Update cached status and notify observers."""
        with self._lock:
            self._status = dict(status)
            callbacks = list(self._status_callbacks)
        
        # Only call callbacks if there are any registered
        if callbacks:
            status_copy = dict(status)
            for callback in callbacks:
                try:
                    callback(status_copy)
                except Exception:
                    import traceback
                    self._log_error(f"AMS status observer failed for {self.name}: {traceback.format_exc()}")

    def latest_status(self) -> Dict[str, Any]:
        """Return the most recently cached status snapshot."""
        with self._lock:
            return dict(self._status)

    def register_status_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Register a callback to be notified of status updates."""
        with self._lock:
            if callback not in self._status_callbacks:
                self._status_callbacks.append(callback)

    def unregister_status_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Unregister a previously registered status callback."""
        with self._lock:
            if callback in self._status_callbacks:
                self._status_callbacks.remove(callback)

    def update_lane_snapshot(self, unit_name: str, lane_name: str, lane_state: bool,
                           hub_state: Optional[bool], eventtime: float, *,
                           spool_index: Optional[int] = None,
                           tool_state: Optional[bool] = None,
                           emit_spool_event: bool = True) -> None:
        """Update the cached state snapshot for a specific lane.

        Publishes events to subscribers when lane state changes are detected,
        enabling event-driven updates instead of polling.
        """
        key = f"{unit_name}:{lane_name}"
        
        normalized_index: Optional[int]
        if spool_index is not None:
            try:
                normalized_index = int(spool_index)
            except (TypeError, ValueError):
                normalized_index = None
            else:
                if normalized_index < 0:
                    normalized_index = None
        else:
            normalized_index = None

        with self._lock:
            old_snapshot = self._lane_snapshots.get(key, {})

            self._lane_snapshots[key] = {
                "unit": unit_name,
                "lane": lane_name,
                "lane_state": bool(lane_state),
                "hub_state": None if hub_state is None else bool(hub_state),
                "timestamp": eventtime,
            }
            if normalized_index is not None:
                self._lane_snapshots[key]["spool_index"] = normalized_index
            elif "spool_index" in old_snapshot:
                self._lane_snapshots[key]["spool_index"] = old_snapshot["spool_index"]
            if tool_state is not None:
                self._lane_snapshots[key]["tool_state"] = bool(tool_state)
            
        # Determine the best spool index to report with events
        event_spool_index = normalized_index
        if event_spool_index is None:
            event_spool_index = old_snapshot.get("spool_index")

        # Publish state change events
        old_lane_state = old_snapshot.get("lane_state")
        new_lane_state = bool(lane_state)

        if emit_spool_event and (old_lane_state is None or old_lane_state != new_lane_state) and event_spool_index is not None:
            event_type = "spool_loaded" if new_lane_state else "spool_unloaded"
            self.event_bus.publish(
                event_type,
                unit_name=unit_name,
                lane_name=lane_name,
                spool_index=event_spool_index,
                eventtime=eventtime,
            )

        old_hub_state = old_snapshot.get("hub_state")
        new_hub_state = hub_state

        if old_hub_state is not None and new_hub_state is not None:
            if old_hub_state != new_hub_state:
                event_type = "lane_hub_loaded" if new_hub_state else "lane_hub_unloaded"
                self.event_bus.publish(
                    event_type,
                    unit_name=unit_name,
                    lane_name=lane_name,
                    spool_index=spool_index,
                    eventtime=eventtime
                )
        
        if tool_state is not None:
            old_tool_state = old_snapshot.get("tool_state")
            if old_tool_state is not None and old_tool_state != tool_state:
                event_type = "lane_tool_loaded" if tool_state else "lane_tool_unloaded"
                self.event_bus.publish(
                    event_type,
                    unit_name=unit_name,
                    lane_name=lane_name,
                    spool_index=spool_index,
                    eventtime=eventtime
                )

    def latest_lane_snapshot(self, unit_name: str, lane_name: str) -> Optional[Dict[str, Any]]:
        """Return the most recent state snapshot for a specific lane."""
        key = f"{unit_name}:{lane_name}"
        with self._lock:
            snapshot = self._lane_snapshots.get(key)
        return dict(snapshot) if snapshot else None

    def resolve_lane_for_spool(self, unit_name: str, spool_index: Optional[int]) -> Optional[str]:
        """Map a spool index to its corresponding lane name.
        
        Uses LaneRegistry instead of local mapping.
        """
        if spool_index is None:
            return None
        try:
            normalized = int(spool_index)
        except (TypeError, ValueError):
            return None
        
        # Use registry
        return self.registry.resolve_lane_name(unit_name, normalized)

    def resolve_lane_for_spool_with_afc(self, unit_name: str, spool_index: Optional[int]) -> Optional[str]:
        """Resolve lane name via registry with AFC fallback if needed."""
        lane_name = self.resolve_lane_for_spool(unit_name, spool_index)
        if lane_name is not None:
            return lane_name
        return self._resolve_lane_name_from_afc(unit_name, spool_index)

    def _resolve_lane_name_from_afc(self, unit_name: str, spool_index: Optional[int]) -> Optional[str]:
        """Fallback: resolve lane name by looking up AFC units directly."""
        if spool_index is None:
            return None
        try:
            normalized_index = int(spool_index)
        except (TypeError, ValueError):
            return None

        try:
            afc = self.printer.lookup_object("AFC", None)
        except Exception:
            afc = None
        if afc is None or not hasattr(afc, 'units'):
            return None

        for unit_obj in afc.units.values():
            if not hasattr(unit_obj, 'oams_name'):
                continue
            if unit_obj.oams_name != unit_name:
                continue

            target_slot = normalized_index + 1
            for lane_name, lane_obj in getattr(unit_obj, 'lanes', {}).items():
                lane_unit = getattr(lane_obj, 'unit', None)
                if lane_unit and ':' in lane_unit:
                    try:
                        slot = int(lane_unit.split(':', 1)[1])
                    except (ValueError, IndexError):
                        continue
                    if slot == target_slot:
                        return lane_name
        return None

    def latest_lane_snapshot_for_spool(self, unit_name: str, spool_index: Optional[int]) -> Optional[Dict[str, Any]]:
        """Return the most recent state snapshot for a spool by its index."""
        lane_name = self.resolve_lane_for_spool(unit_name, spool_index)
        if lane_name is None:
            return None
        return self.latest_lane_snapshot(unit_name, lane_name)

    def _require_controller(self):
        """Ensure a controller is available, raising if not."""
        controller = self.resolve_controller()
        if controller is None:
            raise RuntimeError(f"OpenAMS controller '{self.name}' is not ready")
        return controller

    def load_spool(self, spool_index: int) -> None:
        """Command the OAMS to load a specific spool.

        Sends load command to hardware and publishes spool_loaded event
        to notify subscribers of the state change.
        """
        controller = self._require_controller()
        controller.oams_load_spool_cmd.send([spool_index])
        
        # Publish event
        eventtime = self._monotonic()
        self.event_bus.publish(
            "spool_loaded",
            unit_name=self.name,
            spool_index=spool_index,
            eventtime=eventtime
        )

    def unload_spool(self) -> None:
        """Command the OAMS to unload the current spool.

        Sends unload command to hardware and publishes spool_unloaded event
        to notify subscribers of the state change.
        """
        controller = self._require_controller()
        current_spool = getattr(controller, "current_spool", None)
        controller.oams_unload_spool_cmd.send([])
        
        # Publish event
        eventtime = self._monotonic()
        self.event_bus.publish(
            "spool_unloaded",
            unit_name=self.name,
            spool_index=current_spool,
            eventtime=eventtime
        )

    def set_follower(self, enable: bool, direction: int) -> None:
        """Enable or disable the OAMS follower motor."""
        controller = self._require_controller()
        controller.oams_follower_cmd.send([1 if enable else 0, direction])

    def set_led_error(self, idx: int, value: int) -> None:
        """Set the error LED state for a specific spool bay."""
        controller = self._require_controller()
        controller.set_led_error(idx, value)


class AMSRunoutCoordinator:
    """Coordinates runout events between OpenAMS and AFC """

    _units: Dict[Tuple[int, str], List[Any]] = {}
    _monitors: Dict[Tuple[int, str], List[Any]] = {}
    _lock = threading.RLock()

    @classmethod
    def _key(cls, printer, name: str) -> Tuple[int, str]:
        """Generate a unique key for printer/name combinations."""
        return (id(printer), name)

    @classmethod
    def register_afc_unit(cls, unit) -> AMSHardwareService:
        """Register an ``afcAMS`` unit as participating in AMS integration."""
        service = AMSHardwareService.for_printer(unit.printer, unit.oams_name, unit.logger)
        key = cls._key(unit.printer, unit.oams_name)
        with cls._lock:
            cls._units.setdefault(key, [])
            if unit not in cls._units[key]:
                cls._units[key].append(unit)
        return service

    @classmethod
    def register_runout_monitor(cls, monitor) -> AMSHardwareService:
        """Register an OpenAMS runout monitor and return the hardware service."""
        printer = getattr(monitor, "printer", None)
        state = getattr(monitor, "fps_state", None)
        oams_name = getattr(state, "current_oams", None) if state else None
        if not oams_name:
            oams_name = getattr(monitor, "fps_name", "default")
        key = cls._key(printer, oams_name)
        with cls._lock:
            cls._monitors.setdefault(key, [])
            if monitor not in cls._monitors[key]:
                cls._monitors[key].append(monitor)
        return AMSHardwareService.for_printer(printer, oams_name)

    @classmethod
    def notify_runout_detected(cls, monitor, spool_index: Optional[int], *, lane_name: Optional[str] = None) -> None:
        """Forward runout detection from OpenAMS to any registered AFC units."""
        printer = getattr(monitor, "printer", None)
        state = getattr(monitor, "fps_state", None)
        oams_name = getattr(state, "current_oams", None) if state else None
        if not oams_name:
            oams_name = getattr(monitor, "fps_name", "default")
        key = cls._key(printer, oams_name)
        with cls._lock:
            units = list(cls._units.get(key, ()))
        lane_hint = lane_name or getattr(monitor, "latest_lane_name", None)
        for unit in units:
            try:
                unit.handle_runout_detected(spool_index, monitor, lane_name=lane_hint)
            except Exception:
                unit.logger.error("Failed to propagate OpenAMS runout to AFC unit %s", unit.name)

    @classmethod
    def notify_afc_error(cls, printer, name: str, message: str, *, pause: bool = False) -> None:
        """Deliver an OpenAMS pause/error message to any registered AFC units."""
        key = cls._key(printer, name)
        with cls._lock:
            units = list(cls._units.get(key, ()))

        for unit in units:
            afc = getattr(unit, "afc", None)
            if afc is None:
                continue

            error_obj = getattr(afc, "error", None)
            if error_obj is None:
                continue

            try:
                error_obj.AFC_error(message, pause=pause, level=3)
            except Exception:
                logger = getattr(unit, "logger", None)
                if logger is None:
                    logger = logging.getLogger(__name__)
                logger.error("Failed to deliver OpenAMS error '%s' to AFC unit %s", message, unit)

    @classmethod
    def notify_lane_tool_state(cls, printer, name: str, lane_name: str, *, loaded: bool, spool_index: Optional[int] = None, eventtime: Optional[float] = None) -> bool:
        """Propagate lane tool state changes from OpenAMS into AFC."""
        key = cls._key(printer, name)
        with cls._lock:
            units = list(cls._units.get(key, ()))

        if not units:
            return False

        if eventtime is None:
            try:
                eventtime = printer.get_reactor().monotonic()
            except Exception:
                eventtime = None

        handled = False
        for unit in units:
            try:
                if unit.handle_openams_lane_tool_state(lane_name, loaded, spool_index=spool_index, eventtime=eventtime):
                    handled = True
            except Exception:
                unit.logger.error("Failed to update AFC lane %s from OpenAMS tool state", lane_name)
        return handled

    @classmethod
    def mark_cross_extruder_runout(cls, printer, name: str, lane_name: str) -> bool:
        """Ask AFC units to mark a lane as participating in cross-extruder runout."""

        key = cls._key(printer, name)
        with cls._lock:
            units = list(cls._units.get(key, ()))

        if not units:
            return False

        handled = False
        for unit in units:
            try:
                marker = getattr(unit, "mark_cross_extruder_runout", None)
                if callable(marker) and marker(lane_name):
                    handled = True
            except Exception:
                unit.logger.error("Failed to mark lane %s as cross-extruder runout participant", lane_name)
        return handled

    @classmethod
    def active_units(cls, printer, name: str) -> Iterable[Any]:
        """Return all AFC units registered for a specific OpenAMS instance."""
        key = cls._key(printer, name)
        with cls._lock:
            return tuple(cls._units.get(key, ()))

    @classmethod
    def active_monitors(cls, printer, name: str) -> Iterable[Any]:
        """Return all runout monitors registered for a specific OpenAMS instance."""
        key = cls._key(printer, name)
        with cls._lock:
            return tuple(cls._monitors.get(key, ()))
