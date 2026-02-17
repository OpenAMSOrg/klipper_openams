# OpenAMS Mainboard
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

import mcu
import struct
from dataclasses import dataclass
from math import pi
from typing import Tuple, List, Optional, Dict

try:  # pragma: no cover - optional dependency during unit tests
    from extras.openams_integration import AMSHardwareService, OPENAMS_VERSION
except Exception:  # pragma: no cover - best-effort integration only
    AMSHardwareService = None
    OPENAMS_VERSION = "0.0.3"  # Fallback if import fails

# Pre-compiled struct formats for float conversions
_FLOAT_STRUCT = struct.Struct("f")
_U32_STRUCT = struct.Struct("I")

# OAMS Hardware Status Constants
class OAMSStatus:
    """Hardware status codes reported by OAMS firmware."""
    LOADING = 0
    UNLOADING = 1
    FORWARD_FOLLOWING = 2
    REVERSE_FOLLOWING = 3
    COASTING = 4
    STOPPED = 5
    CALIBRATING = 6
    ERROR = 7

# OAMS Operation Result Codes  
class OAMSOpCode:
    """Operation result codes from OAMS firmware."""
    SUCCESS = 0
    ERROR_UNSPECIFIED = 1
    ERROR_BUSY = 2
    SPOOL_ALREADY_IN_BAY = 3
    NO_SPOOL_IN_BAY = 4
    ERROR_KLIPPER_CALL = 5


@dataclass
class RetryState:
    """Tracks retry state for a single spool load/unload operation."""
    count: int = 0
    last_attempt: Optional[float] = None
    was_retry: bool = False

    def reset(self):
        """Reset retry state for a fresh operation."""
        self.count = 0
        self.last_attempt = None
        self.was_retry = False


class OAMS:
    """
    OpenAMS hardware controller for managing filament spools.
    
    Hardware Interface:
    - Controls 4 filament bays (indexed 0-3)
    - Monitors Hall Effect Sensors (HES) for spool detection
    - Manages BLDC motor for filament feeding
    - Tracks encoder position for motion feedback
    
    Key State Variables:
    - current_spool: Index (0-3) of currently loaded spool, None if unloaded
    - f1s_hes_value: Array of filament sensor readings [bay0, bay1, bay2, bay3]
    - hub_hes_value: Array of hub sensor readings [bay0, bay1, bay2, bay3] 
    - fps_value: Current pressure sensor reading
    - encoder_clicks: Current encoder position
    """
    
    def __init__(self, config):
        # Core printer interface
        self.printer = config.get_printer()
        self.section_name = config.get_name().split()[-1]
        self.mcu = mcu.get_printer_mcu(self.printer, config.get("mcu", "mcu"))
        self.reactor = self.printer.get_reactor()
        # Use load_object to ensure AFC is loaded even if not yet initialized
        afc_obj = self.printer.load_object(config, "AFC")
        self.logger = afc_obj.logger

        # OPTIMIZATION: Cache gcode object
        self._cached_gcode = None
        
        # Hardware configuration - Pressure sensor thresholds
        self.fps_upper_threshold: float = config.getfloat("fps_upper_threshold")

        self.fps_lower_threshold: float = config.getfloat("fps_lower_threshold")
 
        self.fps_is_reversed: bool = config.getboolean("fps_is_reversed")

        
        # Current state variables
        self.current_spool: Optional[int] = None
        self.encoder_clicks: int = 0
        self.i_value: float = 0.0
        
        # Sensor configuration - Hall Effect Sensor thresholds
        self.f1s_hes_on: List[float] = list(
            map(lambda x: float(x.strip()), config.get("f1s_hes_on").split(","))
        )
        self.f1s_hes_is_above: bool = config.getboolean("f1s_hes_is_above")

        self.hub_hes_on: List[float] = list(
            map(lambda x: float(x.strip()), config.get("hub_hes_on").split(","))
        )
        self.hub_hes_is_above: bool = config.getboolean("hub_hes_is_above")

        
        # Physical configuration
        self.filament_path_length: float = config.getfloat("ptfe_length")

        self.oams_idx: int = config.getint("oams_idx")


        # PID control parameters for pressure
        self.kd: float = config.getfloat("kd", 0.0)
        self.ki: float = config.getfloat("ki", 0.0)
        self.kp: float = config.getfloat("kp", 6.0)

        # PID control parameters for current
        self.current_kp: float = config.getfloat("current_kp", 0.375)
        self.current_ki: float = config.getfloat("current_ki", 0.0)
        self.current_kd: float = config.getfloat("current_kd", 0.0)

        # Target values
        self.fps_target: float = config.getfloat(
            "fps_target",
            0.5,
            minval=0.0,
            maxval=1.0,
            above=self.fps_lower_threshold,
            below=self.fps_upper_threshold,
        )
        self.current_target: float = config.getfloat(
            "current_target", 0.3, minval=0.1, maxval=0.4
        )
        
        # Hardware state arrays (updated by firmware)
        self.fps_value: float = 0
        self.f1s_hes_value: List[int] = [0, 0, 0, 0]
        self.hub_hes_value: List[int] = [0, 0, 0, 0]
        
        # Action status tracking
        self.action_status: Optional[int] = None
        self.action_status_code: Optional[int] = None
        self.action_status_value: Optional[int] = None
        
        # Setup MCU communication
        self.mcu.register_response(self._oams_action_status, "oams_action_status")

        self.mcu.register_response(self._oams_cmd_stats, "oams_cmd_stats")

        self.mcu.register_response(self._oams_cmd_current_status, "oams_cmd_current_status")

        self.mcu.register_config_callback(self._build_config)
        
        # Register commands and event handlers
        self.name = config.get_name()
        self.register_commands(self.name.split()[-1])

        # Retry configuration
        self.load_retry_max: int = config.getint("load_retry_max", 3, minval=1, maxval=5)
        self.unload_retry_max: int = config.getint("unload_retry_max", 2, minval=1, maxval=3)
        self.retry_delay: float = config.getfloat("retry_delay", 3.0, above=0.0)
        self.auto_unload_on_failed_load: bool = config.getboolean(
            "auto_unload_on_failed_load", True
        )
        self.dock_load: bool = config.getboolean("dock_load", False)
        self.post_load_purge: float = config.getfloat("post_load_purge", 0.0)
        self.extra_retract: float = config.getfloat("extra_retract", -10.0)

        # Retry state tracking
        self._load_retry_state: Dict[int, RetryState] = {}
        self._unload_retry_count: int = 0
        self._last_unload_attempt: float = 0.0
        self._last_successful_load: Dict[int, float] = {}

        # Retry failure statistics for monitoring
        self._load_retry_failures: int = 0
        self._unload_retry_failures: int = 0
        self._last_load_failure_time: Optional[float] = None
        self._last_unload_failure_time: Optional[float] = None
        self.hardware_service = None

        # Expose the underlying hardware controller to AFC when available
        if AMSHardwareService is not None:
            try:
                service = AMSHardwareService.for_printer(
                    self.printer, self.section_name
                )
                service.attach_controller(self)
                self.hardware_service = service
            except Exception as e:
                self.logger.error(
                    f"Failed to register OAMS controller with AMSHardwareService: {e}"
                )
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def _resolve_lane_name(self, spool_idx: int) -> Optional[str]:
        """Resolve lane name via integration layer (registry + AFC fallback)."""
        if self.hardware_service is None:
            return None
        return self.hardware_service.resolve_lane_for_spool_with_afc(self.section_name, spool_idx)

    def get_status(self, eventtime: float) -> dict:
        """Return current hardware status for monitoring."""
        return {
            "current_spool": self.current_spool,
            "f1s_hes_value": list(self.f1s_hes_value),
            "hub_hes_value": list(self.hub_hes_value),
            "fps_value": self.fps_value,
            # Retry failure statistics
            "load_retry_failures": self._load_retry_failures,
            "unload_retry_failures": self._unload_retry_failures,
            "last_load_failure_time": self._last_load_failure_time,
            "last_unload_failure_time": self._last_unload_failure_time,
        }
    
    def is_bay_ready(self, bay_index: int) -> bool:
        """Check if a spool bay has filament ready to load."""
        if not (0 <= bay_index < len(self.f1s_hes_value)):
            self.logger.error(f"Invalid bay_index {bay_index}, must be 0-{len(self.f1s_hes_value)-1}")
            return False
        return bool(self.f1s_hes_value[bay_index])
    
    def is_bay_loaded(self, bay_index: int) -> bool:
        """Check if a spool bay has filament loaded into the hub."""
        if not (0 <= bay_index < len(self.hub_hes_value)):
            self.logger.error(f"Invalid bay_index {bay_index}, must be 0-{len(self.hub_hes_value)-1}")
            return False
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

    def handle_connect(self):
        """Initialize MCU commands """
        # Define all commands in a structured way
        command_defs = {
            'load_spool': "oams_cmd_load_spool spool=%c",
            'unload_spool': "oams_cmd_unload_spool",
            'follower': "oams_cmd_follower enable=%c direction=%c",
            'calibrate_ptfe_length': "oams_cmd_calibrate_ptfe_length spool=%c",
            'calibrate_hub_hes': "oams_cmd_calibrate_hub_hes spool=%c",
            'pid': "oams_cmd_pid kp=%u ki=%u kd=%u target=%u",
            'set_led_error': "oams_set_led_error idx=%c value=%c",
        }
        
        try:
            # Batch command lookup 
            for cmd_name, cmd_string in command_defs.items():
                cmd_obj = self.mcu.lookup_command(cmd_string)
                setattr(self, f'oams_{cmd_name}_cmd', cmd_obj)
            
            # Query command with dedicated queue
            cmd_queue = self.mcu.alloc_command_queue()
            self.oams_spool_query_spool_cmd = self.mcu.lookup_query_command(
                "oams_cmd_query_spool",
                "oams_query_response_spool spool=%u",
                cq=cmd_queue,
            )
            
            self.clear_errors()
            
        except Exception as e:
            self.logger.error(f"Failed to initialize OAMS commands: {e}")
    def handle_ready(self):
        """Clear software error states when klippy is ready.

        This ensures that any stale action_status states from previous sessions
        don't prevent the follower from enabling when it should be enabled.
        Hardware error clearing (LED errors) was already done in handle_connect().
        """
        # Clear any stale action status from previous operations
        # Don't send MCU commands here to avoid interfering with manager initialization
        self.action_status = None
        self.action_status_code = None
        self.action_status_value = None
        self.logger.info(f"OAMS[{self.oams_idx}]: Cleared software error states on ready")
    def get_spool_status(self, bay_index):
        if not (0 <= bay_index < len(self.f1s_hes_value)):
            self.logger.error(f"Invalid bay_index {bay_index}, must be 0-{len(self.f1s_hes_value)-1}")
            return 0
        return self.f1s_hes_value[bay_index]
            
    def clear_errors(self):
        """Clear all error states including LED errors and action status."""
        # Clear all LED errors with individual try/except to ensure all LEDs are attempted
        # even if one fails. This prevents partial clearing if MCU communication is flaky.
        for i in range(4):
            try:
                self.set_led_error(i, 0)
            except Exception as e:
                self.logger.error(f"Failed to clear LED error for bay {i} on {getattr(self, 'name', 'unknown')}: {e}")

        # Clear any stale action status from previous operations
        self.action_status = None
        self.action_status_code = None
        self.action_status_value = None

        # Re-determine current spool to ensure state is accurate
        try:
            self.current_spool = self.determine_current_spool()
        except Exception as e:
            self.logger.error(f"Failed to determine current spool during clear_errors on {getattr(self, 'name', 'unknown')}: {e}")
            
    def set_led_error(self, idx, value):
        self.logger.debug(f"Setting LED {idx} to {value}")
        self.oams_set_led_error_cmd.send([idx, value])
        
    def determine_current_spool(self):
        params = self.oams_spool_query_spool_cmd.send()
        if params is None:
            self.logger.warning(f"OAMS[{self.oams_idx}]: Failed to query current spool - no response from MCU")
            return None

        if "spool" not in params:
            self.logger.warning(f"OAMS[{self.oams_idx}]: Spool query response missing 'spool' field")

            return None

        spool_val = params["spool"]
        if 0 <= spool_val <= 3:
            return spool_val

        # Spool index 255 (0xFF) is the hardware's way of saying "no spool loaded"
        # Only log if it's an unexpected invalid value
        if spool_val != 255:
            self.logger.warning(
                f"OAMS[{self.oams_idx}]: Unexpected spool index {spool_val} from hardware (expected 0-3 or 255); treating as no spool loaded"
            )
        else:
            self.logger.debug(
                f"OAMS[{self.oams_idx}]: No spool loaded (hardware returned 255)"
            )
        return None
        

    def register_commands(self, name):
        id = str(self.oams_idx)
        # OPTIMIZATION: Cache gcode object lookup
        if self._cached_gcode is None:
            self._cached_gcode = self.printer.lookup_object("gcode", None)
            if self._cached_gcode is None:
                return  # Gcode not available yet

        gcode = self._cached_gcode

        # Register all mux commands
        commands = [
            ("OAMS_LOAD_SPOOL", self.cmd_OAMS_LOAD_SPOOL, self.cmd_OAMS_LOAD_SPOOL_help),
            ("OAMS_UNLOAD_SPOOL", self.cmd_OAMS_UNLOAD_SPOOL, self.cmd_OAMS_UNLOAD_SPOOL_help),
            ("OAMS_FOLLOWER", self.cmd_OAMS_FOLLOWER, self.cmd_OAMS_FOLLOWER_help),
            ("OAMS_CALIBRATE_PTFE_LENGTH", self.cmd_OAMS_CALIBRATE_PTFE_LENGTH, self.cmd_OAMS_CALIBRATE_PTFE_LENGTH_help),
            ("OAMS_CALIBRATE_HUB_HES", self.cmd_OAMS_CALIBRATE_HUB_HES, self.cmd_OAMS_CALIBRATE_HUB_HES_help),
            ("OAMS_PID_AUTOTUNE", self.cmd_OAMS_PID_AUTOTUNE, self.cmd_OAMS_PID_AUTOTUNE_help),
            ("OAMS_PID_SET", self.cmd_OAMS_PID_SET, self.cmd_OAMS_PID_SET_help),
            ("OAMS_CURRENT_PID_SET", self.cmd_OAMS_CURRENT_PID_SET, self.cmd_OAMS_CURRENT_PID_SET_help),
            ("OAMS_ABORT_ACTION", self.cmd_OAMS_ABORT_ACTION, self.cmd_OAMS_ABORT_ACTION_help),
            ("OAMS_RETRY_STATUS", self.cmd_OAMS_RETRY_STATUS, self.cmd_OAMS_RETRY_STATUS_help),
            (
                "OAMS_RESET_RETRY_COUNTS",
                self.cmd_OAMS_RESET_RETRY_COUNTS,
                self.cmd_OAMS_RESET_RETRY_COUNTS_help,
            ),
        ]

        for cmd_name, handler, help_text in commands:
            gcode.register_mux_command(cmd_name, "OAMS", id, handler, desc=help_text)

    cmd_OAMS_RETRY_STATUS_help = "Display retry configuration and state"

    def cmd_OAMS_RETRY_STATUS(self, gcmd):
        msg_lines = [
            f"OAMS[{self.oams_idx}] Retry Status:",
            f"  Load retry max: {self.load_retry_max}",
            f"  Unload retry max: {self.unload_retry_max}",
            f"  Retry delay: {self.retry_delay:.1f}s",
            f"  Auto-unload on failed load: {self.auto_unload_on_failed_load}",
            f"  Current unload retry count: {self._unload_retry_count}",
        ]

        if self._load_retry_state:
            msg_lines.append("  Load retry counts:")

            for spool_idx, retry in sorted(self._load_retry_state.items()):
                msg_lines.append(
                    f"    Spool {spool_idx}: {retry.count}/{self.load_retry_max}"
                )
        else:
            msg_lines.append("  No active load retries")


        gcmd.respond_info("\n".join(msg_lines))

    cmd_OAMS_RESET_RETRY_COUNTS_help = "Reset retry counters"

    def cmd_OAMS_RESET_RETRY_COUNTS(self, gcmd):
        self._load_retry_state.clear()
        self._unload_retry_count = 0
        self._last_unload_attempt = 0.0
        self._last_successful_load.clear()
        gcmd.respond_info(f"OAMS[{self.oams_idx}]: Reset all retry counters")

    def _calculate_retry_delay(self, attempt_number: int) -> float:
        """Calculate constant retry delay."""
        return self.retry_delay

    def _reset_load_retry_count(self, spool_idx: int) -> None:
        """Clear retry tracking for a specific spool."""
        self._load_retry_state.pop(spool_idx, None)

    def _reset_unload_retry_count(self) -> None:
        """Clear unload retry tracking."""
        self._unload_retry_count = 0
        self._last_unload_attempt = 0.0

    def load_spool_with_retry(self, spool_idx: int, max_retries: Optional[int] = None) -> Tuple[bool, str]:
        """Load spool with automatic retry on failure.

        Args:
            spool_idx: Index of the spool to load
            max_retries: Override for load_retry_max. If provided, uses this value instead
                        of the configured load_retry_max. Allows AFC to control retry count.
        """
        retry = self._load_retry_state.setdefault(spool_idx, RetryState())
        retry_count = retry.count
        attempt_history = []  # Track failure reasons for diagnostic context

        # Use AFC's retry config if provided, otherwise fall back to OAMS config
        retry_limit = max_retries if max_retries is not None else self.load_retry_max

        # Use a loop instead of recursion to prevent monitor state issues
        while retry_count < retry_limit:
            if retry_count > 0:
                delay = self._calculate_retry_delay(retry_count)
                # Resolve lane name for retry message
                lane_name = self._resolve_lane_name(spool_idx)
                lane_label = f"lane {lane_name}" if lane_name else f"lane (spool {spool_idx})"
                self.logger.info(
                    f"OAMS[{self.oams_idx}]: Load retry {retry_count + 1}/{retry_limit} for {lane_label}, waiting {delay:.1f}s"
                )
                self.reactor.pause(self.reactor.monotonic() + delay)

            retry.count = retry_count + 1
            retry.last_attempt = self.reactor.monotonic()

            success, message = self.load_spool(spool_idx)

            if success:
                # Record successful load timestamp for stuck spool detection
                self._last_successful_load[spool_idx] = self.reactor.monotonic()
                # Track whether this load completed after retries
                retry.was_retry = retry_count > 0
                self._reset_load_retry_count(spool_idx)
                # Try to resolve lane name for better user messaging
                lane_name = self._resolve_lane_name(spool_idx)
                # Always use "lane X" terminology to align with AFC system
                lane_label = f"lane {lane_name}" if lane_name else f"lane (spool {spool_idx})"
                self.logger.info(
                    f"OAMS[{self.oams_idx}]: Successfully loaded {lane_label} on attempt {retry_count + 1}"
                )
                return True, message

            # Load failed - record attempt in history
            attempt_history.append(f"Attempt {retry_count + 1}: {message}")

            # Resolve lane name for error messages
            lane_name = self._resolve_lane_name(spool_idx)
            lane_label = f"lane {lane_name}" if lane_name else f"lane (spool {spool_idx})"

            if retry_count + 1 < retry_limit:
                self.logger.warning(
                    f"OAMS[{self.oams_idx}]: Load failed for {lane_label}: {message}. Attempt {retry_count + 1}/{retry_limit}"
                )

                if self.auto_unload_on_failed_load:
                    # Unload retry: Always use raw unload (no cut needed)
                    # This handles stuck spool during load - filament hasn't engaged with extruder yet
                    self.logger.info(f"OAMS[{self.oams_idx}]: Auto-unloading before retry (raw unload, no cut)")
                    unload_success, unload_msg = self.unload_spool_with_retry()

                    if not unload_success:
                        self.logger.error(
                            f"OAMS[{self.oams_idx}]: Failed to unload before retry: {unload_msg}"
                        )
                        # Abort load retry sequence if auto-unload fails
                        # Filament is likely stuck in the tube - continuing load attempts will fail
                        self._reset_load_retry_count(spool_idx)
                        # Record retry failure for monitoring
                        self._load_retry_failures += 1
                        self._last_load_failure_time = self.reactor.monotonic()
                        return False, (
                            f"Failed to unload {lane_label} back to AMS before retry. "
                            f"Load aborted after {retry_count + 1} attempts. {unload_msg}"
                        )

                # Increment and continue loop
                retry_count += 1
            else:
                # Max retries reached
                break

        self._reset_load_retry_count(spool_idx)
        # Record retry failure for monitoring
        self._load_retry_failures += 1
        self._last_load_failure_time = self.reactor.monotonic()

        # Resolve lane name for final error message
        lane_name = self._resolve_lane_name(spool_idx)
        lane_label = f"lane {lane_name}" if lane_name else f"lane (spool {spool_idx})"

        # Build detailed error message with attempt history
        history_str = "; ".join(attempt_history) if attempt_history else message
        return False, (
            f"Failed to load {lane_label} after {retry_limit} attempts. "
            f"Attempt history: {history_str}"
        )

    def get_last_load_attempt_time(self, spool_idx: int) -> Optional[float]:
        """Return timestamp of the most recent load attempt for spool."""
        retry = self._load_retry_state.get(spool_idx)
        return retry.last_attempt if retry is not None else None

    def get_last_successful_load_time(self, spool_idx: int) -> Optional[float]:
        """Return timestamp of the most recent successful load for spool."""
        return self._last_successful_load.get(spool_idx)

    def last_load_was_retry(self, spool_idx: int) -> bool:
        """
        Check if the last successful load for this spool was after retries.

        This is used by the manager to skip post-load pressure checks after retries,
        since pressure may not have stabilized yet and could cause false positives.
        """
        retry = self._load_retry_state.get(spool_idx)
        return retry.was_retry if retry is not None else False

    def unload_spool_with_retry(self, max_retries: Optional[int] = None) -> Tuple[bool, str]:
        """Unload spool with automatic retry on failure.

        Args:
            max_retries: Override for unload_retry_max. If provided, uses this value instead
                        of the configured unload_retry_max. Allows AFC to control retry count.
        """
        attempt_history = []  # Track failure reasons for diagnostic context

        # Use AFC's retry config if provided, otherwise fall back to OAMS config
        retry_limit = max_retries if max_retries is not None else self.unload_retry_max

        # Use a loop instead of recursion to prevent monitor state issues
        while self._unload_retry_count < retry_limit:
            if self._unload_retry_count > 0:
                delay = self._calculate_retry_delay(self._unload_retry_count)
                self.logger.info(
                    f"OAMS[{self.oams_idx}]: Unload retry {self._unload_retry_count + 1}/{retry_limit}, waiting {delay:.1f}s"
                )
                self.reactor.pause(self.reactor.monotonic() + delay)
                try:
                    gcode = self._cached_gcode
                    if gcode is None:
                        gcode = self.printer.lookup_object("gcode")
                        self._cached_gcode = gcode
                    gcode.run_script_from_command("M83")
                    gcode.run_script_from_command("G92 E0")
                    gcode.run_script_from_command("G1 E-5.00 F1200")
                    gcode.run_script_from_command("M400")
                except Exception as e:
                    self.logger.warning(
                        f"OAMS[{self.oams_idx}]: Failed to retract extruder before unload retry: {e}"
                    )

            self._unload_retry_count += 1
            attempt_number = self._unload_retry_count
            self._last_unload_attempt = self.reactor.monotonic()

            success, message = self.unload_spool()

            if success:
                self._reset_unload_retry_count()
                # Get lane name for better user messaging (use current_spool which was just unloaded)
                lane_name = None
                if self.current_spool is not None:
                    lane_name = self._resolve_lane_name(self.current_spool)
                # Always use "lane X" terminology to align with AFC system
                lane_label = f"lane {lane_name}" if lane_name else "filament"
                self.logger.info(
                    f"OAMS[{self.oams_idx}]: Successfully unloaded {lane_label} on attempt {attempt_number}"
                )
                return True, message

            # Unload failed - record attempt in history
            attempt_history.append(f"Attempt {self._unload_retry_count}: {message}")


            if self._unload_retry_count < retry_limit:
                self.logger.warning(
                    f"OAMS[{self.oams_idx}]: Unload failed: {message}. Attempt {self._unload_retry_count}/{retry_limit}"
                )
                # Continue loop for retry
            else:
                # Max retries reached
                break

        self._reset_unload_retry_count()
        # Record retry failure for monitoring
        self._unload_retry_failures += 1
        self._last_unload_failure_time = self.reactor.monotonic()

        # Build detailed error message with attempt history
        history_str = "; ".join(attempt_history) if attempt_history else message
        return False, (
            f"Failed to unload after {retry_limit} attempts. "
            f"Attempt history: {history_str}"
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

    cmd_OAMS_PID_AUTOTUNE_help = "Run PID autotune"
    def cmd_OAMS_PID_AUTOTUNE(self, gcmd):
        target_flow = gcmd.get_float("TARGET_FLOW", None)
        target_temp = gcmd.get_float("TARGET_TEMP", None)

        if target_flow is None:
            raise gcmd.error("TARGET flowrate in mm^3/s is required")

        if target_temp is None:
            raise gcmd.error("TARGET temperature in degrees C is required")


        extrusion_speed_per_min = (60 * target_flow / (pi * (1.75 / 2) ** 2))
        extrusion_length = (extrusion_speed_per_min / 60 * 30)

        # Use cached gcode object if available, otherwise lookup with None check
        gcode = self._cached_gcode or self.printer.lookup_object("gcode", None)
        if gcode is None:
            gcmd.error("Failed to access gcode object")
            return

        gcode.send("M104 S%f" % target_temp)
        gcode.send("G1 E%f F%f" % (extrusion_length, extrusion_speed_per_min))

    cmd_OAMS_CALIBRATE_HUB_HES_help = "Calibrate the range of a single hub HES"
    def cmd_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        self.action_status = OAMSStatus.CALIBRATING
        spool_idx = gcmd.get_int("SPOOL", None)
        if spool_idx is None:
            raise gcmd.error("SPOOL index is required")

        if spool_idx < 0 or spool_idx > 3:
            raise gcmd.error("Invalid SPOOL index")

        self.oams_calibrate_hub_hes_cmd.send([spool_idx])
        while self.action_status is not None:
            # Increased from 0.1s to 0.5s to reduce reactor pressure
            # Calibration is manual operation, 0.5s interval is perfectly acceptable
            self.reactor.pause(self.reactor.monotonic() + 0.5)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            value = self.u32_to_float(self.action_status_value)
            gcmd.respond_info("Calibrated HES %d to %f threshold" % (spool_idx, value))
            configfile = self.printer.lookup_object("configfile", None)
            if configfile is None:
                gcmd.error("Failed to access configfile object")
                return

            self.hub_hes_on[spool_idx] = value
            values = ",".join(map(str, self.hub_hes_on))
            configfile.set(self.name, "hub_hes_on", "%s" % (values,))
            gcmd.respond_info("Done calibrating HES; please note this value, and update parameter hub_hes_on for index %d in the configuration" % (spool_idx,))
        else:
            gcmd.error("Calibration of HES %d failed" % spool_idx)

    cmd_OAMS_CALIBRATE_PTFE_LENGTH_help = "Calibrate the length of the PTFE tube"
    def cmd_OAMS_CALIBRATE_PTFE_LENGTH(self, gcmd):
        self.action_status = OAMSStatus.CALIBRATING
        spool = gcmd.get_int("SPOOL", None)
        if spool is None:
            raise gcmd.error("SPOOL index is required")

        self.oams_calibrate_ptfe_length_cmd.send([spool])
        while self.action_status is not None:
            # Increased from 0.1s to 0.5s to reduce reactor pressure
            # Calibration is manual operation, 0.5s interval is perfectly acceptable
            self.reactor.pause(self.reactor.monotonic() + 0.5)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            gcmd.respond_info("Calibrated PTFE length to %d" % self.action_status_value)
            configfile = self.printer.lookup_object("configfile", None)
            if configfile is None:
                gcmd.error("Failed to access configfile object")
                return

            configfile.set(self.name, "ptfe_length", "%d" % (self.action_status_value,))
            gcmd.respond_info("Done calibrating clicks, please note this value and update parameter ptfe_length in the configuration")

        else:
            gcmd.error("Calibration of PTFE length failed")

    def load_spool(self, spool_idx):
        self.action_status = OAMSStatus.LOADING
        self.oams_load_spool_cmd.send([spool_idx])
        timeout = self.reactor.monotonic() + 30.0  # 30 second timeout

        # CRITICAL FIX: Use reactor.pause() to avoid queueing behind print moves
        # toolhead.dwell() queues into move queue and blocks during printing
        # reactor.pause() waits without affecting toolhead command stream
        while self.action_status is not None:
            if self.reactor.monotonic() > timeout:
                self.logger.error(f"OAMS[{self.oams_idx}]: Load operation timed out after 30 seconds")
                self.action_status = None
                self.action_status_code = OAMSOpCode.ERROR_UNSPECIFIED
                return False, "OAMS load operation timed out (MCU unresponsive)"
            # Use reactor.pause to wait without queueing into toolhead
            # Poll every 0.2s for responsive stuck detection while managing reactor load
            # 0.2s interval allows stuck detection abort to be noticed quickly (~1s)
            # while still reducing iterations by 80% compared to original 0.1s polling
            self.reactor.pause(self.reactor.monotonic() + 0.2)

        if self.action_status_code == OAMSOpCode.SUCCESS:
            self.current_spool = spool_idx
            return True, "Spool loaded successfully"
        elif self.action_status_code == OAMSOpCode.ERROR_KLIPPER_CALL:
            return False, "Spool loading stopped by klipper monitor"
        elif self.action_status_code == OAMSOpCode.ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS with code %d" % self.action_status_code

    cmd_OAMS_LOAD_SPOOL_help = "Load a new spool of filament"
    def cmd_OAMS_LOAD_SPOOL(self, gcmd):
        self.action_status = OAMSStatus.LOADING
        self.oams_spool_query_spool_cmd.send()
        spool_idx = gcmd.get_int("SPOOL", None)
        if spool_idx is None:
            raise gcmd.error("SPOOL index is required")

        if spool_idx < 0 or spool_idx > 3:
            raise gcmd.error("Invalid SPOOL index")

        
        quiet = gcmd.get_int("QUIET", 0)
        success, message = self.load_spool_with_retry(spool_idx)
        
        if success and not quiet:
            gcmd.respond_info(message)
        elif not success:
            gcmd.error(message)
            
    def unload_spool(self):
        self.action_status = OAMSStatus.UNLOADING
        self.oams_unload_spool_cmd.send()
        timeout = self.reactor.monotonic() + 60.0  # 60 second timeout (increased from 30s to handle MCU recovery)

        # CRITICAL FIX: Use reactor.pause() to avoid queueing behind print moves
        # toolhead.dwell() queues into move queue and blocks during printing
        # reactor.pause() waits without affecting toolhead command stream
        while self.action_status is not None:
            if self.reactor.monotonic() > timeout:
                self.logger.error(f"OAMS[{self.oams_idx}]: Unload operation timed out after 60 seconds")
                self.action_status = None
                self.action_status_code = OAMSOpCode.ERROR_UNSPECIFIED
                return False, "OAMS unload operation timed out (MCU unresponsive)"
            # Use reactor.pause to wait without queueing into toolhead
            # Poll every 0.2s for responsive stuck detection while managing reactor load
            # 0.2s interval allows stuck detection abort to be noticed quickly (~1s)
            # while still reducing iterations by 80% compared to original 0.1s polling
            self.reactor.pause(self.reactor.monotonic() + 0.2)

        if self.action_status_code == OAMSOpCode.SUCCESS:
            self.current_spool = None
            return True, "Spool unloaded successfully"
        elif self.action_status_code == OAMSOpCode.ERROR_KLIPPER_CALL:
            return False, "Spool unloading stopped by klipper monitor"
        elif self.action_status_code == OAMSOpCode.ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS"

    cmd_OAMS_UNLOAD_SPOOL_help = "Unload a spool of filament"
    def cmd_OAMS_UNLOAD_SPOOL(self, gcmd):
        max_retries = gcmd.get_int("MAX_RETRIES", None)
        success, message = self.unload_spool_with_retry(max_retries=max_retries)
        if success:
            gcmd.respond_info(message)
        else:
            gcmd.error(message)

    cmd_OAMS_ABORT_ACTION_help = "Abort the current OAMS action"
    def cmd_OAMS_ABORT_ACTION(self, gcmd):
        code = gcmd.get_int("CODE", OAMSOpCode.ERROR_KLIPPER_CALL)
        wait = gcmd.get_int("WAIT", 1)
        self.abort_current_action(code=code, wait=bool(wait))

    def set_oams_follower(self, enable, direction):
        self.oams_follower_cmd.send([enable, direction])

    def abort_current_action(
        self,
        code: int = OAMSOpCode.ERROR_KLIPPER_CALL,
        wait: bool = True,
    ) -> None:
        """Abort any in-flight hardware action initiated by Klipper helpers.

        IMPORTANT: This waits for the MCU to finish the current operation before
        clearing the action status. Simply clearing action_status without waiting
        creates a desync where Klipper thinks the operation is done but the MCU
        is still busy, causing "OAMS is busy" errors on subsequent operations.
        """
        if self.action_status is None:
            return

        if wait:
            self.logger.debug(
                f"OAMS[{self.oams_idx}]: Aborting current action {self.action_status} with code {code} - waiting for MCU to complete"
            )

        if wait:
            # Wait for MCU to finish the current operation (max 5 seconds)
            # The MCU will send an oams_action_status response when done
            timeout = self.reactor.monotonic() + 5.0
            pause_delay = 0.5
            while self.action_status is not None:
                if self.reactor.monotonic() > timeout:
                    self.logger.debug(
                        f"OAMS[{self.oams_idx}]: Abort timeout - MCU did not respond, forcing clear"
                    )
                    break
                # Use reactor.pause to wait without queueing into toolhead
                # Increased from 0.05s to 0.5s to reduce reactor pressure during MCU communication issues
                # Using 0.5s provides good balance between responsiveness and reducing MCU load
                self.reactor.pause(self.reactor.monotonic() + pause_delay)
                # Gradually stretch the pause during longer waits to avoid tight loops when firmware is unresponsive
                pause_delay = min(pause_delay + 0.25, 1.5)

            # Now clear the status (may already be None if MCU responded)
            self.action_status_code = code
            self.action_status_value = None
            self.action_status = None

            self.logger.info(f"OAMS[{self.oams_idx}]: Abort complete - ready for new operations")
        else:
            # CRITICAL FIX: Must clear action_status even when not waiting
            # Otherwise load_spool/unload_spool will keep polling until timeout
            # The "while self.action_status is not None" loop needs this cleared
            self.action_status_code = code
            self.action_status_value = None
            self.action_status = None
            self.logger.debug(
                f"OAMS[{self.oams_idx}]: Abort requested without waiting - status cleared immediately"
            )
    cmd_OAMS_FOLLOWER_help = "Enable or disable follower and set its direction"
    def cmd_OAMS_FOLLOWER(self, gcmd):
        enable = gcmd.get_int("ENABLE", None)
        if enable is None:
            raise gcmd.error("ENABLE is required")

        direction = gcmd.get_int("DIRECTION", None)
        if direction is None:
            raise gcmd.error("DIRECTION is required")

        self.set_oams_follower(enable, direction)
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
        
    def _oams_cmd_current_status(self, params):
        self.i_value = self.u32_to_float(params["current_value"])

    def get_current(self):
        return self.i_value

    def _oams_action_status(self, params):
        self.logger.debug("OAMS status received")

        
        action = params["action"]
        code = params["code"]
        
        if action in (OAMSStatus.LOADING, OAMSStatus.UNLOADING, OAMSStatus.ERROR):
            self.action_status = None
            self.action_status_code = code
        elif action == OAMSStatus.CALIBRATING:
            self.action_status = None
            self.action_status_code = code
            self.action_status_value = params["value"]
        elif code == OAMSOpCode.ERROR_KLIPPER_CALL:
            self.action_status = None
            self.action_status_code = code
        elif action in (
            OAMSStatus.FORWARD_FOLLOWING,
            OAMSStatus.REVERSE_FOLLOWING,
            OAMSStatus.COASTING,
            OAMSStatus.STOPPED,
        ):
            self.logger.debug(
                f"OAMS status update (non-action) code={code} action={action}"
            )
        else:
            self.logger.debug(
                f"OAMS status update (unhandled) code={code} action={action}"
            )

    def float_to_u32(self, f: float) -> int:
        """Convert float to u32 using pre-compiled struct"""
        return _U32_STRUCT.unpack(_FLOAT_STRUCT.pack(f))[0]

    def u32_to_float(self, i: int) -> float:
        """Convert u32 to float using pre-compiled struct"""
        return _FLOAT_STRUCT.unpack(_U32_STRUCT.pack(i))[0]

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
