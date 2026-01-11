# Filament Buffer Pressure Sensor
#
# Copyright (C) 2023-2025 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from typing import List, Callable, Optional, Any

class FPS:
    """
    Filament Pressure Sensor - Monitors filament pressure and interfaces with OAMS units.
    
    State Variables:
    - fps_value: Current pressure sensor reading (0.0-1.0)
    - extruder: Reference to the associated extruder object
    - oams: List of OAMS units this FPS can work with
    """
    
    def __init__(self, config):
        # Core objects
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        self.printer.add_object(self.name, self)
        
        # Current sensor state
        self.fps_value: float = 0.0  # Pressure reading (0.0-1.0)
        
        # Hardware configuration
        self._pin: str = config.get('pin')
        self._sample_count: int = config.getint('sample_count', 5)
        self._sample_time: float = config.getfloat('sample_time', 0.005)
        self._report_time: float = config.getfloat('report_time', 0.100)
        self._reversed: bool = config.getboolean('reversed', False)

        # Motion parameters
        self._sf_max_speed: float = config.getfloat('max_speed', 300.0)
        self._accel: float = config.getfloat('accel', 0.0)
        self._set_point: float = config.getfloat('set_point', 0.5)

        # Runout monitoring configuration
        #
        # This optional safety margin allows integrators to fine tune when the
        # runout monitor should reload for this specific FPS. When omitted, the
        # global margin from [oams_manager] is used instead.
        self.reload_before_toolhead_distance: Optional[float] = config.getfloat(
            'reload_before_toolhead_distance',
            default=None,
        )
        
        # Associated objects
        self.extruder_name: str = config.get('extruder')
        self.extruder = None  # Will be set in on_ready()
        
        # Parse OAMS names and store references
        self.oams_names: List[str] = config.get('oams').split(',')
        self.oams: List[Any] = []  # List of OAMS objects this FPS works with
        
        # Initialize OAMS references
        for name in self.oams_names:
            oam = self.printer.lookup_object("oams " + name.strip())
            if oam is None:
                raise ValueError(f"OAMS object '{name}' not found")
            self.oams.append(oam)
        
        # Hardware interfaces
        self.ppins = None
        self.adc = None
        self.callbacks: List[Callable] = []  # List of callback functions for value changes

        # Initialize ADC hardware interface
        self.ppins = self.printer.lookup_object('pins')
        self.adc = self.ppins.setup_pin('adc', self._pin)
        
        # Setup ADC sampling based on Kalico vs standard Klipper
        self.use_kalico: bool = config.getboolean('use_kalico', False)
        if self.use_kalico:
            self.adc.setup_minmax(self._sample_time, self._sample_count)
        else:
            self.adc.setup_adc_sample(self._sample_time, self._sample_count)
        self.adc.setup_adc_callback(self._report_time, self._adc_callback)
        
        # Register event handlers
        self.printer.register_event_handler("klippy:ready", self.on_ready)
        
    def on_ready(self) -> None:
        """Initialize extruder reference when printer is ready."""
        self.extruder = self.printer.lookup_object(self.extruder_name)
        if self.extruder is None:
            raise ValueError(f"Extruder '{self.extruder_name}' not found")
        
    def add_callback(self, callback: Callable) -> None:
        """Add a callback function to be called when FPS value changes."""
        self.callbacks.append(callback)

    def _adc_callback(self, read_time: float, read_value: float) -> None:
        """Process new ADC reading and notify callbacks."""
        if self._reversed:
            read_value = 1.0 - read_value
        self.fps_value = read_value
        
        # Notify all registered callbacks
        if self.callbacks:
            for callback in self.callbacks:
                callback(read_time, read_value)
        
    def get_status(self, eventtime: float) -> dict:
        """Return current FPS status for monitoring."""
        return {
            'fps_value': self.fps_value,
        }
    
    def get_value(self) -> float:
        """Get current pressure sensor value (0.0-1.0)."""
        return self.fps_value
            
def load_config_prefix(config):
    return FPS(config)