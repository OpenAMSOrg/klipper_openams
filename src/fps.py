# Filament Buffer Pressure Sensor (FPS) — one feed-path lane
#
# Copyright (C) 2023-2026 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# An FPS is the pressure sensor + buffer that joins a lane's OAMS units to one
# toolhead extruder. The pressure value is produced by the FPS hardware and read
# here via the FPS MCU's ADC (host-side view; the OAMS firmware also reads the
# same analog signal locally to run its hub-motor control). Multiple FPS may be
# defined (one per toolhead) using named sections [fps <name>]; a single unnamed
# [fps] also works for the common one-lane case.


class FPS:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()              # "fps" or "fps <name>"
        self.fps_name = self.name.split()[-1]      # short lane key

        self.fps_value = 0.0

        self._pin = config.get('pin')
        self._sample_count = config.getint('sample_count', 5)
        self._sample_time = config.getfloat('sample_time', 0.005)
        self._report_time = config.getfloat('report_time', 0.100)
        self._reversed = config.getboolean('reversed', False)

        # Reserved tuning parameters (kept for config compatibility / future use).
        self._sf_max_speed = config.getfloat('max_speed', 300.0)
        self._accel = config.getfloat('accel', 0.0)
        self._set_point = config.getfloat('set_point', 0.5)
        # Accepted for config compatibility; the ADC API is now detected from
        # the running Klipper/Kalico instead of trusting this flag.
        config.getboolean('use_kalico', False)

        # The toolhead extruder this lane feeds (resolved at connect).
        self.extruder_name = config.get('extruder', 'extruder')
        self.extruder = None

        self.ppins = self.printer.lookup_object('pins')
        self.adc = self.ppins.setup_pin('adc', self._pin)
        if hasattr(self.adc, 'setup_adc_sample'):
            # Current mainline Klipper API:
            #   setup_adc_sample(report_time, sample_time, sample_count)
            #   setup_adc_callback(callback)  # list of (time, value) samples
            self.adc.setup_adc_sample(self._report_time,
                                      sample_time=self._sample_time,
                                      sample_count=self._sample_count)
            self.adc.setup_adc_callback(self._adc_callback)
        else:
            # Kalico / pre-2024 mainline MCU_adc API:
            #   setup_minmax(sample_time, sample_count, ...)
            #   setup_adc_callback(report_time, callback)  # scalar value
            self.adc.setup_minmax(self._sample_time, self._sample_count)
            self.adc.setup_adc_callback(self._report_time,
                                        self._adc_callback_scalar)

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)

    def _handle_connect(self):
        self.extruder = self.printer.lookup_object(self.extruder_name, None)
        if self.extruder is None:
            raise self.printer.config_error(
                "[%s]: extruder '%s' not found; check the 'extruder:' option"
                % (self.name, self.extruder_name))

    def _adc_callback(self, samples):
        # Mainline Klipper delivers a list of (read_time, value); use the latest.
        read_time, read_value = samples[-1]
        self._update_value(read_value)

    def _adc_callback_scalar(self, read_time, read_value):
        # Kalico delivers one (read_time, value) pair per report.
        self._update_value(read_value)

    def _update_value(self, read_value):
        if self._reversed:
            read_value = 1.0 - read_value
        self.fps_value = read_value

    def get_status(self, eventtime):
        return {'fps_value': self.fps_value}

    def get_value(self):
        return self.fps_value


def load_config(config):
    # Unnamed [fps] (single-lane setups).
    return FPS(config)


def load_config_prefix(config):
    # Named [fps <name>] (multi-lane / IDEX / multi-tool setups).
    return FPS(config)
