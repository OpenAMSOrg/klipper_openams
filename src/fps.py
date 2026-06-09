# Filament Buffer Pressure Sensor
#
# Copyright (C) 2023-2026 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class FPS:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.printer.add_object(self.name, self)

        # state
        self.fps_value = 0.0
        self.callbacks = []

        self._pin = config.get('pin')
        self._sample_count = config.getint('sample_count', 5)
        self._sample_time = config.getfloat('sample_time', 0.005)
        self._report_time = config.getfloat('report_time', 0.100)
        self._reversed = config.getboolean('reversed', False)

        # Reserved tuning parameters (kept for config compatibility / future use).
        self._sf_max_speed = config.getfloat('max_speed', 300.0)
        self._accel = config.getfloat('accel', 0.0)
        self._set_point = config.getfloat('set_point', 0.5)
        # Accepted for compatibility with Kalico-based setups. Both mainline
        # Klipper and Kalico expose the same MCU_adc.setup_adc_sample API, so it
        # no longer changes behaviour; retained so existing configs don't error.
        self._use_kalico = config.getboolean('use_kalico', False)

        self.ppins = self.printer.lookup_object('pins')
        self.adc = self.ppins.setup_pin('adc', self._pin)
        # Klipper signature: setup_adc_sample(report_time, sample_time, sample_count)
        self.adc.setup_adc_sample(self._report_time,
                                  sample_time=self._sample_time,
                                  sample_count=self._sample_count)
        self.adc.setup_adc_callback(self._adc_callback)

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def _adc_callback(self, samples):
        # Klipper delivers a list of (read_time, value) samples; use the latest.
        read_time, read_value = samples[-1]
        if self._reversed:
            read_value = 1.0 - read_value
        self.fps_value = read_value
        for callback in self.callbacks:
            callback(read_time, read_value)

    def get_status(self, eventtime):
        return {'fps_value': self.fps_value}

    def get_value(self):
        return self.fps_value


def load_config(config):
    return FPS(config)
