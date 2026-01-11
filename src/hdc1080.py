# Driver for the Texas Instruments HDC1080 temperature and humidity sensor
# Copyright (C) 2024 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from . import bus

# device register values that can be written to (do not write to others)
# used to specify location (pointer) to read from/write to
TEMP_REG = 0x00  # temperature register
HUMI_REG = 0x01  # humidity register
CONF_REG = 0x02  # configuration register
FSER_REG = 0xFB  # first two bytes of serial ID register
MSER_REG = 0xFC  # middle two bytes of serial ID register
LSER_REG = 0xFD  # last two bytes of serial ID register
MFID_REG = 0xFE  # manufacturer ID register
DVID_REG = 0xFF  # device ID register

HDC1080_I2C_ADDR = 0x40 # Device address, 64

CONFIG_RESET_BIT = 0x8000 # Reset bit
CONFIG_BATTERY_STATUS_BIT = 0x0800 # Battery status bit
HEATER_ENABLE_BIT = 0x2000 # Heater enable bit

TEMP_RES_14 = 0x0000 # 14 bit temperature resolution
TEMP_RES_11 = 0x0400 # 11 bit temperature resolution

TEMP_RES = {14 : TEMP_RES_14, 11: TEMP_RES_11}

HUMI_RES_14 = 0x0000 # 14 bit humidity resolution
HUMI_RES_11 = 0x0100 # 11 bit humidity resolution
HUMI_RES_8  = 0x0200 # 8 bit humidity resolution

HUMI_RES = {14: HUMI_RES_14, 11: HUMI_RES_11,8:  HUMI_RES_8}

class HDC1080:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=HDC1080_I2C_ADDR, default_speed=100000)
        self.report_time = config.getint('HDC1080_report_time',5,minval=5)
        self.temp = self.min_temp = self.max_temp = self.humidity = 0.
        self.sample_timer = self.reactor.register_timer(self._sample_hdc1080)
        self.printer.add_object("hdc1080 " + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.temp_resolution = config.getint('temp_resolution',14,minval=11,maxval=14)
        if self.temp_resolution not in list(TEMP_RES.keys()):
            raise ValueError("Invalid temperature resolution, valid values are %s " % (", ".join(str(x) for x in list(TEMP_RES.keys()))))
        self.temp_resolution = TEMP_RES[self.temp_resolution]
        
        self.humidity_resolution = config.getint('humidity_resolution',14,minval=8,maxval=14)
        if self.humidity_resolution not in list(HUMI_RES.keys()):
            raise ValueError("Invalid humidity resolution, valid values are %s " % (", ".join(str(x) for x in list(HUMI_RES.keys()))))
        self.humidity_resolution = HUMI_RES[self.humidity_resolution]
        
        self.temp_offset = config.getfloat('temp_offset',0.0)
        self.humidity_offset = config.getfloat('humidity_offset',0.0)
        self.heater_enabled = config.getboolean('heater_enabled',False)
                
        self.is_calibrated  = False
        self.init_sent = False

    def handle_connect(self):
        self._init_device()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time
    
    def _init_device(self):
        # Reset the device
        data = [CONF_REG, 1 << 4, 0x00]
        self.i2c.i2c_write(data)
        manufacturer_id = self.read_manufacturer_id()
        device_id = self.read_device_id()
        serial_id = self.read_serial_id()
        
        self.set_temperature_resolution(self.temp_resolution)
        self.set_humidity_resolution(self.humidity_resolution)
        
        if self.heater_enabled:
            self.turn_heater_on()
        
        temp_resolution = {v: k for k, v in TEMP_RES.items()}[self.temp_resolution]
        humidity_resolution = {v: k for k, v in HUMI_RES.items()}[self.humidity_resolution]
        
        battery_ok = self.get_battery_status()
        heater_enabled = self.get_heater_status()
        
        
        logging.info("hdc1080: manufacturer_id: %s, device_id: %s, serial_id: %s", hex(manufacturer_id), hex(device_id), hex(serial_id))
        logging.info("hdc1080: temp_resolution: %s-bit, humidity_resolution: %s-bit", temp_resolution, humidity_resolution)
        logging.info("hdc1080: battery_ok: %s heater_enabled: %s", str(battery_ok).lower(), str(heater_enabled).lower())
        
        self.init_sent = True
        
    def _read_temp(self, celsius=True):
        """ Read the temperature

        Keyword arguments:
        celsius -- If the data is kept as celsius after reading (default False)
        """
        # write to the pointer register, changing it to the temperature register
        try:
            self.i2c.i2c_write([TEMP_REG])
            self.reactor.pause(self.reactor.monotonic() + .0635)
            read = self.i2c.i2c_read([], 2)
            data = bytearray(read['response'])
            temp = (data[0] * 256) + data[1]
            cTemp = (temp / 65536.0) * 165.0 - 40
            return cTemp
        except:
            return 0.0
        
    def _read_humi(self):
        try:
            self.i2c.i2c_write([HUMI_REG])
            self.reactor.pause(self.reactor.monotonic() + .0635)
            read = self.i2c.i2c_read([], 2)
            data = bytearray(read['response'])
            humidity = (data[0] * 256) + data[1]
            percentHumidity = (humidity / 65536.0) * 100.0
            return percentHumidity
        except:
            return 0.0

    def _make_measurements(self):
        if not self.init_sent:
            logging.info("hdc1080: init not sent")
            return False
        
        self.temp = self._read_temp() + self.temp_offset
        self.reactor.pause(self.reactor.monotonic() + .015)
        self.humidity = self._read_humi() + self.humidity_offset
        
        #logging.info("hdc1080: temp: %s, humi: %s", self.temp, self.humidity)

        return True

    def _sample_hdc1080(self, eventtime):
        if not self._make_measurements():
            self.temp = self.humidity = .0
            return self.reactor.NEVER

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "HDC1080 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        self._callback(print_time, self.temp)
        return measured_time + self.report_time
    
    def read_device_id(self):
        self.i2c.i2c_write([DVID_REG])
        self.reactor.pause(self.reactor.monotonic() + .0635)
        read = self.i2c.i2c_read([], 2)
        data = bytearray(read['response'])
        device_id = (data[0] * 256) + data[1]
        return device_id
    
    def read_manufacturer_id(self):
        self.i2c.i2c_write([MFID_REG])
        self.reactor.pause(self.reactor.monotonic() + .0635)
        read = self.i2c.i2c_read([], 2)
        data = bytearray(read['response'])
        manufacturer_id = (data[0] * 256) + data[1]
        return manufacturer_id
    
    def read_serial_id(self):
        self.i2c.i2c_write([FSER_REG])
        self.reactor.pause(self.reactor.monotonic() + .0635)
        read = self.i2c.i2c_read([], 2)
        data = bytearray(read['response'])
        serial_id = (data[0] * 256) + data[1]
        
        self.i2c.i2c_write([MSER_REG])
        self.reactor.pause(self.reactor.monotonic() + .0635)
        read = self.i2c.i2c_read([], 2)
        data = bytearray(read['response'])
        serial_id = serial_id*256 + (data[0] * 256) + data[1]
        
        self.i2c.i2c_write([LSER_REG])
        self.reactor.pause(self.reactor.monotonic() + .0635)
        read = self.i2c.i2c_read([], 2)
        data = bytearray(read['response'])
        serial_id = serial_id*256 + (data[0] * 256) + data[1]
        
        return serial_id
    
    def read_config(self):
        self.i2c.i2c_write([CONF_REG])
        self.reactor.pause(self.reactor.monotonic() + .0635)
        read = self.i2c.i2c_read([], 2)
        data = bytearray(read['response'])
        config = (data[0] * 256) + data[1]
        return config
    
    def set_humidity_resolution(self, resolution):
        if not resolution in list(HUMI_RES.values()):
            raise ValueError("Invalid humidity resolution, valid values are %s " % (", ".join(str(x) for x in HUMI_RES)))
        
        config = self.read_config()
        config = (config & ~0x0300) | resolution 
        data = [CONF_REG, config >> 8, 0x00]
        self.i2c.i2c_write(data)
        self.reactor.pause(self.reactor.monotonic() + .015)
        
    def set_temperature_resolution(self, resolution):
        if not resolution in list(TEMP_RES.values()):
            raise ValueError("Invalid temperature resolution, valid values are %s" % (", ".join(str(x) for x in TEMP_RES)))
        
        config = self.read_config()
        config = (config & ~0x0400) | resolution 
        data = [CONF_REG, config >> 8, 0x00]
        self.i2c.i2c_write(data)
        self.reactor.pause(self.reactor.monotonic() + .015)
        
    def turn_heater_on(self):
        config = self.read_config()
        config = config | HEATER_ENABLE_BIT
        data = [CONF_REG, config >> 8, 0x00]
        self.i2c.i2c_write(data)
        self.reactor.pause(self.reactor.monotonic() + .015)
        
    def turn_heater_off(self):
        config = self.read_config()
        config = config & ~HEATER_ENABLE_BIT
        data = [CONF_REG, config >> 8, 0x00]
        self.i2c.i2c_write(data)
        self.reactor.pause(self.reactor.monotonic() + .015)
        
    def get_battery_status(self):
        config = self.read_config()
        return not bool((config & CONFIG_BATTERY_STATUS_BIT) >> 11)
    
    def get_heater_status(self):
        config = self.read_config()
        return bool((config & HEATER_ENABLE_BIT) >> 13)

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
            'humidity': self.humidity,
        }

def load_config(config):
    # Register sensor
    pheater = config.get_printer().lookup_object("heaters")
    pheater.add_sensor_factory("HDC1080", HDC1080)
