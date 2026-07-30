# OpenAMS dual MFRC522 reader over one Klipper software-SPI bus.
# Copyright (C) 2026 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


import json
import logging
import re

try:
    from . import bus
except (ImportError, ValueError):
    import bus


_RFID_DEBUG_RESPONSE = (
    "oams_rfid_debug_response gpiob_moder=%u gpiob_levels=%u "
    "gpiod_moder=%u gpiod_levels=%u spi0_config=%u spi0_last=%u "
    "spi1_config=%u spi1_last=%u")
_RFID_TRANSFER_COMMAND = (
    "oams_rfid_transfer oid=%c reader=%c data=%*s")
_RFID_TRANSFER_RESPONSE = (
    "oams_rfid_transfer_response oid=%c reader=%c ok=%c response=%*s")
_RFID_RESET_COMMAND = (
    "oams_rfid_reset oid=%c reader=%c enabled=%c")
_RFID_RESET_RESPONSE = (
    "oams_rfid_reset_response oid=%c reader=%c enabled=%c ok=%c")


class Mfrc522Error(Exception):
    pass


class Mfrc522NoCard(Mfrc522Error):
    pass


class Mfrc522:
    # Page 0: command and status registers
    COMMAND = 0x01
    COM_IRQ = 0x04
    ERROR = 0x06
    STATUS2 = 0x08
    FIFO_DATA = 0x09
    FIFO_LEVEL = 0x0A
    CONTROL = 0x0C
    BIT_FRAMING = 0x0D
    COLL = 0x0E

    # Page 1: configuration registers
    MODE = 0x11
    TX_CONTROL = 0x14
    TX_ASK = 0x15

    # Page 2: timer and CRC registers
    CRC_RESULT_H = 0x21
    CRC_RESULT_L = 0x22
    T_MODE = 0x2A
    T_PRESCALER = 0x2B
    T_RELOAD_H = 0x2C
    T_RELOAD_L = 0x2D

    VERSION = 0x37

    CMD_IDLE = 0x00
    CMD_CALC_CRC = 0x03
    CMD_TRANSCEIVE = 0x0C
    CMD_MF_AUTHENT = 0x0E
    CMD_SOFT_RESET = 0x0F

    PICC_REQA = 0x26
    PICC_ANTICOLL = (0x93, 0x95, 0x97)
    PICC_MF_AUTH_KEY_A = 0x60
    PICC_MF_AUTH_KEY_B = 0x61
    PICC_MF_READ = 0x30

    # FM17580 VersionReg is a production-version byte with no fixed reset
    # value. OpenAMS hardware has been observed with both A1 and EE.
    VALID_VERSIONS = (0x88, 0x90, 0x91, 0x92, 0xA1, 0xB2, 0xEE)
    def __init__(self, spi, reactor=None):
        self.spi = spi
        self.reactor = reactor

    # These two methods are the transport-neutral contract used by the
    # OpenAMS RFID stack.
    def reg_read(self, reg):
        address = 0x80 | ((int(reg) << 1) & 0x7E)
        params = self.spi.spi_transfer([address, 0x00])
        response = params.get("response", b"")
        if len(response) != 2:
            raise Mfrc522Error("short SPI response while reading register 0x%02x" % reg)
        value = response[1]
        if not isinstance(value, int):
            value = ord(value)
        return value

    def reg_write(self, reg, value):
        address = (int(reg) << 1) & 0x7E
        self.spi.spi_send([address, int(value) & 0xFF])

    def reader_power(self, on):
        # OpenAMS has no separate coil-enable line. TxControlReg owns the RF
        # field; this method intentionally only satisfies the AFC link API.
        return None

    def _set_bits(self, reg, mask):
        self.reg_write(reg, self.reg_read(reg) | mask)

    def _clear_bits(self, reg, mask):
        self.reg_write(reg, self.reg_read(reg) & ~mask)

    def initialize(self):
        self.reg_write(self.COMMAND, self.CMD_SOFT_RESET)
        command_value = None
        for _ in range(50):
            command_value = self.reg_read(self.COMMAND)
            if not command_value & (1 << 4):
                break
            self._pause(0.001)
        else:
            raise Mfrc522Error(
                "soft reset timed out (CommandReg=0x%02x)" % command_value)

        # Proven FM17580/MFRC522 timer setup used by the AFC reader stack.
        self.reg_write(self.T_MODE, 0x8D)
        self.reg_write(self.T_PRESCALER, 0x3E)
        self.reg_write(self.T_RELOAD_H, 0x00)
        self.reg_write(self.T_RELOAD_L, 30)
        self.reg_write(self.TX_ASK, 0x40)
        self.reg_write(self.MODE, 0x3D)
        self.antenna_on()

        version = self.reg_read(self.VERSION)
        if version not in self.VALID_VERSIONS:
            raise Mfrc522Error("unexpected VersionReg value 0x%02x" % version)
        return version

    def antenna_on(self):
        if self.reg_read(self.TX_CONTROL) & 0x03 != 0x03:
            self._set_bits(self.TX_CONTROL, 0x03)

    def antenna_off(self):
        self._clear_bits(self.TX_CONTROL, 0x03)

    def _pause(self, seconds):
        if self.reactor is not None:
            self.reactor.pause(self.reactor.monotonic() + seconds)

    def _calculate_crc(self, data):
        # ISO14443-A CRC (poly 0x8408, init 0x6363). Doing this on the host is
        # dramatically cheaper than polling CalcCRC over CAN register reads.
        crc = 0x6363
        for value in data:
            value ^= crc & 0xFF
            value = (value ^ (value << 4)) & 0xFF
            crc = ((crc >> 8) ^ (value << 8) ^ (value << 3)
                   ^ (value >> 4)) & 0xFFFF
        return [crc & 0xFF, crc >> 8]

    def _communicate(self, command, send_data, tx_last_bits=0,
                     rx_align=0, check_crc=False):
        wait_irq = 0x10 if command == self.CMD_MF_AUTHENT else 0x30
        self.reg_write(self.COMMAND, self.CMD_IDLE)
        self.reg_write(self.COM_IRQ, 0x7F)
        self._set_bits(self.FIFO_LEVEL, 0x80)
        for value in send_data:
            self.reg_write(self.FIFO_DATA, value)
        self.reg_write(self.BIT_FRAMING,
                       ((rx_align & 7) << 4) | (tx_last_bits & 7))
        self.reg_write(self.COMMAND, command)
        if command == self.CMD_TRANSCEIVE:
            self._set_bits(self.BIT_FRAMING, 0x80)

        for _ in range(100):
            irq = self.reg_read(self.COM_IRQ)
            if irq & wait_irq:
                break
            if irq & 0x01:
                raise Mfrc522NoCard("no card detected")
            self._pause(0.0005)
        else:
            raise Mfrc522Error("card communication did not complete")

        if command == self.CMD_TRANSCEIVE:
            self._clear_bits(self.BIT_FRAMING, 0x80)
        error = self.reg_read(self.ERROR)
        if error & 0x13:
            raise Mfrc522Error("MFRC522 ErrorReg=0x%02x" % error)

        count = self.reg_read(self.FIFO_LEVEL)
        if count > 64:
            raise Mfrc522Error("invalid FIFO length %d" % count)
        response = [self.reg_read(self.FIFO_DATA) for _ in range(count)]
        valid_bits = self.reg_read(self.CONTROL) & 0x07
        bit_length = (count - 1) * 8 + valid_bits if valid_bits else count * 8

        if check_crc:
            if len(response) < 2 or bit_length % 8:
                raise Mfrc522Error("response has no valid CRC")
            if self._calculate_crc(response[:-2]) != response[-2:]:
                raise Mfrc522Error("response CRC mismatch")
        return response, bit_length

    def request(self):
        response, bits = self._communicate(
            self.CMD_TRANSCEIVE, [self.PICC_REQA], tx_last_bits=7)
        if len(response) != 2 or bits != 16:
            raise Mfrc522Error("invalid ATQA response")
        return response

    def read_uid(self):
        self.request()
        uid = []
        for cascade_command in self.PICC_ANTICOLL:
            self.reg_write(self.COLL, 0x80)
            part, bits = self._communicate(
                self.CMD_TRANSCEIVE, [cascade_command, 0x20])
            if len(part) != 5 or bits != 40:
                raise Mfrc522Error("invalid anticollision response")
            if part[0] ^ part[1] ^ part[2] ^ part[3] != part[4]:
                raise Mfrc522Error("UID BCC mismatch")

            select = [cascade_command, 0x70] + part
            select += self._calculate_crc(select)
            sak, _ = self._communicate(
                self.CMD_TRANSCEIVE, select, check_crc=True)
            if len(sak) != 3:
                raise Mfrc522Error("invalid SAK response")

            if part[0] == 0x88:
                uid.extend(part[1:4])
            else:
                uid.extend(part[:4])
            if not sak[0] & 0x04:
                return bytes(uid)
        raise Mfrc522Error("UID cascade did not terminate")

    def authenticate(self, block, key, uid, key_b=False):
        if len(key) != 6 or len(uid) < 4:
            raise ValueError("MIFARE authentication requires a 6-byte key and UID")
        auth = self.PICC_MF_AUTH_KEY_B if key_b else self.PICC_MF_AUTH_KEY_A
        frame = [auth, block] + list(key) + list(uid[-4:])
        self._communicate(self.CMD_MF_AUTHENT, frame)
        if not self.reg_read(self.STATUS2) & 0x08:
            raise Mfrc522Error("MIFARE authentication failed")

    def stop_crypto(self):
        self._clear_bits(self.STATUS2, 0x08)

    def read_block(self, block, key, uid, key_b=False):
        self.authenticate(block, key, uid, key_b)
        try:
            frame = [self.PICC_MF_READ, block]
            frame += self._calculate_crc(frame)
            data, bits = self._communicate(
                self.CMD_TRANSCEIVE, frame, check_crc=True)
            if len(data) != 18 or bits != 144:
                raise Mfrc522Error("invalid MIFARE block response")
            return bytes(data[:16])
        finally:
            self.stop_crypto()


class OpenAmsRfidSpiBus:
    """One no-CS Klipper SPI object with firmware-controlled reader selection."""

    def __init__(self, spi):
        self.spi = spi
        self.oid = spi.get_oid()
        self.mcu = spi.get_mcu()
        self.command_queue = spi.get_command_queue()
        self.transfer_command = None
        self.reset_command = None

    def _lookup_commands(self):
        if self.transfer_command is not None:
            return
        self.transfer_command = self.mcu.lookup_query_command(
            _RFID_TRANSFER_COMMAND, _RFID_TRANSFER_RESPONSE,
            oid=self.oid, cq=self.command_queue)
        self.reset_command = self.mcu.lookup_query_command(
            _RFID_RESET_COMMAND, _RFID_RESET_RESPONSE,
            oid=self.oid, cq=self.command_queue)

    def transfer(self, reader, data):
        self._lookup_commands()
        params = self.transfer_command.send(
            [self.oid, int(reader), list(data)])
        if not params.get("ok", 0):
            raise Mfrc522Error(
                "firmware rejected RFID transfer for reader %d" % reader)
        return params

    def set_reset(self, reader, enabled):
        self._lookup_commands()
        params = self.reset_command.send(
            [self.oid, int(reader), bool(enabled)])
        if not params.get("ok", 0):
            raise Mfrc522Error(
                "firmware rejected RFID reset for reader %d" % reader)


class OpenAmsRfidSpiEndpoint:
    """Reader-indexed view of the one shared OpenAMS SPI transport."""

    def __init__(self, shared_bus, reader):
        self.shared_bus = shared_bus
        self.reader = reader

    def spi_transfer(self, data):
        return self.shared_bus.transfer(self.reader, data)

    def spi_send(self, data):
        self.shared_bus.transfer(self.reader, data)

    def get_mcu(self):
        return self.shared_bus.mcu

    def get_command_queue(self):
        return self.shared_bus.command_queue


class OpenAmsRfidRegistry:
    def __init__(self, printer):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.readers = {}
        self.drivers = {}
        printer.lookup_object("gcode").register_command(
            "OAMSM_RFID_READ", self.cmd_RFID_READ,
            desc="Read an OpenAMS RFID card reader")
        printer.register_event_handler("klippy:ready", self._handle_ready)

    def add_driver(self, driver, config):
        if driver.oams_index in self.drivers:
            raise config.error(
                "duplicate MFRC522 driver for OAMS=%d" % driver.oams_index)
        self.drivers[driver.oams_index] = driver
        for reader in driver.readers:
            key = (reader.oams_index, reader.rfid_card)
            if key in self.readers:
                raise config.error(
                    "duplicate MFRC522 mapping for OAMS=%d RFID_CARD=%d" % key)
            self.readers[key] = reader

    def _handle_ready(self):
        # SPI queries wait for MCU responses, so defer them out of Klippy's
        # pause-disabled ready handler. Each dual-reader driver owns one callback
        # and one poll timer for its shared bus.
        self.reactor.register_callback(self._handle_start)

    def _handle_start(self, eventtime):
        drivers = list(self.drivers.values())
        for driver in drivers:
            driver._initialize(eventtime)
        start_time = self.reactor.monotonic()
        for driver in drivers:
            self.reactor.register_timer(
                driver._poll, start_time + driver.poll_interval)

    def cmd_RFID_READ(self, gcmd):
        oams_index = gcmd.get_int("OAMS", None, minval=0)
        rfid_card = gcmd.get_int("RFID_CARD", None, minval=0, maxval=1)
        if oams_index is None or rfid_card is None:
            raise gcmd.error(
                "Usage: OAMSM_RFID_READ OAMS=<index> RFID_CARD=<0|1>")
        reader = self.readers.get((oams_index, rfid_card))
        if reader is None:
            raise gcmd.error(
                "No RFID reader configured for OAMS=%d RFID_CARD=%d"
                % (oams_index, rfid_card))
        reader.read_now()
        gcmd.respond_info(reader.format_last_read())


class OpenAmsMfrc522:
    """State and MFRC522 operations for one endpoint of a dual-reader driver."""

    def __init__(self, owner, name, rfid_card, spi, reset_pin):
        self.owner = owner
        self.printer = owner.printer
        self.reactor = owner.reactor
        self.name = name
        self.oams_index = owner.oams_index
        self.rfid_card = rfid_card
        self.spi = spi
        self.reset_pin = reset_pin
        self.reader = Mfrc522(self.spi, self.reactor)
        self.poll_interval = owner.poll_interval
        self.read_block_number = owner.read_block_number
        self.key = owner.key
        self.key_b = owner.key_b
        self.version = None
        self.uid = None
        self.block_data = None
        self.tag_data = None
        self.tag_processor = None
        self.last_error = None
        self.last_read_status = "NOT_READ"
        self.last_read_time = None
        self.misses = 0
        self._firmware_debug_reported = False

        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "MFRC522_QUERY", "READER", self.name, self.cmd_QUERY,
            desc="Read an OpenAMS MFRC522 reader")

    @staticmethod
    def _parse_key(value):
        value = re.sub(r"[^0-9a-fA-F]", "", value)
        if len(value) != 12:
            raise Mfrc522Error("key must contain exactly six hexadecimal bytes")
        return bytes.fromhex(value)

    @staticmethod
    def _format_firmware_debug(params):
        def pin_state(port, pin, moder, levels):
            mode = (moder >> (pin * 2)) & 3
            idr = (levels >> pin) & 1
            odr = (levels >> (16 + pin)) & 1
            return "%s%d(mode=%d in=%d out=%d)" % (port, pin, mode, idr, odr)

        def spi_state(index):
            config = params.get("spi%d_config" % index, 0)
            last = params.get("spi%d_last" % index, 0)
            if not config:
                return "spi%d(unused)" % index
            encoded = (config >> 8) & 0xFF
            cs = ("NONE" if encoded == 0xFF else "P%s%d" % (
                chr(ord("A") + (encoded >> 4)), encoded & 0x0F))
            return (
                "spi%d(oid=%d cs=%s configured=%d active_high=%d mode=%d "
                "period_ticks=%d tx0=0x%02x rx0=0x%02x rx_last=0x%02x "
                "selected_cs=%d idle_cs=%d other_cs_idle=%d reader=%d count_mod16=%d)"
                % (index, config & 0xFF, cs, (config >> 16) & 1,
                   (config >> 18) & 1, (config >> 19) & 3, config >> 21,
                   last & 0xFF, (last >> 8) & 0xFF, (last >> 16) & 0xFF,
                   (last >> 24) & 1, (last >> 25) & 1,
                   (last >> 26) & 1, (last >> 31) & 1,
                   (last >> 27) & 0x0F))

        b_moder = params.get("gpiob_moder", 0)
        b_levels = params.get("gpiob_levels", 0)
        d_moder = params.get("gpiod_moder", 0)
        d_levels = params.get("gpiod_levels", 0)
        return " ".join((
            pin_state("PB", 0, b_moder, b_levels),
            pin_state("PB", 2, b_moder, b_levels),
            pin_state("PB", 3, b_moder, b_levels),
            pin_state("PD", 2, d_moder, d_levels),
            spi_state(0), spi_state(1)))

    def _log_firmware_debug(self):
        if self._firmware_debug_reported:
            return
        self._firmware_debug_reported = True
        try:
            command = self.spi.get_mcu().lookup_query_command(
                "oams_rfid_debug", _RFID_DEBUG_RESPONSE,
                cq=self.spi.get_command_queue())
            params = command.send([])
            logging.error("MFRC522 %s firmware RFID debug: %s",
                          self.name, self._format_firmware_debug(params))
        except Exception as exc:
            logging.info("MFRC522 %s firmware RFID diagnostics unavailable: %s",
                         self.name, exc)

    def _initialize(self, eventtime):
        self.tag_processor = self.owner.tag_processor
        try:
            self.version = self.reader.initialize()
            self.last_error = None
        except Exception as exc:
            self.last_read_status = "ERROR"
            self.last_read_time = eventtime
            self.last_error = str(exc)
            logging.exception("MFRC522 %s initialization failed", self.name)
            self._log_firmware_debug()

    def _read_card(self):
        uid = self.reader.read_uid()
        data = None
        if self.read_block_number is not None:
            data = self.reader.read_block(
                self.read_block_number, self.key, uid, self.key_b)
        tag_data = self.tag_data if uid == self.uid else None
        if self.tag_processor is not None and tag_data is None:
            tag_data = self.tag_processor.read_tag(self.reader, uid)
        return uid, data, tag_data

    def _remove_cached_card(self):
        if self.uid is None:
            return
        old_uid = self.uid
        self.uid = self.block_data = self.tag_data = None
        self.printer.send_event("mfrc522:removed", self, old_uid)

    def _sample(self, eventtime, debounce_removal):
        self.last_read_time = eventtime
        try:
            if self.version is None:
                self.version = self.reader.initialize()
            uid, data, tag_data = self._read_card()
            changed = (uid != self.uid or data != self.block_data
                       or tag_data != self.tag_data)
            self.uid, self.block_data, self.tag_data = uid, data, tag_data
            self.last_read_status = "PRESENT"
            self.last_error = None
            self.misses = 0
            if changed:
                logging.info("MFRC522 %s card %s", self.name, uid.hex())
                self.printer.send_event("mfrc522:card", self, uid, data)
        except Mfrc522NoCard:
            self.last_read_status = "EMPTY"
            self.last_error = None
            self.misses += 1
            if not debounce_removal or self.misses >= 2:
                self._remove_cached_card()
        except Mfrc522Error as exc:
            self.last_read_status = "ERROR"
            self.last_error = str(exc)
            self.misses += 1
            if not debounce_removal or self.misses >= 2:
                self._remove_cached_card()
        except Exception as exc:
            self.last_read_status = "ERROR"
            self.last_error = str(exc)
            logging.exception("MFRC522 %s polling failed", self.name)

    def _poll(self, eventtime):
        self._sample(eventtime, debounce_removal=True)
        return eventtime + self.poll_interval

    def read_now(self):
        self._sample(self.reactor.monotonic(), debounce_removal=False)

    def get_status(self, eventtime):
        return {
            "oams": self.oams_index,
            "rfid_card": self.rfid_card,
            "present": self.uid is not None,
            "uid": None if self.uid is None else self.uid.hex(),
            "block": None if self.block_data is None else self.block_data.hex(),
            "filament": self.tag_data,
            "version": self.version,
            "last_read_status": self.last_read_status,
            "last_read_time": self.last_read_time,
            "error": self.last_error,
        }

    def format_last_read(self):
        current_status = "PRESENT" if self.uid is not None else "EMPTY"
        uid = "NONE" if self.uid is None else self.uid.hex().upper()
        block = ("NONE" if self.block_data is None
                 else self.block_data.hex().upper())
        version = ("UNKNOWN" if self.version is None
                   else "0x%02X" % self.version)
        filament = ("NONE" if self.tag_data is None else
                    json.dumps(self.tag_data, sort_keys=True, separators=(",", ":")))
        error = "NONE" if self.last_error is None else self.last_error
        return (
            "OAMSM_RFID_READ OAMS=%d RFID_CARD=%d STATUS=%s "
            "LAST_READ_STATUS=%s UID=%s BLOCK=%s VERSION=%s FILAMENT=%s ERROR=%s"
            % (self.oams_index, self.rfid_card, current_status,
               self.last_read_status, uid, block, version, filament, error))

    def cmd_QUERY(self, gcmd):
        self.read_now()
        gcmd.respond_info(self.format_last_read())


class OpenAmsMfrc522Pair:
    """One Klipper object that owns and serializes both OpenAMS readers."""

    RESET_LOW_TIME = 0.010
    RESET_SETTLE_TIME = 0.002

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.oams_index = config.getint("oams", minval=0)
        self.poll_interval = config.getfloat(
            "poll_interval", 0.5, minval=0.1)
        self.read_block_number = config.getint(
            "read_block", None, minval=0, maxval=255)
        self.key = OpenAmsMfrc522._parse_key(
            config.get("key", "FFFFFFFFFFFF"))
        self.key_b = config.getboolean("key_b", False)
        self.tag_processor = None

        spi = bus.MCU_SPI_from_config(
            config, mode=0, pin_option="cs_pin", default_speed=750000)
        self.shared_bus = OpenAmsRfidSpiBus(spi)
        self.readers = (
            OpenAmsMfrc522(
                self, "rfid_a", 0,
                OpenAmsRfidSpiEndpoint(self.shared_bus, 0), None),
            OpenAmsMfrc522(
                self, "rfid_b", 1,
                OpenAmsRfidSpiEndpoint(self.shared_bus, 1), None),
        )

        registry = self.printer.lookup_object("oamsm_rfid_registry", None)
        if registry is None:
            registry = OpenAmsRfidRegistry(self.printer)
            self.printer.add_object("oamsm_rfid_registry", registry)
        registry.add_driver(self, config)

    def _initialize(self, eventtime):
        self.tag_processor = self.printer.lookup_object(
            "bambu_lab_tag_processor", None)
        try:
            self.shared_bus.set_reset(0, False)
            self.shared_bus.set_reset(1, False)
            self.reactor.pause(
                self.reactor.monotonic() + self.RESET_LOW_TIME)
            self.shared_bus.set_reset(0, True)
            self.shared_bus.set_reset(1, True)
            self.reactor.pause(
                self.reactor.monotonic() + self.RESET_SETTLE_TIME)
        except Exception as exc:
            logging.exception("OpenAMS RFID hard reset failed")
            for reader in self.readers:
                reader.last_read_status = "ERROR"
                reader.last_read_time = eventtime
                reader.last_error = str(exc)
            return
        for reader in self.readers:
            reader._initialize(eventtime)

    def _poll(self, eventtime):
        for reader in self.readers:
            reader._sample(eventtime, debounce_removal=True)
        return eventtime + self.poll_interval

    def get_status(self, eventtime):
        return {
            "oams": self.oams_index,
            "readers": [reader.get_status(eventtime) for reader in self.readers],
        }


def load_config(config):
    return OpenAmsMfrc522Pair(config)


def load_config_prefix(config):
    return OpenAmsMfrc522Pair(config)
