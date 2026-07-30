#!/usr/bin/env python3
import pathlib
import sys
import types


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.modules.setdefault("bus", types.SimpleNamespace())
sys.path.insert(0, str(ROOT / "src"))
import mfrc522


class FakeSpi:
    def __init__(self):
        self.sent = []
        self.response = b"\x00\x92"

    def spi_transfer(self, data):
        self.sent.append(("transfer", list(data)))
        return {"response": self.response}

    def spi_send(self, data):
        self.sent.append(("send", list(data)))


def test_register_framing_and_crc():
    spi = FakeSpi()
    reader = mfrc522.Mfrc522(spi)

    assert reader.reg_read(reader.VERSION) == 0x92
    assert spi.sent[-1] == ("transfer", [0xEE, 0x00])

    reader.reg_write(reader.TX_CONTROL, 0x03)
    assert spi.sent[-1] == ("send", [0x28, 0x03])

    # ISO14443-A CRC for a MIFARE READ of block 4, little-endian on the wire.
    assert reader._calculate_crc([0x30, 0x04]) == [0x26, 0xEE]
    assert reader.reader_power(True) is None


class FakeGcmd:
    def __init__(self, params):
        self.params = params
        self.responses = []

    def get_int(self, name, default, **kwargs):
        return self.params.get(name, default)

    def error(self, message):
        return RuntimeError(message)

    def respond_info(self, message):
        self.responses.append(message)


def test_oamsm_rfid_read_command():
    calls = []
    reader = types.SimpleNamespace(
        oams_index=2, rfid_card=1,
        read_now=lambda: calls.append("read"),
        format_last_read=lambda: (
            "OAMSM_RFID_READ OAMS=2 RFID_CARD=1 STATUS=PRESENT "
            "LAST_READ_STATUS=PRESENT UID=01020304 BLOCK=NONE "
            "VERSION=0x92 ERROR=NONE"))
    registry = mfrc522.OpenAmsRfidRegistry.__new__(
        mfrc522.OpenAmsRfidRegistry)
    registry.readers = {(2, 1): reader}
    gcmd = FakeGcmd({"OAMS": 2, "RFID_CARD": 1})

    registry.cmd_RFID_READ(gcmd)

    assert calls == ["read"]
    assert "LAST_READ_STATUS=PRESENT" in gcmd.responses[0]
    assert "UID=01020304" in gcmd.responses[0]


def test_ready_serializes_dual_reader_drivers():
    callbacks = []
    timers = []
    operations = []
    reactor = types.SimpleNamespace(
        register_callback=lambda cb: callbacks.append(cb),
        register_timer=lambda cb, when: timers.append((cb, when)),
        monotonic=lambda: 20.0)
    poll_a = lambda eventtime: eventtime
    poll_b = lambda eventtime: eventtime

    def fake_driver(name, poll):
        return types.SimpleNamespace(
            _initialize=lambda eventtime: operations.append(
                (name, "initialize", eventtime)),
            _poll=poll, poll_interval=0.5)

    driver_a = fake_driver("a", poll_a)
    driver_b = fake_driver("b", poll_b)
    registry = mfrc522.OpenAmsRfidRegistry.__new__(
        mfrc522.OpenAmsRfidRegistry)
    registry.reactor = reactor
    registry.drivers = {1: driver_a, 2: driver_b}

    registry._handle_ready()
    assert operations == []
    assert len(callbacks) == 1

    callbacks[0](10.0)
    assert operations == [
        ("a", "initialize", 10.0),
        ("b", "initialize", 10.0)]
    assert timers == [(poll_a, 20.5), (poll_b, 20.5)]


def test_soft_reset_timeout_reports_command_register():
    reader = mfrc522.Mfrc522(FakeSpi())
    reader.reg_write = lambda reg, value: None
    reader.reg_read = lambda reg: 0x10

    try:
        reader.initialize()
    except mfrc522.Mfrc522Error as exc:
        assert str(exc) == "soft reset timed out (CommandReg=0x10)"
    else:
        raise AssertionError("soft reset timeout was not reported")


def test_fm17580_production_versions_are_accepted():
    for production_version in (0xA1, 0xEE):
        reader = mfrc522.Mfrc522(FakeSpi())
        writes = []
        reader.reg_write = lambda reg, value: writes.append((reg, value))
        reader.reg_read = lambda reg, version=production_version: (
            version if reg == reader.VERSION else 0)

        assert reader.initialize() == production_version
        assert (reader.COMMAND, reader.CMD_SOFT_RESET) in writes


def test_one_driver_owns_and_serializes_both_readers():
    class PairSpi:
        def __init__(self, mcu, queue):
            self.mcu = mcu
            self.queue = queue

        def get_mcu(self):
            return self.mcu

        def get_command_queue(self):
            return self.queue

    class ResetPin:
        def __init__(self):
            self.start_values = []

        def setup_start_value(self, value, shutdown_value):
            self.start_values.append((value, shutdown_value))

    common_mcu = object()
    spis = [PairSpi(common_mcu, "queue-a"),
            PairSpi(common_mcu, "queue-b")]
    spi_options = []
    original_spi_factory = getattr(
        mfrc522.bus, "MCU_SPI_from_config", None)

    def make_spi(config, **kwargs):
        spi_options.append(kwargs["pin_option"])
        return spis[len(spi_options) - 1]

    mux_readers = []
    gcode = types.SimpleNamespace(
        register_command=lambda *args, **kwargs: None,
        register_mux_command=lambda command, key, value, callback, **kwargs:
            mux_readers.append(value))
    reactor = types.SimpleNamespace(monotonic=lambda: 100.0)
    reset_pins = {}
    pins = types.SimpleNamespace(
        setup_pin=lambda pin_type, name: reset_pins.setdefault(
            name, ResetPin()))
    objects = {"gcode": gcode, "pins": pins}
    printer = types.SimpleNamespace(
        get_reactor=lambda: reactor,
        lookup_object=lambda name, default=None: objects.get(name, default),
        add_object=lambda name, value: objects.__setitem__(name, value),
        register_event_handler=lambda *args: None)
    values = {
        "oams": 1,
        "cs_pin_a": "oams_mcu1:PB3",
        "reset_pin_a": "oams_mcu1:PD2",
        "cs_pin_b": "oams_mcu1:PB2",
        "reset_pin_b": "oams_mcu1:PB0",
        "spi_speed": 100000,
    }
    config = types.SimpleNamespace(
        get_printer=lambda: printer,
        get_name=lambda: "mfrc522 openams",
        getint=lambda name, default=None, **kwargs: values.get(name, default),
        getfloat=lambda name, default=None, **kwargs: values.get(name, default),
        getboolean=lambda name, default=None, **kwargs: values.get(name, default),
        get=lambda name, default=None: values.get(name, default),
        error=lambda message: RuntimeError(message))

    try:
        mfrc522.bus.MCU_SPI_from_config = make_spi
        driver = mfrc522.OpenAmsMfrc522Pair(config)
    finally:
        if original_spi_factory is None:
            del mfrc522.bus.MCU_SPI_from_config
        else:
            mfrc522.bus.MCU_SPI_from_config = original_spi_factory

    assert spi_options == ["cs_pin_a", "cs_pin_b"]
    assert [reader.rfid_card for reader in driver.readers] == [0, 1]
    assert mux_readers == ["rfid_a", "rfid_b"]
    assert [reader.spi for reader in driver.readers] == spis
    assert reset_pins["oams_mcu1:PD2"].start_values == [(1, 1)]
    assert reset_pins["oams_mcu1:PB0"].start_values == [(1, 1)]
    registry = objects["oamsm_rfid_registry"]
    assert registry.drivers == {1: driver}
    assert sorted(registry.readers) == [(1, 0), (1, 1)]

    field_operations = []
    for reader in driver.readers:
        card = reader.rfid_card
        reader.reader.initialize = lambda card=card: (
            field_operations.append(("init", card)) or (0x91 + card))
        reader.reader.antenna_on = lambda card=card: (
            field_operations.append(("on", card)))
        reader.reader.antenna_off = lambda card=card: (
            field_operations.append(("off", card)))
    driver._initialize(9.0)
    assert field_operations == [("init", 0), ("off", 0), ("init", 1)]
    assert driver.active_reader == 1
    assert [reader.version for reader in driver.readers] == [0x91, 0x92]

    # Switching readers disables the old RF field before enabling the new one.
    field_operations[:] = []
    assert driver.activate_reader(driver.readers[0]) == 0x91
    assert field_operations == [("off", 1), ("on", 0)]
    assert driver.active_reader == 0

    # Reusing the active reader does not touch CS, reset, or its RF field.
    assert driver.activate_reader(driver.readers[0]) == 0x91
    assert field_operations == [("off", 1), ("on", 0)]

    operations = []
    for reader in driver.readers:
        card = reader.rfid_card
        reader._sample = lambda eventtime, debounce_removal, card=card: (
            operations.append((card, eventtime, debounce_removal)))
    assert driver._poll(10.0) == 10.5
    assert operations == [(0, 10.0, True), (1, 10.0, True)]


def test_firmware_debug_decodes_shared_bus_state():
    # PB0 (B NPD), PB2 (B CS), and PB3 (A CS) are outputs and high.
    b_moder = (1 << 0) | (1 << 4) | (1 << 6)
    b_levels = (0x000D << 16) | 0x000D
    d_moder = 1 << 4
    d_levels = (0x0004 << 16) | 0x0004

    def spi_config(oid, encoded_cs):
        return (oid | (encoded_cs << 8) | (1 << 16) | (1 << 17)
                | (480 << 21))

    def spi_last(tx, rx, bus_index):
        return (tx | (0xFF << 8) | (rx << 16) | (1 << 25)
                | (1 << 26) | (bus_index << 31))

    params = {
        "gpiob_moder": b_moder, "gpiob_levels": b_levels,
        "gpiod_moder": d_moder, "gpiod_levels": d_levels,
        "spi0_config": spi_config(3, 0x13),
        "spi0_last": spi_last(0xEE, 0xA1, 0),
        "spi1_config": spi_config(5, 0x12),
        "spi1_last": spi_last(0x82, 0xFF, 0),
    }
    report = mfrc522.OpenAmsMfrc522._format_firmware_debug(params)
    assert "PB0(mode=1 in=1 out=1)" in report
    assert "PB2(mode=1 in=1 out=1)" in report
    assert "PB3(mode=1 in=1 out=1)" in report
    assert "PD2(mode=1 in=1 out=1)" in report
    assert "spi0(oid=3 cs=PB3 configured=1 active_high=0" in report
    assert "spi1(oid=5 cs=PB2 configured=1 active_high=0" in report
    assert "period_ticks=480" in report
    assert "tx0=0x82 rx0=0xff rx_last=0xff" in report
    assert "selected_cs=0 idle_cs=1 other_cs_idle=1 bus=0" in report

    sends = []
    command = types.SimpleNamespace(
        send=lambda args: sends.append(args) or params)
    mcu = types.SimpleNamespace(lookup_query_command=lambda *args, **kwargs: command)
    obj = mfrc522.OpenAmsMfrc522.__new__(mfrc522.OpenAmsMfrc522)
    obj.name = "rfid_b"
    obj._firmware_debug_reported = False
    obj.spi = types.SimpleNamespace(
        get_mcu=lambda: mcu, get_command_queue=lambda: "rfid_queue")
    obj._log_firmware_debug()
    obj._log_firmware_debug()
    assert sends == [[]]


if __name__ == "__main__":
    test_register_framing_and_crc()
    test_oamsm_rfid_read_command()
    test_ready_serializes_dual_reader_drivers()
    test_soft_reset_timeout_reports_command_register()
    test_fm17580_production_versions_are_accepted()
    test_one_driver_owns_and_serializes_both_readers()
    test_firmware_debug_decodes_shared_bus_state()
    print("PASS: MFRC522/FM17580 initialization and RFID command")
