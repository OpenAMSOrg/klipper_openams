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


def test_ready_serializes_shared_bus_initialization():
    callbacks = []
    timers = []
    operations = []
    reactor = types.SimpleNamespace(
        register_callback=lambda cb: callbacks.append(cb),
        register_timer=lambda cb, when: timers.append((cb, when)),
        monotonic=lambda: 20.0)
    poll_a = lambda eventtime: eventtime
    poll_b = lambda eventtime: eventtime

    def fake_reader(name, poll):
        return types.SimpleNamespace(
            _initialize=lambda eventtime: operations.append(
                (name, "initialize", eventtime)),
            _poll=poll, poll_interval=0.5)

    reader_a = fake_reader("a", poll_a)
    reader_b = fake_reader("b", poll_b)
    registry = mfrc522.OpenAmsRfidRegistry.__new__(
        mfrc522.OpenAmsRfidRegistry)
    registry.reactor = reactor
    registry.readers = {(1, 0): reader_a, (1, 1): reader_b}

    registry._handle_ready()
    assert operations == []
    assert len(callbacks) == 1

    callbacks[0](10.0)
    assert operations == [
        ("a", "initialize", 10.0),
        ("b", "initialize", 10.0)]
    assert timers == [(poll_a, 20.5), (poll_b, 20.5)]


def test_shared_bus_switch_holds_inactive_reader_in_reset():
    operations = []
    pauses = []
    reset_states = {"a": False, "b": False}
    reactor = types.SimpleNamespace(
        monotonic=lambda: 10.0,
        pause=lambda deadline: pauses.append(deadline))

    def fake_reader(name):
        obj = types.SimpleNamespace(
            oams_index=1, version=None, reset_pin=object())

        def set_reset(enabled):
            reset_states[name] = bool(enabled)
            operations.append((name, "reset", bool(enabled)))
            assert sum(reset_states.values()) <= 1

        def initialize():
            operations.append((name, "initialize"))
            assert reset_states[name]
            assert sum(reset_states.values()) == 1
            return 0xA1 if name == "a" else 0xEE

        obj._set_reset = set_reset
        obj.reader = types.SimpleNamespace(initialize=initialize)
        return obj

    reader_a = fake_reader("a")
    reader_b = fake_reader("b")
    registry = mfrc522.OpenAmsRfidRegistry.__new__(
        mfrc522.OpenAmsRfidRegistry)
    registry.reactor = reactor
    registry.readers = {(1, 0): reader_a, (1, 1): reader_b}
    registry.active_readers = {}

    assert registry.activate_reader(reader_a) == 0xA1
    first_activation = list(operations)
    assert first_activation == [
        ("a", "reset", False), ("b", "reset", False),
        ("a", "reset", True), ("a", "initialize")]
    assert pauses == [10.010, 10.002, 10.005]

    # Reusing the current reader must not reset it between register accesses.
    assert registry.activate_reader(reader_a) == 0xA1
    assert operations == first_activation

    assert registry.activate_reader(reader_b) == 0xEE
    assert operations[-4:] == [
        ("a", "reset", False), ("b", "reset", False),
        ("b", "reset", True), ("b", "initialize")]
    assert reset_states == {"a": False, "b": True}

    # Switching back must hard-reset and reinitialize A because NPD erased its
    # register configuration while B owned the bus.
    assert registry.activate_reader(reader_a) == 0xA1
    assert operations[-4:] == [
        ("a", "reset", False), ("b", "reset", False),
        ("a", "reset", True), ("a", "initialize")]
    assert reset_states == {"a": True, "b": False}


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


def test_hardware_reset_precedes_soft_reset():
    pin_values = []
    pauses = []
    times = iter((1.0, 1.0, 1.010, 1.010))
    mcu = types.SimpleNamespace(estimated_print_time=lambda when: when + 100.0)
    reset_pin = types.SimpleNamespace(
        get_mcu=lambda: mcu,
        set_digital=lambda when, value: pin_values.append((when, value)))
    reader = mfrc522.OpenAmsMfrc522.__new__(mfrc522.OpenAmsMfrc522)
    reader.reset_pin = reset_pin
    reader.reactor = types.SimpleNamespace(
        monotonic=lambda: next(times),
        pause=lambda deadline: pauses.append(deadline))
    initialized = []
    reader.reader = types.SimpleNamespace(
        initialize=lambda: initialized.append(True) or 0xA1)

    assert reader._reset_and_initialize() == 0xA1
    assert pin_values == [(101.0, 0), (101.01, 1)]
    assert pauses == [1.01, 1.012]
    assert initialized == [True]


def test_firmware_debug_decodes_shared_bus_state():
    # PB0 (B NPD), PB2 (B CS), and PB3 (A CS) are outputs and high.
    b_moder = (1 << 0) | (1 << 4) | (1 << 6)
    b_levels = (0x000D << 16) | 0x000D
    d_moder = 1 << 4
    d_levels = (0x0004 << 16) | 0x0004

    def spi_config(oid, encoded_cs):
        return oid | (encoded_cs << 8) | (1 << 16) | (1 << 17) | (1 << 21)

    def spi_last(tx, rx):
        return tx | (0xFF << 8) | (rx << 16) | (1 << 25) | (1 << 26)

    params = {
        "gpiob_moder": b_moder, "gpiob_levels": b_levels,
        "gpiod_moder": d_moder, "gpiod_levels": d_levels,
        "spi0_config": spi_config(3, 0x13),
        "spi0_last": spi_last(0xEE, 0xA1),
        "spi1_config": spi_config(5, 0x12),
        "spi1_last": spi_last(0x82, 0xFF),
    }
    report = mfrc522.OpenAmsMfrc522._format_firmware_debug(params)
    assert "PB0(mode=1 in=1 out=1)" in report
    assert "PB2(mode=1 in=1 out=1)" in report
    assert "PB3(mode=1 in=1 out=1)" in report
    assert "PD2(mode=1 in=1 out=1)" in report
    assert "spi0(oid=3 cs=PB3 configured=1 active_high=0" in report
    assert "spi1(oid=5 cs=PB2 configured=1 active_high=0" in report
    assert "tx0=0x82 rx0=0xff rx_last=0xff" in report
    assert "selected_cs=0 idle_cs=1 other_cs_idle=1" in report

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
    test_ready_serializes_shared_bus_initialization()
    test_shared_bus_switch_holds_inactive_reader_in_reset()
    test_soft_reset_timeout_reports_command_register()
    test_fm17580_production_versions_are_accepted()
    test_hardware_reset_precedes_soft_reset()
    test_firmware_debug_decodes_shared_bus_state()
    print("PASS: MFRC522/FM17580 initialization and RFID command")
