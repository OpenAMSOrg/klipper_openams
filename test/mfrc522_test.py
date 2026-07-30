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


if __name__ == "__main__":
    test_register_framing_and_crc()
    test_oamsm_rfid_read_command()
    print("PASS: MFRC522 SPI framing, CRC-A, and OAMSM RFID command")
