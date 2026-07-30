#!/usr/bin/env python3
import pathlib
import struct
import sys
import types


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import bambu_lab_tag_processor


TEST_KEY = "00112233445566778899AABBCCDDEEFF"
TEST_UID = bytes.fromhex("01020304")


def make_processor(key=TEST_KEY):
    config = types.SimpleNamespace(
        get=lambda name: key,
        error=lambda message: ValueError(message))
    return bambu_lab_tag_processor.BambuLabTagProcessor(config)


def test_master_key_validation_and_derivation():
    processor = make_processor()
    keys = processor.derive_sector_keys(TEST_UID)

    assert len(keys) == 16
    assert all(len(key) == 6 for key in keys)
    assert keys[0].hex() == "3f18a349815f"

    try:
        make_processor("not-a-16-byte-key")
    except ValueError as exc:
        assert "16 hexadecimal bytes" in str(exc)
    else:
        raise AssertionError("invalid Bambu master key was accepted")


def test_selected_block_read_and_decode():
    processor = make_processor()
    image = bytearray(1024)
    image[32:48] = b"PLA\x00" + b"\x00" * 12
    image[64:80] = b"Basic PLA\x00" + b"\x00" * 6
    image[80:84] = bytes((0x12, 0x34, 0x56, 0xFF))
    struct.pack_into("<H", image, 84, 1000)
    struct.pack_into("<f", image, 88, 1.75)
    struct.pack_into("<H", image, 96, 55)
    struct.pack_into("<H", image, 98, 8)
    struct.pack_into("<H", image, 102, 60)
    struct.pack_into("<H", image, 104, 230)
    struct.pack_into("<H", image, 106, 190)
    struct.pack_into("<f", image, 140, 0.4)
    image[144:160] = bytes(range(16))
    struct.pack_into("<H", image, 164, 6700)
    image[192:208] = b"2026-07-29\x00" + b"\x00" * 5
    struct.pack_into("<H", image, 228, 330)

    calls = []

    def read_block(block, key, uid, key_b=False):
        calls.append((block, bytes(key), bytes(uid), key_b))
        return bytes(image[block * 16:(block + 1) * 16])

    filament = processor.read_tag(
        types.SimpleNamespace(read_block=read_block), TEST_UID)
    keys = processor.derive_sector_keys(TEST_UID)

    assert [call[0] for call in calls] == list(processor.READ_BLOCKS)
    assert all(call[1] == keys[call[0] // 4] for call in calls)
    assert all(call[2] == TEST_UID and not call[3] for call in calls)
    assert filament["manufacturer"] == "Bambu"
    assert filament["type"] == "PLA"
    assert filament["detailed"] == "Basic PLA"
    assert filament["color_argb"] == 0xFF123456
    assert filament["weight_g"] == 1000
    assert filament["diameter_mm"] == 1.75
    assert filament["spool_width_mm"] == 67.0
    assert filament["length_m"] == 330


if __name__ == "__main__":
    test_master_key_validation_and_derivation()
    test_selected_block_read_and_decode()
    print("PASS: Bambu key validation, selected-block read, and decode")
