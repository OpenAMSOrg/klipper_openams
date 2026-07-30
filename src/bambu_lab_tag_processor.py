# Bambu Lab RFID tag key derivation and filament metadata decoder.
# Copyright (C) 2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import hashlib
import hmac
import re
import struct


class BambuLabTagError(Exception):
    pass


class BambuLabTagProcessor:
    # Blocks containing the public filament fields used by the decoder.
    READ_BLOCKS = (2, 4, 5, 6, 8, 9, 10, 12, 14)

    def __init__(self, config):
        value = re.sub(r"[^0-9a-fA-F]", "", config.get("key"))
        if len(value) != 32:
            raise config.error(
                "bambu_lab_tag_processor key must contain exactly "
                "16 hexadecimal bytes")
        self.master_key = bytes.fromhex(value)

    @staticmethod
    def _hkdf_sha256(salt, ikm, info, length):
        prk = hmac.new(salt, ikm, hashlib.sha256).digest()
        output = b""
        previous = b""
        counter = 0
        while len(output) < length:
            counter += 1
            previous = hmac.new(
                prk, previous + info + bytes([counter]),
                hashlib.sha256).digest()
            output += previous
        return output[:length]

    def derive_sector_keys(self, uid):
        material = self._hkdf_sha256(
            self.master_key, bytes(uid), b"RFID-A\x00", 6 * 16)
        return tuple(material[offset:offset + 6]
                     for offset in range(0, len(material), 6))

    @staticmethod
    def _string(data, offset, length):
        return data[offset:offset + length].split(b"\x00")[0].decode(
            "ascii", "replace")

    @staticmethod
    def _u16(data, offset):
        return struct.unpack_from("<H", data, offset)[0]

    @staticmethod
    def _f32(data, offset):
        return struct.unpack_from("<f", data, offset)[0]

    def decode(self, data):
        if len(data) != 1024:
            raise BambuLabTagError(
                "expected a 1024-byte MIFARE Classic image")
        red, green, blue, alpha = data[80:84]
        nozzle = round(self._f32(data, 140), 2)
        spool_width = self._u16(data, 164)
        return {
            "manufacturer": "Bambu",
            "type": self._string(data, 32, 16),
            "detailed": self._string(data, 64, 16),
            "color_argb": ((alpha << 24) | (red << 16)
                           | (green << 8) | blue),
            "weight_g": self._u16(data, 84),
            "diameter_mm": round(self._f32(data, 88), 3),
            "drying_temp_c": self._u16(data, 96),
            "drying_time_h": self._u16(data, 98),
            "bed_temp_c": self._u16(data, 102),
            "hotend_max_c": self._u16(data, 104),
            "hotend_min_c": self._u16(data, 106),
            "nozzle_diameter": nozzle if 0 < nozzle < 2 else None,
            "spool_width_mm": (round(spool_width / 100.0, 2)
                               if spool_width else None),
            "length_m": self._u16(data, 228) or None,
            "tray_uid": data[144:160].hex(),
            "production": self._string(data, 192, 16),
        }

    def read_tag(self, reader, uid):
        keys = self.derive_sector_keys(uid)
        image = bytearray(1024)
        for block in self.READ_BLOCKS:
            payload = reader.read_block(block, keys[block // 4], uid)
            if len(payload) != 16:
                raise BambuLabTagError(
                    "invalid MIFARE block %d length" % block)
            image[block * 16:(block + 1) * 16] = payload
        return self.decode(bytes(image))


def load_config(config):
    return BambuLabTagProcessor(config)
