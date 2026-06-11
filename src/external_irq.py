# External IRQ
#
# Copyright (C) 2023-2026 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import re

import mcu

# Pins look like "PA10" / "PF11" (optional leading 'P'), case-insensitive.
_PIN_RE = re.compile(r'^(?:P)?([A-Za-z])(\d+)$')


class ExternalIRQ:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu = mcu.get_printer_mcu(self.printer, config.get('mcu'))

        # serialhdl keeps exactly ONE handler per (message, oid) and
        # ext_irq_trigger carries no oid, so per-instance registration would let
        # the last [external_irq] section on an MCU silently clobber the others.
        # Register one dispatcher per MCU (registry stored on the mcu object so
        # its lifetime matches) and attribute triggers to sections via the
        # (port, pin) carried in the message.
        sections = getattr(self.mcu, '_openams_ext_irq_sections', None)
        if sections is None:
            sections = {}            # (port_index, pin_number) -> section name
            self.mcu._openams_ext_irq_sections = sections
            self.mcu.register_serial_response(
                lambda params, s=sections: self._ext_irq_triggered(params, s),
                "ext_irq_trigger port=%c pin=%c")

        # Parse the configured pins into (port_index, pin_number) and allocate an
        # oid per pin.
        self._pins = []
        self._oids = []
        for raw in config.get('pins').split(','):
            pin = raw.strip()
            if not pin:
                continue
            m = _PIN_RE.match(pin)
            if not m:
                raise config.error(
                    "Invalid pin format '%s' in [%s]; expected e.g. PA10 or PF11"
                    % (pin, config.get_name()))
            # Port letter -> zero-based bank index (A=0, B=1, ...), matching the
            # firmware's GPIO(port, bit) numbering.
            port_index = ord(m.group(1).upper()) - ord('A')
            pin_number = int(m.group(2))
            if (port_index, pin_number) in sections:
                raise config.error(
                    "Pin '%s' in [%s] is already claimed by [external_irq %s]"
                    % (pin, config.get_name(), sections[(port_index, pin_number)]))
            sections[(port_index, pin_number)] = self.name
            self._pins.append((port_index, pin_number))
            self._oids.append(self.mcu.create_oid())

        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        for oid, (port_index, pin_number) in zip(self._oids, self._pins):
            self.mcu.add_config_cmd(
                "config_ext_irq oid=%d irq_port=%d irq_pin=%d"
                % (oid, port_index, pin_number))

    @staticmethod
    def _ext_irq_triggered(params, sections):
        # Serial response handlers are invoked with a single params dict.
        port, pin = params.get("port"), params.get("pin")
        name = sections.get((port, pin), "?")
        logging.info("ExternalIRQ[%s]: triggered on port=%s pin=%s",
                     name, port, pin)


def load_config_prefix(config):
    return ExternalIRQ(config)


def load_config(config):
    return ExternalIRQ(config)
