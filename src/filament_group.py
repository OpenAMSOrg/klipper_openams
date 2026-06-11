# Filament Group
#
# Copyright (C) 2023-2026 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class FilamentGroup:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        self.group_name = config.get_name().split()[-1]
        self.bays = []
        self.oams = []
        self._initialize_bays(config)

    def _initialize_bays(self, config):
        # "group" is a comma-separated list of "<oams_name>-<bay_index>" entries.
        for raw in config.get("group").split(","):
            entry = raw.strip().strip('"').strip()
            if not entry:
                continue
            if entry.count("-") != 1:
                raise config.error(
                    "Invalid filament_group bay '%s' in [%s]; expected"
                    " '<oams_name>-<bay_index>' (e.g. oams1-0)"
                    % (entry, config.get_name()))
            oams_name, bay_text = entry.split("-")
            try:
                bay_index = int(bay_text)
            except ValueError:
                raise config.error(
                    "Invalid bay index '%s' in filament_group [%s]; must be an"
                    " integer 0-3" % (bay_text, config.get_name()))
            if not 0 <= bay_index <= 3:
                raise config.error(
                    "Bay index %d out of range in filament_group [%s]; must be"
                    " 0-3" % (bay_index, config.get_name()))
            oam = self.printer.lookup_object("oams " + oams_name.strip())
            self.add_bay(oam, bay_index)

    def add_bay(self, oam, bay_index):
        self.bays.append((oam, bay_index))
        if oam not in self.oams:
            self.oams.append(oam)


def load_config_prefix(config):
    return FilamentGroup(config)


def load_config(config):
    return FilamentGroup(config)
