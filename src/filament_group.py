# Filament Group
#
# Copyright (C) 2023-2026 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# A filament_group is a thin config holder: it parses its own bay list and does
# NOT resolve the referenced OAMS objects. Cross-section validation (that each
# OAMS exists, bays are unique and share one FPS lane, etc.) is owned centrally
# by [oams_manager] via oams_topology, so a group never depends on whether its
# OAMS sections happen to load before or after it.


class FilamentGroup:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.group_name = config.get_name().split()[-1]
        # [(oams_short_name, bay_index)] in declared order. The manager resolves
        # and validates these against the real OAMS units.
        self.bay_specs = self._parse(config)

    def _parse(self, config):
        # "group" is a comma-separated list of "<oams_name>-<bay_index>" entries.
        # Optional/empty is allowed so a group can be declared and then populated
        # at runtime (OAMSM_ASSIGN_BAY).
        specs = []
        for raw in config.get("group", "").split(","):
            entry = raw.strip().strip('"').strip()
            if not entry:
                continue
            # rsplit on the LAST '-' so OAMS names may themselves contain '-'.
            oams_name, sep, bay_text = entry.rpartition("-")
            if not sep or not oams_name:
                raise config.error(
                    "Invalid filament_group bay '%s' in [%s]; expected"
                    " '<oams_name>-<bay_index>' (e.g. oams1-0)"
                    % (entry, config.get_name()))
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
            specs.append((oams_name.strip(), bay_index))
        return specs


def load_config_prefix(config):
    return FilamentGroup(config)


def load_config(config):
    return FilamentGroup(config)
