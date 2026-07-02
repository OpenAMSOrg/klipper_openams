# OpenAMS topology — pure configuration model + validation
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Like oams_state.py this module is INTENTIONALLY PURE: it imports nothing from
# Klipper and performs no I/O. It owns the relationships between FPS lanes, OAMS
# units and filament groups, validates them, and is the authoritative (immutable)
# model the manager uses both at config time and for runtime group editing
# (create group / reassign bay). Keeping it pure makes the rules exhaustively
# unit-testable and lets the runtime-edit API reuse the exact same checks before
# persisting a change back to the config files.
#
# Invariants enforced:
#   F1  at least one FPS lane; lane names unique
#   O1  OAMS names unique; oams_idx unique
#   O2  every OAMS resolves to a known FPS lane (explicit 'fps:' or, when there
#       is exactly one lane, that sole lane)
#   G1  every group bay references a defined OAMS and a bay in 0..3
#   G2  a group's bays all live on one FPS lane
#   G3  a bay belongs to at most one group (so a bay can be *re*assigned, never
#       silently shared)

from dataclasses import dataclass, replace
import re
from typing import Mapping, Optional, Tuple


class TopologyError(Exception):
    """Invalid OpenAMS configuration. The manager re-raises this as a Klipper
    config.error, so messages are written to be user-facing and actionable."""


# Names created at RUNTIME must survive the round trip through a config file:
# a single token (multi-word section names collapse to their last token at
# load), and none of the characters that break a section header or get eaten
# by Klipper's comment stripping (']', '[', '#', ';', whitespace, newlines).
_GROUP_NAME_RE = re.compile(r'^[A-Za-z0-9_.\-]+$')


def _check_group_name(name):
    if not name or not _GROUP_NAME_RE.match(name):
        raise TopologyError(
            "Invalid filament_group name %r: use only letters, digits,"
            " '_', '-' and '.'" % (name,))


@dataclass(frozen=True)
class OamsSpec:
    name: str                   # short section name, e.g. "oams1"
    idx: int                    # oams_idx
    fps: Optional[str] = None   # explicit FPS lane, or None to default to the sole lane


@dataclass(frozen=True)
class Topology:
    fps_names: Tuple[str, ...]
    # oams short name -> (oams_idx, resolved fps lane)
    oams: Mapping[str, Tuple[int, str]]
    # group name -> tuple of (oams short name, bay index), in declared order
    groups: Mapping[str, Tuple[Tuple[str, int], ...]]

    # ------------------------------------------------------------- accessors
    def idx_of(self, oams_name):
        return self.oams[oams_name][0]

    def lane_of_oams(self, oams_name):
        return self.oams[oams_name][1]

    def oams_on_lane(self, lane):
        return tuple(n for n, (_idx, l) in self.oams.items() if l == lane)

    def lane_of_group(self, group):
        bays = self.groups[group]
        return self.lane_of_oams(bays[0][0]) if bays else None

    def group_bays_idx(self, group):
        """Group bays as (oams_idx, bay) — the form the reducer/world consume."""
        return tuple((self.idx_of(n), b) for (n, b) in self.groups[group])

    def group_config_value(self, group):
        """The `group:` option string for config writeback, e.g. 'oams1-0,oams1-3'."""
        return ",".join("%s-%d" % (n, b) for (n, b) in self.groups[group])


# ===================================================================== build

def build_topology(fps_names, oams_specs, group_specs):
    """Validate the configuration and return an immutable Topology.

    fps_names:   ordered list of FPS lane short names.
    oams_specs:  list of OamsSpec.
    group_specs: list of (group_name, [(oams_name, bay_index), ...]) in order.

    Raises TopologyError (with a user-facing message) on any invalid relation.
    """
    if not fps_names:
        raise TopologyError(
            "No [fps] section found; OpenAMS requires at least one FPS lane.")
    seen = set()
    for f in fps_names:
        if f in seen:
            raise TopologyError("Duplicate FPS lane '%s'." % f)
        seen.add(f)
    sole = fps_names[0] if len(fps_names) == 1 else None

    oams = {}
    idx_owner = {}
    for spec in oams_specs:
        if spec.name in oams:
            raise TopologyError("Duplicate OAMS '%s'." % spec.name)
        lane = spec.fps or sole
        if lane is None:
            raise TopologyError(
                "[oams %s] must set 'fps:' when several [fps] lanes are defined."
                % spec.name)
        if lane not in seen:
            raise TopologyError(
                "[oams %s] references unknown fps '%s'." % (spec.name, lane))
        if spec.idx in idx_owner:
            raise TopologyError(
                "oams_idx %d is shared by '%s' and '%s'; each OAMS needs a"
                " unique oams_idx." % (spec.idx, idx_owner[spec.idx], spec.name))
        idx_owner[spec.idx] = spec.name
        oams[spec.name] = (spec.idx, lane)

    groups = {}
    for gname, bays in group_specs:
        if gname in groups:
            raise TopologyError(
                "Duplicate filament_group '%s'; names must be unique." % gname)
        groups[gname] = tuple(bays)
    _validate_groups(oams, groups)

    return Topology(fps_names=tuple(fps_names), oams=oams, groups=groups)


def _validate_groups(oams, groups):
    """Check G1-G3 for every group against the (fixed) oams map. Shared by
    build_topology and every runtime mutation so the rules never diverge."""
    bay_owner = {}
    for gname, bays in groups.items():
        lane = None
        within = set()
        for entry in bays:
            oams_name, bay = entry
            if not 0 <= bay <= 3:
                raise TopologyError(
                    "filament_group '%s': bay index %d out of range (0-3)."
                    % (gname, bay))
            if oams_name not in oams:
                raise TopologyError(
                    "filament_group '%s' references unknown OAMS '%s'; define"
                    " [oams %s] or fix the name." % (gname, oams_name, oams_name))
            if entry in within:
                raise TopologyError(
                    "filament_group '%s' lists %s-%d twice."
                    % (gname, oams_name, bay))
            within.add(entry)
            if entry in bay_owner:
                raise TopologyError(
                    "bay %s-%d is in both '%s' and '%s'; a bay may belong to"
                    " only one filament_group." % (oams_name, bay,
                                                   bay_owner[entry], gname))
            bay_owner[entry] = gname
            oam_lane = oams[oams_name][1]
            if lane is None:
                lane = oam_lane
            elif lane != oam_lane:
                raise TopologyError(
                    "filament_group '%s' spans FPS lanes %s and %s; a group's"
                    " bays must share one FPS lane." % (gname, lane, oam_lane))


# ================================================================== mutation
# Runtime group editing for the future UI. Each returns a NEW, re-validated
# Topology; the manager swaps it in and persists the affected group(s).

def with_group(topo, name):
    _check_group_name(name)
    if name in topo.groups:
        raise TopologyError("filament_group '%s' already exists." % name)
    groups = dict(topo.groups)
    groups[name] = ()
    return replace(topo, groups=groups)


def without_group(topo, name):
    if name not in topo.groups:
        raise TopologyError("filament_group '%s' does not exist." % name)
    groups = dict(topo.groups)
    del groups[name]
    return replace(topo, groups=groups)


def with_bay(topo, group, oams_name, bay):
    """Assign (oams_name, bay) to `group`, removing it from any group that
    currently holds it (reassignment is a move, not a copy)."""
    if group not in topo.groups:
        raise TopologyError("filament_group '%s' does not exist." % group)
    entry = (oams_name, bay)
    groups = {g: tuple(e for e in bays if e != entry)
              for g, bays in topo.groups.items()}
    groups[group] = groups[group] + (entry,)
    _validate_groups(topo.oams, groups)
    return replace(topo, groups=groups)


def without_bay(topo, group, oams_name, bay):
    if group not in topo.groups:
        raise TopologyError("filament_group '%s' does not exist." % group)
    entry = (oams_name, bay)
    if entry not in topo.groups[group]:
        raise TopologyError(
            "bay %s-%d is not in '%s'." % (oams_name, bay, group))
    groups = dict(topo.groups)
    groups[group] = tuple(e for e in groups[group] if e != entry)
    return replace(topo, groups=groups)
