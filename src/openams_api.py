# OpenAMS public UI contract
#
# Copyright (C) 2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""Small, hardware-family-neutral helpers for the OpenAMS UI status API.

The manager owns the policy that turns its runtime state into lanes, units,
and slots.  This module owns only the stable wire shape.  Keeping those two
jobs separate lets the current single-FPS manager and the future multi-FPS,
multi-family manager publish the same contract without sharing internals.
"""

API_VERSION = 1
SCHEMA = "openams.manager"

COMMANDS = {
    "load": "OPENAMS_LOAD",
    "unload": "OPENAMS_UNLOAD",
    "cancel": "OAMSM_LOAD_FILAMENT_CANCEL",
    "reset": "OAMSM_CLEAR_ERRORS",
}


def make_slot(slot_id, bay, ready, loaded):
    return {
        "id": int(slot_id),
        "bay": int(bay),
        "ready": bool(ready),
        "loaded": bool(loaded),
    }


def make_unit(unit_id, name, kind, lane, connected, slots, topology=None):
    if topology is None:
        topology = "linear" if kind == "follower" else "hub"
    return {
        "id": str(unit_id),
        "name": str(name),
        "kind": str(kind),
        "topology": str(topology),
        "lane": str(lane),
        "connected": bool(connected),
        "slots": list(slots),
    }


def make_lane(
    lane_id,
    state,
    current_group=None,
    current_slot=None,
    following=False,
    direction=0,
    message=None,
):
    return {
        "id": str(lane_id),
        "state": str(state),
        "current_group": current_group,
        "current_slot": current_slot,
        "following": bool(following),
        "direction": int(direction),
        "message": message,
    }


def make_group(name, lane, slots):
    return {
        "name": str(name),
        "lane": None if lane is None else str(lane),
        "slots": [int(slot) for slot in slots],
    }


def build_status(ready, lanes, units, groups):
    """Return one complete, versioned snapshot suitable for get_status()."""
    return {
        "api_version": API_VERSION,
        "schema": SCHEMA,
        "ready": bool(ready),
        "commands": dict(COMMANDS),
        "lanes": list(lanes),
        "units": list(units),
        "groups": list(groups),
    }
