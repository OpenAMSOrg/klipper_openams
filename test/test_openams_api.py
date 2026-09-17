#!/usr/bin/env python3

import types

from src import oams_manager


class FakeReactor:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now

    def pause(self, wake_time):
        self.now = wake_time


class FakeUnit:
    def __init__(self, index, ready, loaded=None):
        self.oams_idx = index
        self.name = "oams unit%d" % index
        self.f1s_hes_value = list(ready)
        self.hub_hes_value = list(loaded or [False] * len(ready))
        self.current_spool = None
        self.encoder_clicks = 0
        self.action_status = None
        self.loaded_bays = []

    def is_bay_ready(self, bay):
        return self.f1s_hes_value[bay]

    def is_bay_loaded(self, bay):
        return self.hub_hes_value[bay]

    def start_load_spool(self, bay):
        self.loaded_bays.append(bay)

    def finish_load_spool(self, bay):
        self.current_spool = bay
        self.hub_hes_value[bay] = True
        return 0, "loaded"


class FakeGcmd:
    def __init__(self, group, slot=None):
        self.values = {"GROUP": group}
        if slot is not None:
            self.values["SLOT"] = slot
        self.responses = []

    def get(self, name, default=None):
        return self.values.get(name, default)

    def get_int(self, name, default=None):
        value = self.values.get(name, default)
        return None if value is None else int(value)

    def respond_info(self, message):
        self.responses.append(message)

    def error(self, message):
        return ValueError(message)


def make_manager():
    unit2 = FakeUnit(2, [True, False, True, True])
    unit1 = FakeUnit(1, [True, True, False, True], [False, True, False, False])
    manager = oams_manager.OAMSManager.__new__(oams_manager.OAMSManager)
    manager.oams = {"oams unit2": unit2, "oams unit1": unit1}
    manager.filament_groups = {
        "T1": types.SimpleNamespace(bays=[(unit2, 2), (unit1, 3)]),
        "T0": types.SimpleNamespace(bays=[(unit1, 1), (unit2, 0)]),
    }
    manager.ready = True
    manager.current_group = "T0"
    manager.current_spool = (unit1, 1)
    manager.current_state = oams_manager.OAMSState("LOADED", 10.0, manager.current_spool)
    manager.reactor = FakeReactor()
    manager._load_cancel_requested = False
    return manager, unit1, unit2


def test_status_contract_is_versioned_and_uses_stable_global_slots():
    manager, _, _ = make_manager()

    status = manager.get_status(10.0)

    assert status["api_version"] == 1
    assert status["schema"] == "openams.manager"
    assert status["commands"] == {
        "load": "OPENAMS_LOAD",
        "unload": "OPENAMS_UNLOAD",
        "cancel": "OAMSM_LOAD_FILAMENT_CANCEL",
        "reset": "OAMSM_CLEAR_ERRORS",
    }
    assert [unit["id"] for unit in status["units"]] == ["1", "2"]
    assert [unit["topology"] for unit in status["units"]] == ["hub", "hub"]
    assert [slot["id"] for slot in status["units"][0]["slots"]] == [0, 1, 2, 3]
    assert [slot["id"] for slot in status["units"][1]["slots"]] == [4, 5, 6, 7]
    assert status["lanes"] == [{
        "id": "fps",
        "state": "loaded",
        "current_group": "T0",
        "current_slot": 1,
        "following": False,
        "direction": 0,
        "message": None,
    }]
    groups = {group["name"]: group for group in status["groups"]}
    assert groups["T0"]["slots"] == [1, 4]
    assert groups["T1"]["slots"] == [6, 3]


def test_targeted_load_uses_requested_slot_from_group():
    manager, unit1, unit2 = make_manager()
    manager.current_group = None
    manager.current_spool = None
    manager.current_state = oams_manager.OAMSState("UNLOADED", 10.0, None)
    unit1.hub_hes_value = [False] * 4

    gcmd = FakeGcmd("T1", slot=6)
    manager.cmd_LOAD_FILAMENT(gcmd)

    assert unit1.loaded_bays == []
    assert unit2.loaded_bays == [2]
    assert manager.current_group == "T1"
    assert manager.current_spool == (unit2, 2)


def test_targeted_load_rejects_slot_outside_group():
    manager, unit1, unit2 = make_manager()
    manager.current_group = None
    manager.current_spool = None
    manager.current_state = oams_manager.OAMSState("UNLOADED", 10.0, None)
    unit1.hub_hes_value = [False] * 4

    gcmd = FakeGcmd("T1", slot=0)
    try:
        manager.cmd_LOAD_FILAMENT(gcmd)
        assert False, "expected invalid group membership to abort the macro"
    except ValueError as error:
        assert str(error) == "OpenAMS slot 0 is not assigned to group T1"

    assert unit1.loaded_bays == []
    assert unit2.loaded_bays == []


def test_targeted_load_rejects_unready_slot():
    manager, unit1, _ = make_manager()
    manager.current_group = None
    manager.current_spool = None
    manager.current_state = oams_manager.OAMSState("UNLOADED", 10.0, None)
    unit1.hub_hes_value = [False] * 4
    unit1.f1s_hes_value[3] = False

    try:
        manager.cmd_LOAD_FILAMENT(FakeGcmd("T1", slot=3))
        assert False, "expected an unavailable target to abort the macro"
    except ValueError as error:
        assert str(error) == "OpenAMS slot 3 is not ready"
