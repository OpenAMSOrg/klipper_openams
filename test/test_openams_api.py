#!/usr/bin/env python3

import json
import pathlib
import subprocess
import sys
import types

import pytest

sys.modules.setdefault("mcu", types.ModuleType("mcu"))

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
        self.load_result = 0
        self.unload_result = True
        self.unload_calls = 0

    def is_bay_ready(self, bay):
        return self.f1s_hes_value[bay]

    def is_bay_loaded(self, bay):
        return self.hub_hes_value[bay]

    def start_load_spool(self, bay):
        self.loaded_bays.append(bay)

    def finish_load_spool(self, bay):
        if self.load_result:
            return self.load_result, "load failed"
        self.current_spool = bay
        self.hub_hes_value[bay] = True
        return 0, "loaded"

    def unload_spool(self):
        self.unload_calls += 1
        if not self.unload_result:
            return False, "unload failed"
        self.current_spool = None
        self.hub_hes_value = [False] * len(self.hub_hes_value)
        return True, "unloaded"


class FakeGcmd:
    def __init__(self, group=None, slot=None, **values):
        self.values = {"GROUP": group}
        self.values.update(values)
        if slot is not None:
            self.values["SLOT"] = slot
        self.responses = []

    def get(self, name, default=None):
        return self.values.get(name, default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        value = self.values.get(name, default)
        value = None if value is None else int(value)
        if value is not None:
            if minval is not None and value < minval:
                raise self.error("parameter below minimum")
            if maxval is not None and value > maxval:
                raise self.error("parameter above maximum")
        return value

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
    manager.fps = types.SimpleNamespace(get_value=lambda: 0.5)
    manager.printer = types.SimpleNamespace(lookup_object=lambda name:
        types.SimpleNamespace(get_status=lambda now: {"commands": {
            command: {} for command in (
                "OPENAMS_LOAD", "OPENAMS_UNLOAD",
                "OAMSM_LOAD_FILAMENT_CANCEL", "OAMSM_CLEAR_ERRORS")
        }}))
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
        "pressure": 0.5,
        "set_point": None,
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


def unloaded_manager():
    manager, unit1, unit2 = make_manager()
    manager.current_group = manager.current_spool = None
    manager.current_state = oams_manager.OAMSState("UNLOADED", 10.0, None)
    unit1.hub_hes_value = [False] * 4
    return manager, unit1, unit2


def test_legacy_load_keeps_group_priority_not_sorted_slot_order():
    manager, unit1, unit2 = unloaded_manager()
    manager.cmd_LOAD_FILAMENT(FakeGcmd("T1"))
    assert unit2.loaded_bays == [2]
    assert unit1.loaded_bays == []
    assert manager.current_spool == (unit2, 2)


@pytest.mark.parametrize("case", ["unknown_group", "empty", "failed", "cancelled"])
def test_legacy_load_failures_remain_informational(case):
    manager, unit1, unit2 = unloaded_manager()
    group = "T1"
    expected = "load failed"
    if case == "unknown_group":
        group = "missing"
        expected = "Group missing does not exist"
    elif case == "empty":
        unit1.f1s_hes_value = unit2.f1s_hes_value = [False] * 4
        expected = "No spool available for group T1"
    else:
        unit2.load_result = 6 if case == "cancelled" else 2
    command = FakeGcmd(group)
    manager.cmd_LOAD_FILAMENT(command)
    assert command.responses == [expected]
    assert manager.current_spool is None


def test_legacy_already_loaded_does_not_require_group():
    manager, unit1, _ = make_manager()
    unit1.current_spool = 1
    command = FakeGcmd()
    manager.cmd_LOAD_FILAMENT(command)
    assert command.responses == ["Printer is already loaded with a spool"]
    assert unit1.loaded_bays == []


@pytest.mark.parametrize("strict", [False, True])
def test_unload_failure_only_aborts_when_explicitly_strict(strict):
    manager, unit1, _ = make_manager()
    unit1.current_spool = 1
    unit1.unload_result = False
    command = FakeGcmd(STRICT=int(strict))
    if strict:
        with pytest.raises(ValueError, match="unload failed"):
            manager.cmd_UNLOAD_FILAMENT(command)
    else:
        manager.cmd_UNLOAD_FILAMENT(command)
        assert command.responses == ["unload failed"]
    assert manager.current_spool == (unit1, 1)


@pytest.mark.parametrize("strict", [False, True])
def test_unload_success_clears_existing_state(strict):
    manager, unit1, _ = make_manager()
    unit1.current_spool = 1
    manager.cmd_UNLOAD_FILAMENT(FakeGcmd(STRICT=int(strict)))
    assert manager.current_spool is None
    assert manager.current_group is None
    assert manager.current_state.name == "UNLOADED"
    assert unit1.unload_calls == 1


@pytest.mark.parametrize("code", [1, 2, 5, 6])
def test_targeted_load_failure_aborts(code):
    manager, _, unit2 = unloaded_manager()
    unit2.load_result = code
    with pytest.raises(ValueError, match="load failed"):
        manager.cmd_LOAD_FILAMENT(FakeGcmd("T1", 6))
    assert manager.current_group is None
    assert manager.current_spool is None


@pytest.mark.parametrize("group,slot", [("missing", 0), ("T1", 0), ("T1", 99), ("T1", -1)])
def test_preflight_rejects_bad_target_without_touching_loaded_spool(group, slot):
    manager, unit1, unit2 = make_manager()
    unit1.current_spool = 1
    with pytest.raises(ValueError):
        manager.cmd_VALIDATE_LOAD(FakeGcmd(group, slot))
    assert manager.current_spool == (unit1, 1)
    assert not unit1.unload_calls
    assert not unit1.loaded_bays and not unit2.loaded_bays


def test_targeted_load_does_not_succeed_if_another_spool_remains_loaded():
    manager, unit1, _ = make_manager()
    unit1.current_spool = 1
    with pytest.raises(ValueError, match="Unload the current spool"):
        manager.cmd_LOAD_FILAMENT(FakeGcmd("T1", 6))
    assert manager.current_spool == (unit1, 1)


def test_targeted_load_of_current_slot_is_idempotent():
    manager, unit1, _ = make_manager()
    unit1.current_spool = 1
    unit1.f1s_hes_value[1] = False
    manager.cmd_LOAD_FILAMENT(FakeGcmd("T0", 1))
    assert unit1.loaded_bays == []
    assert manager.current_spool == (unit1, 1)


def test_legacy_webhook_fields_are_preserved():
    manager, _, _ = make_manager()
    replies = []
    manager._webhook_status(types.SimpleNamespace(send=replies.append))
    status = replies[0]["status"]["openams"]
    api = status.pop("api")
    assert status == {
        "ready": True, "current_group": "T0", "units": 2, "fps_value": 0.5,
        "filament_groups": {
            "T1": {"bays": 2, "spools": ["oams2-2", "oams1-3"]},
            "T0": {"bays": 2, "spools": ["oams1-1", "oams2-0"]},
        },
    }
    assert api["schema"] == "openams.manager"
    json.dumps(replies)


def test_old_macro_installation_does_not_advertise_unavailable_ui_commands():
    manager, _, _ = make_manager()
    manager.printer = types.SimpleNamespace(lookup_object=lambda name:
        types.SimpleNamespace(get_status=lambda now: {"commands": {
            "OAMSM_LOAD_FILAMENT_CANCEL": {}, "OAMSM_CLEAR_ERRORS": {}
        }}))
    status = manager.get_status(10.)
    assert status["current_group"] == "T0"
    assert status["commands"] == {
        "cancel": "OAMSM_LOAD_FILAMENT_CANCEL", "reset": "OAMSM_CLEAR_ERRORS"}


def test_status_before_ready_and_without_units_is_serializable():
    manager, _, _ = make_manager()
    manager.ready = False
    manager.oams = {}
    manager.filament_groups = {}
    manager.current_group = manager.current_spool = None
    manager.current_state = oams_manager.OAMSState(None, None, None)
    status = manager.get_status(0.)
    assert status["ready"] is False
    assert status["units"] == []
    assert status["lanes"][0]["state"] == "unloaded"
    json.dumps(status)


def test_upgrade_import_works_with_only_preexisting_extra_symlinks(tmp_path):
    # Reproduce Klipper's import path, not the source-tree import used above.
    # A plain git update changes linked files but cannot add a new extras link.
    extras = tmp_path / "extras"
    extras.mkdir()
    source = pathlib.Path(__file__).resolve().parents[1] / "src"
    for name in ("oams.py", "oams_manager.py"):
        (extras / name).symlink_to(source / name)
    result = subprocess.run([sys.executable, "-c", """
import sys, types
sys.modules['mcu'] = types.ModuleType('mcu')
from extras.oams_manager import OAMSManager
assert hasattr(OAMSManager, 'get_status')
"""], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_lane_pressure_follows_the_fps_past_a_deadband():
    manager, unit1, _ = make_manager()
    reading = {"value": 0.5}
    manager.fps = types.SimpleNamespace(get_value=lambda: reading["value"])
    unit1.fps_target = 0.45

    lane = manager.get_status(1.)["lanes"][0]
    assert lane["pressure"] == 0.5
    assert lane["set_point"] == 0.45

    # Noise inside the deadband keeps the published value still.
    reading["value"] = 0.515
    assert manager.get_status(2.)["lanes"][0]["pressure"] == 0.5

    reading["value"] = 0.73
    assert manager.get_status(3.)["lanes"][0]["pressure"] == 0.73

    # Readings outside the rail are clamped to 0..1.
    reading["value"] = 1.2
    assert manager.get_status(4.)["lanes"][0]["pressure"] == 1.0
