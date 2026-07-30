#!/usr/bin/env python3
import pathlib
import sys
import types


PROJECT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.modules.setdefault("mcu", types.SimpleNamespace())

from src import oams
from src import oams_manager


class FakeReactor:
    NEVER = 1.0e30

    def monotonic(self):
        return 10.0


class FakePrinter:
    def __init__(self, extruder):
        self.extruder = extruder
        self.reactor = FakeReactor()

    def lookup_object(self, name):
        assert name == "extruder"
        return self.extruder

    def get_reactor(self):
        return self.reactor


class FakeOams:
    name = "oams1"
    filament_path_length = 100.0

    def __init__(self, code):
        self.code = code

    def is_bay_ready(self, bay):
        return True

    def load_spool(self, bay):
        return self.code, "result %d" % self.code


def make_manager(code):
    unit = FakeOams(code)
    manager = oams_manager.OAMSManager.__new__(oams_manager.OAMSManager)
    manager.printer = FakePrinter(types.SimpleNamespace(last_position=200.0))
    manager.current_spool = (unit, 0)
    manager.current_group = "T0"
    manager.runout_position = 0.0
    manager.runout_after_position = None
    manager.reload_before_toolhead_distance = 0.0
    manager.filament_groups = {
        "T0": types.SimpleNamespace(bays=[(unit, 1)])}
    manager._register_monitor_spool_timer = lambda: None
    manager._pause_print = lambda: None
    return manager, unit


def test_success_zero_is_success():
    manager, unit = make_manager(oams.OAMS_OP_CODE_SUCCESS)
    result = manager._load_next_spool(10.0, 60.0)
    assert result == manager.printer.reactor.NEVER
    assert manager.current_spool == (unit, 1)
    assert manager.runout_position is None


def test_nonzero_code_is_not_success():
    manager, unit = make_manager(oams.OAMS_OP_CODE_ERROR_BUSY)
    old_spool = manager.current_spool
    manager._load_next_spool(10.0, 60.0)
    assert manager.current_spool == old_spool


def test_cancel_does_not_mark_requested_spool_loaded():
    unit = oams.OAMS.__new__(oams.OAMS)
    unit.current_spool = None
    unit.action_status_code = oams.OAMS_OP_CODE_CANCEL
    code, _ = unit.finish_load_spool(2)
    assert code == oams.OAMS_OP_CODE_CANCEL
    assert unit.current_spool is None


if __name__ == "__main__":
    test_success_zero_is_success()
    test_nonzero_code_is_not_success()
    test_cancel_does_not_mark_requested_spool_loaded()
    print("PASS: OAMS firmware return-code handling")
