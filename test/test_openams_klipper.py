"""Real Klipper G-code/macro integration, with no hardware or service access.

Run with OPENAMS_KLIPPER_DIR pointing to a checkout containing KLIPPER_REV.
Sources are read from that commit, never from a user's modified working tree.
"""
import configparser
import contextlib
import os
from pathlib import Path
import subprocess
import types

import pytest

from test_openams_api import FakeReactor, FakeUnit
from src import oams_manager

ROOT = Path(__file__).resolve().parents[1]
KLIPPER_REV = "57a018ad7bbacde28c35305fd79cb9488bd201db"
LEGACY_REV = "01de61b6ccf406b8bd17e4ff8a5e1250ec942ac1"


def committed_source(repo, revision, path):
    return subprocess.check_output(
        ["git", "-C", str(repo), "show", revision + ":" + path], text=True)


def module_from_source(name, source):
    module = types.ModuleType(name)
    module.__package__ = name.rpartition(".")[0]
    exec(compile(source, name, "exec"), module.__dict__)
    return module


@pytest.fixture(scope="module")
def klipper():
    directory = os.environ.get("OPENAMS_KLIPPER_DIR")
    if not directory:
        pytest.skip("set OPENAMS_KLIPPER_DIR for pinned Klipper integration")
    return types.SimpleNamespace(**{
        name: module_from_source(name, committed_source(directory, KLIPPER_REV, path))
        for name, path in (("gcode", "klippy/gcode.py"),
                           ("gcode_macro", "klippy/extras/gcode_macro.py"))
    })


class Reactor(FakeReactor):
    NOW = 0.
    NEVER = 1.e30

    def mutex(self):
        return contextlib.nullcontext()

    def assert_no_pause(self):
        return contextlib.nullcontext()

    def register_timer(self, callback, when):
        return (callback, when)


class Config:
    def __init__(self, printer, name, values=None):
        self.printer, self.name = printer, name
        self.values = values or {}
        self.error = ValueError

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def get(self, key, default=None):
        return self.values.get(key, default)

    def getfloat(self, key, default):
        return float(self.get(key, default))

    def get_prefix_options(self, prefix):
        return [key for key in self.values if key.startswith(prefix)]


class Status:
    def __init__(self, **values):
        self.values = values

    def get_status(self, now):
        return dict(self.values)


class Printer:
    config_error = ValueError

    def __init__(self, klipper):
        self.objects = {}
        self.events = {}
        self.trace = []
        self.reactor = Reactor()
        self.command_error = klipper.gcode.CommandError
        self.gcode = klipper.gcode.GCodeDispatch(self)
        self.objects["gcode"] = self.gcode
        self.objects["gcode_macro"] = klipper.gcode_macro.PrinterGCodeMacro(
            Config(self, "gcode_macro"))
        self.objects["toolhead"] = Status(homed_axes="xyz")
        self.objects["exclude_object"] = Status(current_object=None, excluded_objects=[])
        self.objects["filament_switch_sensor extruder_in"] = Status(filament_detected=True)
        self.objects["filament_switch_sensor extruder_out"] = Status(filament_detected=False)
        self.objects["fps"] = types.SimpleNamespace(get_value=lambda: 0.5)
        self.objects["webhooks"] = types.SimpleNamespace(register_endpoint=lambda *args: None)
        for command in ("G0", "G1", "G28", "G4", "M400", "M83", "PAUSE",
                        "SET_STEPPER_ENABLE", "SAVE_GCODE_STATE", "RESTORE_GCODE_STATE",
                        "CLEAN_NOZZLE"):
            self.gcode.register_command(command, self.record)
        self.gcode.register_command("RESPOND", lambda command: None)

    def record(self, command):
        self.trace.append(command.get_commandline())

    def get_start_args(self):
        return {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def lookup_objects(self, module=None):
        return [(name, obj) for name, obj in self.objects.items()
                if module is None or name.split()[0] == module]

    def load_object(self, config, name):
        return self.objects[name]

    def register_event_handler(self, name, handler):
        self.events.setdefault(name, []).append(handler)

    def send_event(self, name):
        for handler in self.events.get(name, []):
            handler()

    def invoke_shutdown(self, message):
        raise AssertionError(message)


class Unit(FakeUnit):
    def __init__(self, printer):
        super().__init__(1, [True] * 4)
        self.printer = printer

    def start_load_spool(self, bay):
        self.printer.trace.append("feed:%d" % bay)
        super().start_load_spool(bay)

    def unload_spool(self):
        self.printer.trace.append("unload")
        return super().unload_spool()

    def set_oams_follower(self, enable, direction):
        self.printer.trace.append("follower:%d:%d" % (enable, direction))


@pytest.fixture
def setup(klipper):
    def build(legacy_manager=False, legacy_macros=False, loaded=None):
        printer = Printer(klipper)
        unit = Unit(printer)
        if loaded is not None:
            unit.current_spool = loaded
            unit.hub_hes_value[loaded] = True
        printer.objects["oams unit1"] = unit
        printer.objects["filament_group T0"] = types.SimpleNamespace(bays=[(unit, 0), (unit, 1)])
        printer.objects["filament_group T1"] = types.SimpleNamespace(bays=[(unit, 2), (unit, 3)])
        manager_module = oams_manager
        if legacy_manager:
            manager_module = module_from_source("src.legacy_manager", committed_source(
                ROOT, LEGACY_REV, "src/oams_manager.py"))
        manager = manager_module.OAMSManager(Config(printer, "oams_manager"))
        printer.objects["oams_manager"] = manager
        sample = configparser.RawConfigParser(inline_comment_prefixes=("#", ";"))
        sample.read(ROOT / "oams_sample.cfg")
        name = "gcode_macro _oams_macro_variables"
        printer.objects[name] = klipper.gcode_macro.GCodeMacro(
            Config(printer, name, dict(sample[name])))
        macros = (committed_source(ROOT, LEGACY_REV, "oams_macros.cfg")
                  if legacy_macros else (ROOT / "oams_macros.cfg").read_text())
        config = configparser.RawConfigParser(inline_comment_prefixes=("#", ";"))
        config.read_string(macros)
        for name in config.sections():
            printer.objects[name] = klipper.gcode_macro.GCodeMacro(
                Config(printer, name, dict(config[name])))
        printer.send_event("klippy:ready")
        return printer, manager, unit
    return build


@pytest.mark.parametrize("case", ["load", "same_group", "switch", "load_error", "empty",
                                 "unload", "unload_error", "excluded", "sensor_error"])
@pytest.mark.parametrize("legacy_macros", [True, False])
def test_legacy_execution_matches_master(setup, case, legacy_macros):
    traces = []
    states = []
    for old in (True, False):
        loaded = 0 if case in ("same_group", "switch", "unload", "unload_error") else None
        printer, manager, unit = setup(old, True if old else legacy_macros, loaded)
        if case == "load_error":
            unit.load_result = 2
        if case == "empty":
            unit.f1s_hes_value = [False] * 4
        if case == "unload_error":
            unit.unload_result = False
        if case == "excluded":
            printer.objects["exclude_object"].values.update(current_object="part", excluded_objects=["part"])
        if case == "sensor_error":
            printer.objects["gcode_macro _oams_macro_variables"].variables["fs_extruder_in"] = True
            printer.objects["filament_switch_sensor extruder_in"].values["filament_detected"] = False
        command = ("SAFE_UNLOAD_FILAMENT" if case.startswith("unload")
                   else "_TX GROUP=" + ("T1" if case == "switch" else "T0"))
        printer.gcode.run_script(command)
        traces.append(printer.trace)
        states.append((manager.current_group, unit.current_spool, manager.current_state.name))
    assert traces[0] == traces[1]
    assert states[0] == states[1]


@pytest.mark.parametrize("target", ["GROUP=T1 SLOT=-1", "GROUP=T1 SLOT=99", "GROUP=T1 SLOT=0",
                                    "GROUP=T1 SLOT=oops", "GROUP=T1 SLOT=2.9", "GROUP=missing SLOT=2"])
def test_ui_invalid_target_has_no_motion(setup, target):
    printer, manager, unit = setup(loaded=0)
    printer.objects["toolhead"].values["homed_axes"] = ""
    with pytest.raises(printer.command_error):
        printer.gcode.run_script("OPENAMS_LOAD " + target)
    assert printer.trace == []
    assert unit.current_spool == 0


def test_ui_switches_selected_bay_within_same_group(setup):
    printer, manager, unit = setup(loaded=0)
    printer.gcode.run_script("OPENAMS_LOAD GROUP=T0 SLOT=1")
    assert manager.current_group == "T0"
    assert unit.current_spool == 1
    assert printer.trace.index("unload") < printer.trace.index("feed:1")


def test_ui_unload_failure_stops_before_new_feed(setup):
    printer, _, unit = setup(loaded=0)
    unit.unload_result = False
    with pytest.raises(printer.command_error, match="unload failed"):
        printer.gcode.run_script("OPENAMS_LOAD GROUP=T1 SLOT=2")
    assert unit.loaded_bays == []
    assert "CLEAN_NOZZLE" not in printer.trace


def test_ui_load_failure_stops_before_extruder_reload(setup):
    printer, _, unit = setup()
    unit.load_result = 6
    with pytest.raises(printer.command_error, match="load failed"):
        printer.gcode.run_script("OPENAMS_LOAD GROUP=T1 SLOT=2")
    assert printer.trace == ["feed:2"]


def test_ui_sensor_failure_aborts_outer_macro(setup):
    printer, _, _ = setup()
    printer.objects["gcode_macro _oams_macro_variables"].variables["fs_extruder_in"] = True
    printer.objects["filament_switch_sensor extruder_in"].values["filament_detected"] = False
    with pytest.raises(printer.command_error, match="Filament not detected"):
        printer.gcode.run_script("OPENAMS_LOAD GROUP=T1 SLOT=2")
    assert not any(command.startswith("G1") for command in printer.trace)
    assert "CLEAN_NOZZLE" not in printer.trace


def test_ui_unload_of_empty_machine_does_not_move(setup):
    printer, _, _ = setup()
    printer.gcode.run_script("OPENAMS_UNLOAD")
    assert printer.trace == []


def test_ui_unload_homes_before_extrusion_and_uses_relative_mode(setup):
    printer, _, _ = setup(loaded=0)
    printer.objects["toolhead"].values["homed_axes"] = ""
    printer.gcode.run_script("OPENAMS_UNLOAD")
    assert printer.trace[0:2] == ["G28", "M83"]
    assert "unload" in printer.trace


def test_ui_load_from_empty_uses_relative_extrusion(setup):
    printer, _, unit = setup()
    printer.gcode.run_script("OPENAMS_LOAD GROUP=T1 SLOT=3")
    first_extrusion = next(i for i, command in enumerate(printer.trace)
                           if command.startswith("G1 "))
    assert printer.trace[first_extrusion - 1] == "M83"
    assert unit.current_spool == 3


def test_ui_failed_cut_sensor_stops_before_hardware_unload(setup):
    printer, _, unit = setup(loaded=0)
    printer.objects["gcode_macro _oams_macro_variables"].variables["fs_extruder_out"] = True
    printer.objects["filament_switch_sensor extruder_out"].values["filament_detected"] = True
    with pytest.raises(printer.command_error, match="Filament still detected"):
        printer.gcode.run_script("OPENAMS_LOAD GROUP=T1 SLOT=2")
    assert unit.unload_calls == 0
    assert unit.loaded_bays == []


@pytest.mark.parametrize("command", ["OPENAMS_LOAD SLOT=2", "OPENAMS_LOAD GROUP=T1"])
def test_ui_missing_required_parameters_does_not_move(setup, command):
    printer, _, _ = setup(loaded=0)
    with pytest.raises(printer.command_error, match="requires"):
        printer.gcode.run_script(command)
    assert printer.trace == []


def test_ui_not_ready_does_not_move(setup):
    printer, manager, unit = setup(loaded=0)
    manager.ready = False
    with pytest.raises(printer.command_error, match="not ready"):
        printer.gcode.run_script("OPENAMS_LOAD GROUP=T1 SLOT=2")
    assert printer.trace == []
    assert unit.current_spool == 0
