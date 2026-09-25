"""Real Klipper G-code/macro integration, with no hardware or service access.

Run with OPENAMS_KLIPPER_DIR pointing to a checkout containing KLIPPER_REV.
Sources are read from that commit, never from a user's modified working tree.
"""
import configparser
import contextlib
import os
import re
from pathlib import Path
import subprocess
import types

import pytest

from test_openams_api import FakeReactor, FakeUnit
from src import oams_manager

ROOT = Path(__file__).resolve().parents[1]
KLIPPER_REV = "57a018ad7bbacde28c35305fd79cb9488bd201db"
LEGACY_REV = "01de61b6ccf406b8bd17e4ff8a5e1250ec942ac1"
CONTROLS = ("START", "END", "WAIT")


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
        self.received_controls = []
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
        self.objects["extruder"] = Status(can_extrude=True, temperature=220.0)
        self.objects["pause_resume"] = Status(is_paused=False)
        # Klipper publishes lowercase section names with their accessed options.
        self.objects["configfile"] = Status(settings={
            "filament_group t0": {"group": "oams1-0, oams1-1"},
            "filament_group t1": {"group": "oams1-2, oams1-3"},
        })
        self.objects["fps"] = types.SimpleNamespace(get_value=lambda: 0.5)
        self.objects["webhooks"] = types.SimpleNamespace(register_endpoint=lambda *args: None)
        for command in ("G0", "G1", "G28", "G4", "G90", "M400", "M83",
                        "SET_STEPPER_ENABLE", "SAVE_GCODE_STATE", "RESTORE_GCODE_STATE",
                        "CLEAN_NOZZLE"):
            self.gcode.register_command(command, self.record)
        self.gcode.register_command("PAUSE", self.pause)
        self.gcode.register_command("RESPOND", lambda command: None)
        # Stock Klipper has no routine controls. Any rendered control is a bug.
        for command in CONTROLS:
            self.gcode.register_command(command, self.control)

    def record(self, command):
        self.trace.append(command.get_commandline())

    def pause(self, command):
        self.record(command)
        self.objects["pause_resume"].values["is_paused"] = True

    def control(self, command):
        self.received_controls.append(command.get_commandline())
        raise AssertionError("stock Klipper received " + command.get_commandline())

    def enable_gco_routines(self):
        """Advertise a live extension and an ordered _TX without providing it."""
        self.objects["gco_routines"] = Status()
        self.objects["configfile"].values["settings"]["gcode_macro _tx"] = {
            "render_mode": "ordered"}

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
    def build(legacy_manager=False, legacy_macros=False, loaded=None, legacy_sample=False):
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
        sample.read_string(committed_source(ROOT, LEGACY_REV, "oams_sample.cfg")
                           if legacy_sample else (ROOT / "oams_sample.cfg").read_text())
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


CASES = ["load", "same_group", "switch", "load_error", "empty",
         "unload", "unload_error", "excluded", "sensor_error"]


def run_case(setup, case, legacy_manager, legacy_macros):
    loaded = 0 if case in ("same_group", "switch", "unload", "unload_error") else None
    printer, manager, unit = setup(legacy_manager, legacy_macros, loaded)
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
    error = None
    try:
        printer.gcode.run_script(command)
    except printer.command_error as exc:
        error = exc
    state = (manager.current_group, unit.current_spool, manager.current_state.name)
    return printer, state, error


def is_reload(command):
    return command.startswith("G1 E") and not command.startswith("G1 E-")


@pytest.mark.parametrize("case", CASES)
def test_legacy_execution_matches_master(setup, case):
    """The current manager runs the legacy macros exactly as the legacy manager."""
    legacy, legacy_state, legacy_error = run_case(setup, case, True, True)
    current, current_state, current_error = run_case(setup, case, False, True)
    assert legacy_error is None and current_error is None
    assert legacy.trace == current.trace
    assert legacy_state == current_state


@pytest.mark.parametrize("case", CASES)
def test_current_macros_preserve_master_outcomes(setup, case):
    """The shipped macros reach master's manager state through a safer sequence.

    Exact trace equality with the legacy macros cannot hold, because the
    toolchange sequence changed deliberately: G-code state is saved before and
    restored after the change, the nozzle is cleaned after the OpenAMS load and
    before the toolhead reload (so the concurrent path can overlap cleaning with
    loading), there is no absolute G0 Z15 before cleaning, and every safety
    decision is made in a fresh helper render. Master's legacy renders decided
    from stale state, so they extruded and cleaned after a failed load or
    sensor check; the new macros pause and raise instead. This test pins the
    ordering and safety properties per case instead of the byte-for-byte trace.
    """
    _, legacy_state, _ = run_case(setup, case, True, True)
    printer, state, error = run_case(setup, case, False, False)
    trace = printer.trace
    variables = printer.objects["gcode_macro _oams_macro_variables"].variables
    cut = "G0 X%s Y%s F%s" % (variables["cut_x"], variables["cut_y"], variables["cut_speed"])
    loads = case in ("load", "switch")
    failures = ("load_error", "empty", "unload_error", "sensor_error")
    feeds = [index for index, command in enumerate(trace) if command.startswith("feed:")]
    reloads = [index for index, command in enumerate(trace) if is_reload(command)]

    assert state == legacy_state
    if case in failures:
        assert error is not None and "PAUSE" in trace
        assert not any("MOVE=1" in command for command in trace)
    else:
        assert error is None and "PAUSE" not in trace
    if case in ("same_group", "excluded"):
        assert trace == []
    # Cut before unload; feed only after the unload finished.
    if "unload" in trace:
        assert trace.index(cut) < trace.index("unload")
        assert all(index > trace.index("unload") for index in feeds)
    if case == "unload_error":
        assert feeds == []
    # Reload extrusion only after a successful feed; none after a failure.
    assert len(reloads) == (1 if loads else 0)
    assert all(feeds and index > feeds[-1] for index in reloads)
    assert trace.count("CLEAN_NOZZLE") == (1 if loads else 0)
    if loads:
        # Master's settle dwell still precedes the inlet check.
        assert trace[feeds[-1] + 1:feeds[-1] + 3] == ["M400", "G4 P1000"]
        assert feeds[-1] < trace.index("CLEAN_NOZZLE") < reloads[0]
        assert trace[reloads[0] - 1] == "M83"
        assert trace[0] == "SAVE_GCODE_STATE NAME=oams_toolchange"
        assert trace[-1] == "RESTORE_GCODE_STATE NAME=oams_toolchange MOVE=1 MOVE_SPEED=100"
    assert not any(command.startswith("G0 Z") for command in trace)


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
    assert printer.trace == ["SAVE_GCODE_STATE NAME=oams_toolchange", "feed:2"]


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
    assert printer.trace[0:3] == ["G28", "SAVE_GCODE_STATE NAME=oams_unload", "M83"]
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


def render(printer, macro, **params):
    """Render a macro exactly as GCodeMacro.cmd would, without dispatching it."""
    macro = printer.objects["gcode_macro " + macro]
    context = dict(macro.variables)
    context.update(macro.template.create_template_context())
    context["params"] = params
    context["rawparams"] = " ".join("%s=%s" % item for item in params.items())
    return [line.strip() for line in macro.template.render(context).splitlines()
            if line.strip()]


@pytest.mark.parametrize("params, load", [
    ({"GROUP": "T1"}, "OAMSM_LOAD_FILAMENT GROUP=T1"),
    ({"GROUP": "T1", "SLOT": "2"}, "OAMSM_LOAD_FILAMENT GROUP=T1 SLOT=2"),
])
def test_concurrent_branch_renders_literal_controls_only_when_enabled(setup, params, load):
    printer, _, _ = setup()
    assert not set(CONTROLS) & set(render(printer, "_TX", **params))
    printer.enable_gco_routines()
    lines = render(printer, "_TX", **params)
    start = lines.index("START")
    strict = 1 if "SLOT" in params else 0
    assert lines[start:] == [
        "START", load, "END", "CLEAN_NOZZLE", "M400", "WAIT",
        "_OAMS_CONTINUE_AFTER_LOAD GROUP=T1 SLOT=%s STRICT=%d STARTED_PAUSED=0"
        % (params.get("SLOT", -1), strict)]
    # Stock Klipper would receive the controls, which the fake rejects.
    with pytest.raises(AssertionError):
        printer.gcode.run_script("_TX " + " ".join("%s=%s" % item for item in params.items()))
    assert printer.received_controls == ["START"]


@pytest.mark.parametrize("loaded", [None, 0])
@pytest.mark.parametrize("command", ["_TX GROUP=T1", "OPENAMS_LOAD GROUP=T1 SLOT=2"])
def test_paused_toolchange_takes_serial_path_even_with_extension(setup, loaded, command):
    printer, manager, unit = setup(loaded=loaded)
    printer.enable_gco_routines()
    printer.objects["pause_resume"].values["is_paused"] = True
    printer.gcode.run_script(command)
    assert (manager.current_group, unit.current_spool) == ("T1", 2)
    trace = printer.trace
    assert "PAUSE" not in trace
    assert trace.index("feed:2") < trace.index("CLEAN_NOZZLE") < trace.index("G1 E31.2 F1000")
    assert trace[-1] == "RESTORE_GCODE_STATE NAME=oams_toolchange MOVE=1 MOVE_SPEED=100"


def test_pause_during_unload_stops_before_feed(setup):
    printer, manager, unit = setup(loaded=0)
    unload_spool = unit.unload_spool

    def unload_then_pause():
        printer.objects["pause_resume"].values["is_paused"] = True
        return unload_spool()
    unit.unload_spool = unload_then_pause
    with pytest.raises(printer.command_error, match="printer was paused"):
        printer.gcode.run_script("T1")
    assert unit.loaded_bays == []
    assert not any(is_reload(command) or command == "CLEAN_NOZZLE" for command in printer.trace)


@pytest.mark.parametrize("variant", ["cannot_extrude", "below_minimum"])
@pytest.mark.parametrize("command, loaded", [
    ("T1", 0), ("T1", None), ("OPENAMS_LOAD GROUP=T1 SLOT=2", 0),
    ("OPENAMS_LOAD GROUP=T1 SLOT=2", None), ("SAFE_UNLOAD_FILAMENT", 0), ("OPENAMS_UNLOAD", 0)])
def test_cold_extruder_is_rejected_before_any_motion(setup, variant, command, loaded):
    printer, manager, unit = setup(loaded=loaded)
    printer.objects["toolhead"].values["homed_axes"] = ""
    if variant == "cannot_extrude":
        printer.objects["extruder"].values["can_extrude"] = False
    else:
        printer.objects["gcode_macro _oams_macro_variables"].variables[
            "minimum_extrude_temperature"] = 230
    with pytest.raises(printer.command_error, match="Heat the extruder"):
        printer.gcode.run_script(command)
    assert printer.trace == []
    assert unit.current_spool == loaded and unit.unload_calls == 0


def test_unknown_group_is_rejected_before_any_motion(setup):
    printer, _, unit = setup(loaded=0)
    printer.objects["toolhead"].values["homed_axes"] = ""
    with pytest.raises(printer.command_error, match="Unknown OpenAMS filament group T7"):
        printer.gcode.run_script("_TX GROUP=T7")
    assert printer.trace == []
    assert unit.current_spool == 0


def test_existing_variable_sections_use_master_defaults(setup):
    """oams.cfg files created from the previous sample lack the new variables."""
    printer, manager, unit = setup(loaded=0, legacy_sample=True)
    variables = printer.objects["gcode_macro _oams_macro_variables"].variables
    assert not {"unload_speed", "extrusion_unload_additional_length",
                "additional_unload_speed", "minimum_extrude_temperature"} & set(variables)
    printer.gcode.run_script("T1")
    assert (manager.current_group, unit.current_spool) == ("T1", 2)
    assert "G1 E-%s F1000" % variables["extrusion_unload_length"] in printer.trace
    # Master computed 5000 / 60 * (2 + 1) mm at 5000 mm/min.
    assert "G1 E-250 F5000" in printer.trace


EXPECTED_MACROS = {
    "CUT_FILAMENT", "SAFE_UNLOAD_FILAMENT", "_OAMS_FINISH_UNLOAD", "_OAMS_CONFIRM_UNLOADED",
    "_TX", "_OAMS_CONTINUE_AFTER_UNLOAD", "_OAMS_CONTINUE_AFTER_LOAD",
    "_OAMS_CONTINUE_AFTER_INLET", "_OAMS_FINISH_TOOLCHANGE", "_OAMS_FAIL_TOOLCHANGE",
    "_OAMS_ABORT", "_OAMS_RAISE", "OPENAMS_LOAD", "OPENAMS_UNLOAD",
    "_LOAD_FS_IN", "_LOAD_FS_OUT", "_UNLOAD_FS_OUT",
    "OAMS_TORTURE_TEST", "OAMS_TOOLCHANGE_TORTURE_TEST", "T0", "T1", "T2", "T3"}
NEW_VARIABLES = {"extrusion_unload_additional_length": 250, "unload_speed": 1000,
                 "additional_unload_speed": 5000, "minimum_extrude_temperature": 0}


def parse_config(name):
    config = configparser.RawConfigParser(inline_comment_prefixes=("#", ";"))
    config.read_string((ROOT / name).read_text())
    return config


def test_macro_files_define_expected_sections_and_only_overlay_sets_render_mode():
    macros = parse_config("oams_macros.cfg")
    assert set(macros.sections()) == {"gcode_macro " + name for name in EXPECTED_MACROS}
    for section in macros.sections():
        assert macros[section]["gcode"].strip(), section
        assert "render_mode" not in macros[section], section
    overlay = parse_config("oams_macros_ordered.cfg")
    assert overlay.sections() == ["gcode_macro _TX"]
    assert dict(overlay["gcode_macro _TX"]) == {"render_mode": "ordered"}
    sample = parse_config("oams_sample.cfg")
    assert not any("render_mode" in sample[section] for section in sample.sections())


def test_controls_are_literal_lines_inside_one_guarded_branch():
    macros = parse_config("oams_macros.cfg")
    for section in macros.sections():
        lines = [line.strip() for line in macros[section]["gcode"].splitlines()]
        if section != "gcode_macro _TX":
            assert not any(line.split(" ")[0].upper() in CONTROLS for line in lines), section
            continue
        assert [line for line in lines if line in CONTROLS] == list(CONTROLS)
        start, wait = lines.index("START"), lines.index("WAIT")
        branch = [line for line in lines[:start] if line.startswith("{%")][-1]
        assert branch.startswith("{% if HAS_GCO_ROUTINES and not printer.pause_resume.is_paused")
        assert not any(line.startswith(("{% el", "{% endif")) for line in lines[start:wait])


def test_new_variables_have_defaults_and_sample_values():
    text = (ROOT / "oams_macros.cfg").read_text()
    legacy = configparser.RawConfigParser(inline_comment_prefixes=("#", ";"))
    legacy.read_string(committed_source(ROOT, LEGACY_REV, "oams_sample.cfg"))
    known = {option[len("variable_"):] for option in legacy["gcode_macro _oams_macro_variables"]}
    used = set(re.findall(r"\bv\.(\w+)", text))
    assert set(NEW_VARIABLES) <= used
    for name in used - known:
        for match in re.finditer(r"\bv\.%s\b(.{0,9})" % name, text):
            assert match.group(1).startswith("|default("), name
        assert "v.%s|default(%s)" % (name, NEW_VARIABLES[name]) in text
    sample = parse_config("oams_sample.cfg")["gcode_macro _oams_macro_variables"]
    assert {name: int(sample["variable_" + name]) for name in NEW_VARIABLES} == NEW_VARIABLES
    includes = (ROOT / "oams_sample.cfg").read_text().splitlines()
    index = includes.index("[include oams_macros.cfg]")
    assert includes[index + 1] == "#[include oams_macros_ordered.cfg]"
