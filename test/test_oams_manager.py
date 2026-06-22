# Manager-layer tests with a fake Klipper printer/config harness.
#
# These exercise the manager's config validation (force-load + topology) and the
# runtime group-edit commands without a real Klipper instance, by driving the
# methods on a bare OAMSManager with fakes.
#
#   cd klipper_openams && python3 -m unittest discover -s test

import importlib
import os
import sys
import tempfile
import types
import unittest

_SRC = os.path.join(os.path.dirname(__file__), "..", "src")
if "oamspkg" not in sys.modules:
    _pkg = types.ModuleType("oamspkg")
    _pkg.__path__ = [_SRC]
    sys.modules["oamspkg"] = _pkg
sys.modules.setdefault("mcu", types.ModuleType("mcu"))

S = importlib.import_module("oamspkg.oams_state")
T = importlib.import_module("oamspkg.oams_topology")
manager_mod = importlib.import_module("oamspkg.oams_manager")
OAMSManager = manager_mod.OAMSManager


class FakeError(Exception):
    pass


class FakeSection:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class FakeConfig:
    def __init__(self, section_names):
        self._names = section_names

    def get_prefix_sections(self, prefix):
        return [FakeSection(n) for n in self._names if n.startswith(prefix)]

    def error(self, msg):
        return FakeError(msg)


class FakeOam:
    def __init__(self, idx, fps=None):
        self.oams_idx = idx
        self.fps_name = fps
        self.hub_hes_value = [0, 0, 0, 0]
        self.f1s_hes_value = [0, 0, 0, 0]
        self.filament_path_length = 600
        self.protocol_version = None
        self.oams_load_spool_cmd = None
        self._use_gen_protocol = False

    @property
    def firmware_owns_liveness(self):
        return self.protocol_version is not None and self.protocol_version >= 3


class FakeFps:
    def __init__(self, name):
        self.fps_name = name


class FakeGroup:
    def __init__(self, bay_specs):
        self.bay_specs = bay_specs


class FakeConfigfile:
    def __init__(self):
        self.sets = []

    def set(self, section, option, value):
        self.sets.append((section, option, value))


class FakePrinter:
    def __init__(self, objects, configfile):
        # objects: {module: [(full_name, obj), ...]}
        self._objects = objects
        self._configfile = configfile
        self.loaded = []
        self.command_error = FakeError

    def load_object(self, config, name):
        self.loaded.append(name)

    def lookup_objects(self, module):
        return list(self._objects.get(module, []))

    def lookup_object(self, name):
        if name == "configfile":
            return self._configfile
        raise KeyError(name)


class FakeGcmd:
    def __init__(self, params):
        self._p = params
        self.responses = []

    def get(self, key, default="__required__"):
        if key in self._p:
            return self._p[key]
        if default == "__required__":
            raise FakeError("missing %s" % key)
        return default

    def get_int(self, key, minval=None, maxval=None):
        return int(self._p[key])

    def error(self, msg):
        return FakeError(msg)

    def respond_info(self, msg):
        self.responses.append(msg)


class FakeRuntime:
    def __init__(self, lanes):
        self._sys = S.SystemState(lanes=lanes)

    def get_state(self):
        return self._sys


DEFAULT_GROUPS_FILE = """\
# OpenAMS groups
[filament_group T0]
group: oams1-0

[filament_group T1]
group: oams1-1
"""


def build_manager(objects, section_names, lanes=None,
                  groups_file=DEFAULT_GROUPS_FILE):
    """A bare manager with topology built from fakes, ready for edit commands.
    Group edits are persisted to a real temp file so the writeback is exercised
    end to end; the path is on mgr.openams_config_path."""
    mgr = OAMSManager.__new__(OAMSManager)
    mgr.config = FakeConfig(section_names)
    mgr.printer = FakePrinter(objects, FakeConfigfile())
    mgr._build_topology()
    if lanes is None:
        lanes = {fps: S.LaneState() for fps in mgr.topo.fps_names}
    mgr.runtime = FakeRuntime(lanes)
    fd, path = tempfile.mkstemp(suffix=".cfg")
    os.write(fd, groups_file.encode())
    os.close(fd)
    mgr.openams_config_path = path
    return mgr


def read_file(path):
    with open(path) as f:
        return f.read()


def single_lane_objects():
    return {
        "fps": [("fps", FakeFps("fps1"))],
        "oams": [("oams oams1", FakeOam(1))],
        "filament_group": [
            ("filament_group T0", FakeGroup([("oams1", 0)])),
            ("filament_group T1", FakeGroup([("oams1", 1)])),
        ],
    }


def single_lane_sections():
    return ["fps", "oams oams1", "filament_group T0", "filament_group T1",
            "oams_manager"]


class BuildTopologyTests(unittest.TestCase):
    def test_valid_single_lane(self):
        mgr = build_manager(single_lane_objects(), single_lane_sections())
        self.assertEqual(mgr.topo.lane_of_oams("oams1"), "fps1")
        self.assertEqual(mgr.group_lane["T0"], "fps1")
        self.assertEqual(mgr.groups_by_lane["fps1"]["T1"], ((1, 1),))
        self.assertEqual([o.oams_idx for o in mgr.lane_oams["fps1"]], [1])
        # every fps/oams/filament_group section was force-loaded
        self.assertIn("oams oams1", mgr.printer.loaded)
        self.assertIn("filament_group T0", mgr.printer.loaded)

    def test_invalid_group_reference_is_config_error(self):
        objs = single_lane_objects()
        objs["filament_group"] = [
            ("filament_group T0", FakeGroup([("oamsX", 0)]))]
        with self.assertRaises(FakeError) as cm:
            build_manager(objs, single_lane_sections())
        self.assertIn("unknown OAMS", str(cm.exception))

    def test_order_independence(self):
        # filament_group sections listed BEFORE the oams section: must still
        # validate, because the group is self-contained and the manager
        # force-loads + validates centrally.
        objs = single_lane_objects()
        sections = ["filament_group T0", "filament_group T1", "fps",
                    "oams oams1", "oams_manager"]
        mgr = build_manager(objs, sections)
        self.assertEqual(mgr.group_lane["T0"], "fps1")

    def test_multi_lane_requires_explicit_fps(self):
        objs = {
            "fps": [("fps a", FakeFps("a")), ("fps b", FakeFps("b"))],
            "oams": [("oams oams1", FakeOam(1, fps=None))],
            "filament_group": [],
        }
        with self.assertRaises(FakeError) as cm:
            build_manager(objs, ["fps a", "fps b", "oams oams1"])
        self.assertIn("must set 'fps:'", str(cm.exception))


class GroupEditTests(unittest.TestCase):
    def test_create_group_persists_empty(self):
        mgr = build_manager(single_lane_objects(), single_lane_sections())
        mgr.cmd_CREATE_GROUP(FakeGcmd({"GROUP": "NEW"}))
        self.assertIn("NEW", mgr.topo.groups)
        text = read_file(mgr.openams_config_path)
        self.assertIn("[filament_group NEW]", text)
        # original sections preserved
        self.assertIn("[filament_group T0]", text)
        self.assertIn("# OpenAMS groups", text)

    def test_assign_bay_moves_and_persists(self):
        mgr = build_manager(single_lane_objects(), single_lane_sections())
        mgr.cmd_ASSIGN_BAY(FakeGcmd({"GROUP": "T0", "OAMS": "oams1", "BAY": 1}))
        # bay (oams1,1) moved from T1 into T0; model + file both updated
        self.assertIn(("oams1", 1), mgr.topo.groups["T0"])
        self.assertNotIn(("oams1", 1), mgr.topo.groups["T1"])
        self.assertEqual(mgr.groups_by_lane["fps1"]["T0"], ((1, 0), (1, 1)))
        text = read_file(mgr.openams_config_path)
        self.assertIn("group: oams1-0,oams1-1", text)   # T0 updated in place
        self.assertIn("[filament_group T1]\ngroup:", text)  # T1 emptied

    def test_assign_unknown_oams_errors(self):
        mgr = build_manager(single_lane_objects(), single_lane_sections())
        with self.assertRaises(FakeError):
            mgr.cmd_ASSIGN_BAY(
                FakeGcmd({"GROUP": "T0", "OAMS": "oamsX", "BAY": 0}))

    def test_edit_refused_while_lane_busy(self):
        lanes = {"fps1": S.LaneState(op=S.OP_LOADING, unit=(1, 0), op_gen=1)}
        mgr = build_manager(single_lane_objects(), single_lane_sections(),
                            lanes=lanes)
        with self.assertRaises(FakeError) as cm:
            mgr.cmd_ASSIGN_BAY(
                FakeGcmd({"GROUP": "T0", "OAMS": "oams1", "BAY": 1}))
        self.assertIn("busy", str(cm.exception))

    def test_edit_refused_while_group_loaded(self):
        lanes = {"fps1": S.LaneState(op=S.OP_LOADED, group="T0", unit=(1, 0))}
        mgr = build_manager(single_lane_objects(), single_lane_sections(),
                            lanes=lanes)
        with self.assertRaises(FakeError) as cm:
            mgr.cmd_UNASSIGN_BAY(
                FakeGcmd({"GROUP": "T0", "OAMS": "oams1", "BAY": 0}))
        self.assertIn("loaded", str(cm.exception))

    def test_unassign_bay_persists(self):
        mgr = build_manager(single_lane_objects(), single_lane_sections())
        mgr.cmd_UNASSIGN_BAY(
            FakeGcmd({"GROUP": "T0", "OAMS": "oams1", "BAY": 0}))
        self.assertEqual(mgr.topo.groups["T0"], ())
        text = read_file(mgr.openams_config_path)
        self.assertIn("[filament_group T0]\ngroup:", text)
        self.assertNotIn("oams1-0", text)

    def test_delete_group_removes_section(self):
        mgr = build_manager(single_lane_objects(), single_lane_sections())
        mgr.cmd_DELETE_GROUP(FakeGcmd({"GROUP": "T0"}))
        self.assertNotIn("T0", mgr.topo.groups)
        text = read_file(mgr.openams_config_path)
        self.assertNotIn("[filament_group T0]", text)
        self.assertIn("[filament_group T1]", text)      # sibling kept

    def test_write_failure_leaves_model_unchanged(self):
        mgr = build_manager(single_lane_objects(), single_lane_sections())
        mgr.openams_config_path = "/nonexistent_dir/oams.cfg"
        before = mgr.topo.groups
        with self.assertRaises(FakeError):
            mgr.cmd_ASSIGN_BAY(
                FakeGcmd({"GROUP": "T0", "OAMS": "oams1", "BAY": 1}))
        # file write failed -> running model untouched (no divergence)
        self.assertEqual(mgr.topo.groups, before)


class SelfTestTests(unittest.TestCase):
    def _oam(self, idx, connected=True, protocol=3):
        o = FakeOam(idx)
        o.oams_load_spool_cmd = object() if connected else None
        o.protocol_version = protocol
        o._use_gen_protocol = protocol is not None and protocol >= 2
        o.f1s_hes_value = [1, 0, 0, 1]
        return o

    def test_selftest_pass(self):
        objs = single_lane_objects()
        oam = self._oam(1, connected=True, protocol=3)
        objs["oams"] = [("oams oams1", oam)]
        mgr = build_manager(objs, single_lane_sections())
        # FakeFps needs get_value/extruder for the report
        for fps in mgr.fpss.values():
            fps.get_value = lambda: 0.5
            fps.extruder_name = "extruder"
            fps.extruder = object()
        gcmd = FakeGcmd({})
        mgr.cmd_SELFTEST(gcmd)
        out = gcmd.responses[0]
        self.assertIn("RESULT: PASS", out)
        self.assertIn("protocol=3", out)

    def test_selftest_warns_on_disconnected_unit(self):
        objs = single_lane_objects()
        objs["oams"] = [("oams oams1", self._oam(1, connected=False))]
        mgr = build_manager(objs, single_lane_sections())
        for fps in mgr.fpss.values():
            fps.get_value = lambda: 0.0
            fps.extruder_name = "extruder"
            fps.extruder = object()
        gcmd = FakeGcmd({})
        mgr.cmd_SELFTEST(gcmd)
        self.assertIn("WARN", gcmd.responses[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
