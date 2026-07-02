# Unit tests for the OAMS driver's protocol handling (oams.py) without Klipper.
#
# oams.py does `import mcu` and uses package-relative imports, so we stub the
# `mcu` module and expose src/ as a synthetic package, then drive individual
# methods on a bare instance (no real config / MCU / reactor needed).
#
#   cd klipper_openams && python3 -m unittest discover -s test

import importlib
import os
import sys
import types
import unittest
from collections import deque

_SRC = os.path.join(os.path.dirname(__file__), "..", "src")
if "oamspkg" not in sys.modules:
    _pkg = types.ModuleType("oamspkg")
    _pkg.__path__ = [_SRC]
    sys.modules["oamspkg"] = _pkg
# Stub the Klipper `mcu` module that oams.py imports at top level.
sys.modules.setdefault("mcu", types.ModuleType("mcu"))

oams = importlib.import_module("oamspkg.oams")
OAMS = oams.OAMS
S = importlib.import_module("oamspkg.oams_state")


class FakeCmd:
    def __init__(self):
        self.sent = []

    def send(self, args=None):
        self.sent.append(args)


def make_oams(use_gen_protocol):
    """A bare OAMS with just the fields the methods under test touch."""
    o = OAMS.__new__(OAMS)
    o.oams_idx = 1
    o._use_gen_protocol = use_gen_protocol
    o._gen_queue = deque()
    o._pending_bay = None
    o.current_spool = None
    o.action_status = None
    o.action_status_code = None
    o.action_status_value = None
    o.protocol_version = 2 if use_gen_protocol else None
    # action enum (as resolved by _resolve_protocol)
    o.status_loading = oams.OAMS_STATUS_LOADING
    o.status_unloading = oams.OAMS_STATUS_UNLOADING
    o.status_calibrating = oams.OAMS_STATUS_CALIBRATING
    o.status_error = oams.OAMS_STATUS_ERROR
    # command handles
    o.oams_load_spool_cmd = FakeCmd()
    o.oams_unload_spool_cmd = FakeCmd()
    o.oams_calibrate_ptfe_length_cmd = FakeCmd()
    o.oams_calibrate_hub_hes_cmd = FakeCmd()
    o.oams_load_spool2_cmd = FakeCmd()
    o.oams_unload_spool2_cmd = FakeCmd()
    o.oams_calibrate_ptfe_length2_cmd = FakeCmd()
    o.oams_calibrate_hub_hes2_cmd = FakeCmd()
    # completion sink
    o.completions = []
    o.on_action_complete = lambda code, value, gen: o.completions.append(
        (code, value, gen))
    return o


class WireGenTests(unittest.TestCase):
    def test_none_is_zero(self):
        self.assertEqual(OAMS._wire_gen(None), 0)

    def test_masks_to_byte(self):
        self.assertEqual(OAMS._wire_gen(0), 0)
        self.assertEqual(OAMS._wire_gen(255), 255)
        self.assertEqual(OAMS._wire_gen(300), 300 & 0xFF)


class SenderRoutingTests(unittest.TestCase):
    def test_gen_protocol_sends_gen_on_wire_no_fifo(self):
        o = make_oams(use_gen_protocol=True)
        o.start_load_spool(2, gen=7)
        self.assertEqual(o.oams_load_spool2_cmd.sent, [[2, 7]])
        self.assertEqual(o.oams_load_spool_cmd.sent, [])      # legacy untouched
        self.assertEqual(list(o._gen_queue), [])              # no FIFO use
        self.assertEqual(o.action_status, oams.OAMS_STATUS_LOADING)

    def test_legacy_protocol_uses_fifo_no_gen_on_wire(self):
        o = make_oams(use_gen_protocol=False)
        o.start_load_spool(2, gen=7)
        self.assertEqual(o.oams_load_spool_cmd.sent, [[2]])
        self.assertEqual(o.oams_load_spool2_cmd.sent, [])
        self.assertEqual(list(o._gen_queue), [7])

    def test_unload_and_calibrate_routing(self):
        o = make_oams(use_gen_protocol=True)
        o.start_unload_spool(gen=3)
        o.start_calibrate("ptfe", 1, gen=4)
        o.start_calibrate("hub_hes", 0, gen=5)
        self.assertEqual(o.oams_unload_spool2_cmd.sent, [[3]])
        self.assertEqual(o.oams_calibrate_ptfe_length2_cmd.sent, [[1, 4]])
        self.assertEqual(o.oams_calibrate_hub_hes2_cmd.sent, [[0, 5]])

    def test_standalone_gen_none_sends_zero(self):
        o = make_oams(use_gen_protocol=True)
        o.start_load_spool(0)            # gen defaults to None (standalone path)
        self.assertEqual(o.oams_load_spool2_cmd.sent, [[0, 0]])

    def test_send_failure_leaves_no_phantom_fifo_entry(self):
        # On legacy firmware a FIFO entry for an op whose send() raised would
        # desync completion matching for every subsequent op.
        class BoomCmd:
            def send(self, args=None):
                raise RuntimeError("mcu shutdown")

        o = make_oams(use_gen_protocol=False)
        o.oams_load_spool_cmd = BoomCmd()
        with self.assertRaises(RuntimeError):
            o.start_load_spool(2, gen=7)
        self.assertEqual(list(o._gen_queue), [])
        self.assertIsNone(o.action_status)     # sentinel not left armed either


class GenSourcingTests(unittest.TestCase):
    def test_status2_uses_wire_gen(self):
        o = make_oams(use_gen_protocol=True)
        o.action_status = oams.OAMS_STATUS_LOADING
        o._apply_action_status(
            {"action": oams.OAMS_STATUS_LOADING,
             "code": S.OAMS_OP_CODE_SUCCESS, "value": 0}, wire_gen=9)
        self.assertEqual(o.completions, [(S.OAMS_OP_CODE_SUCCESS, None, 9)])
        self.assertIsNone(o.action_status)

    def test_legacy_pops_fifo(self):
        o = make_oams(use_gen_protocol=False)
        o._gen_queue.append(11)
        o.action_status = oams.OAMS_STATUS_UNLOADING
        o._apply_action_status(
            {"action": oams.OAMS_STATUS_UNLOADING,
             "code": S.OAMS_OP_CODE_SUCCESS, "value": 0})
        self.assertEqual(o.completions, [(S.OAMS_OP_CODE_SUCCESS, None, 11)])
        self.assertEqual(list(o._gen_queue), [])

    def test_unsolicited_completion_class_has_no_gen(self):
        # Empty FIFO + no wire gen -> gen=None, which the reducer rejects.
        o = make_oams(use_gen_protocol=False)
        o._apply_action_status(
            {"action": oams.OAMS_STATUS_ERROR,
             "code": S.OAMS_OP_CODE_ERROR_UNSPECIFIED, "value": 0})
        self.assertEqual(o.completions, [(S.OAMS_OP_CODE_ERROR_UNSPECIFIED,
                                          None, None)])

    def test_follower_status_is_dropped(self):
        # Non-completion action (forward following) must not notify the store.
        o = make_oams(use_gen_protocol=True)
        o._gen_queue.append(1)
        o._apply_action_status(
            {"action": oams.OAMS_STATUS_FORWARD_FOLLOWING,
             "code": S.OAMS_OP_CODE_ERROR_BUSY, "value": 0}, wire_gen=1)
        self.assertEqual(o.completions, [])

    def test_load_success_updates_spool_mirror(self):
        o = make_oams(use_gen_protocol=True)
        o._pending_bay = 3
        o._apply_action_status(
            {"action": oams.OAMS_STATUS_LOADING,
             "code": S.OAMS_OP_CODE_SUCCESS, "value": 0}, wire_gen=2)
        self.assertEqual(o.current_spool, 3)


class ResolveProtocolTests(unittest.TestCase):
    def _oams_with_consts(self, consts):
        o = OAMS.__new__(OAMS)
        o.oams_idx = 1
        o.status_loading = oams.OAMS_STATUS_LOADING
        o.status_unloading = oams.OAMS_STATUS_UNLOADING
        o.status_calibrating = oams.OAMS_STATUS_CALIBRATING
        o.status_error = oams.OAMS_STATUS_ERROR
        o.protocol_version = None
        o.mcu = types.SimpleNamespace(get_constants=lambda: consts)
        return o

    def test_legacy_no_constants_keeps_defaults(self):
        o = self._oams_with_consts({})
        o._resolve_protocol()
        self.assertIsNone(o.protocol_version)
        self.assertEqual(o.status_loading, oams.OAMS_STATUS_LOADING)

    def test_published_action_enum_is_adopted(self):
        # A firmware that renumbers the action enum is followed (driver-local).
        o = self._oams_with_consts({
            "OAMS_PROTOCOL_VERSION": 2,
            "OAMS_STATUS_LOADING": 10, "OAMS_STATUS_UNLOADING": 11,
            "OAMS_STATUS_CALIBRATING": 16, "OAMS_STATUS_ERROR": 17,
        })
        o._resolve_protocol()
        self.assertEqual(o.protocol_version, 2)
        self.assertEqual(o.status_loading, 10)
        self.assertEqual(o.status_error, 17)

    def test_string_constants_are_coerced(self):
        # Dictionary values may arrive as strings; they must coerce, not
        # TypeError inside handle_connect (which would skip all command
        # lookups and kill the unit).
        o = self._oams_with_consts({
            "OAMS_PROTOCOL_VERSION": "3",
            "OAMS_STATUS_LOADING": "0",
        })
        o._resolve_protocol()                # must not raise
        self.assertEqual(o.protocol_version, 3)
        self.assertEqual(o.status_loading, 0)

    def test_garbage_constants_fall_back_to_defaults(self):
        o = self._oams_with_consts({
            "OAMS_PROTOCOL_VERSION": "not-a-number",
            "OAMS_STATUS_LOADING": "junk",
        })
        o._resolve_protocol()                # must not raise
        self.assertIsNone(o.protocol_version)          # -> legacy mode
        self.assertEqual(o.status_loading, oams.OAMS_STATUS_LOADING)

    def test_get_constants_failure_is_tolerated(self):
        o = OAMS.__new__(OAMS)
        o.oams_idx = 1
        o.status_loading = oams.OAMS_STATUS_LOADING
        o.status_unloading = oams.OAMS_STATUS_UNLOADING
        o.status_calibrating = oams.OAMS_STATUS_CALIBRATING
        o.status_error = oams.OAMS_STATUS_ERROR
        o.protocol_version = None

        def boom():
            raise RuntimeError("no constants yet")
        o.mcu = types.SimpleNamespace(get_constants=boom)
        o._resolve_protocol()                # must not raise
        self.assertIsNone(o.protocol_version)


class LivenessPropertyTests(unittest.TestCase):
    def test_v3_owns_liveness(self):
        o = make_oams(use_gen_protocol=True)
        o.protocol_version = 3
        self.assertTrue(o.firmware_owns_liveness)

    def test_v2_does_not_own_liveness(self):
        o = make_oams(use_gen_protocol=True)
        o.protocol_version = 2
        self.assertFalse(o.firmware_owns_liveness)

    def test_legacy_does_not_own_liveness(self):
        o = make_oams(use_gen_protocol=False)
        o.protocol_version = None
        self.assertFalse(o.firmware_owns_liveness)


if __name__ == "__main__":
    unittest.main(verbosity=2)
