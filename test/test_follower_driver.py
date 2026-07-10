# Unit tests for the inline-follower driver (follower.py) without Klipper.
#
# Same harness style as test_oams_driver.py: stub the `mcu` module, drive bare
# instances, fake command objects.
#
#   cd klipper_openams && python3 -m unittest discover -s test

import importlib
import os
import sys
import types
import unittest

_SRC = os.path.join(os.path.dirname(__file__), "..", "src")
if "oamspkg" not in sys.modules:
    _pkg = types.ModuleType("oamspkg")
    _pkg.__path__ = [_SRC]
    sys.modules["oamspkg"] = _pkg
sys.modules.setdefault("mcu", types.ModuleType("mcu"))

follower_mod = importlib.import_module("oamspkg.follower")
Follower = follower_mod.Follower
S = importlib.import_module("oamspkg.oams_state")


class FakeCmd:
    def __init__(self):
        self.sent = []

    def send(self, args=None):
        self.sent.append(args)


class FakeMcu:
    def __init__(self, clock_freq=1000000.0):
        self._freq = clock_freq

    def get_name(self):
        return "follower_mcu"

    def estimated_print_time(self, eventtime):
        return eventtime            # tests use print_time == eventtime

    def print_time_to_clock(self, print_time):
        return int(print_time * self._freq)


class FakeToolhead:
    def __init__(self, flushed_pt):
        self.flushed_pt = flushed_pt

    def get_status(self, eventtime):
        return {"print_time": self.flushed_pt}


class LinearExtruder:
    """Commanded position moves at `v` mm/s (with optional step at t_step)."""

    def __init__(self, v, v2=None, t_step=None):
        self.v = v
        self.v2 = v2
        self.t_step = t_step

    def find_past_position(self, print_time):
        if self.t_step is None or print_time <= self.t_step:
            return self.v * print_time
        return (self.v * self.t_step
                + self.v2 * (print_time - self.t_step))


class FakeError(Exception):
    pass


class FakeReactor:
    """Async callbacks run immediately with a late eventtime (so the
    telemetry reconciliation's recent-local-change guard does not fire
    unless a test arranges it)."""

    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def register_async_callback(self, cb):
        cb(self.now)


def make_follower(steps_per_mm=100.0):
    f = Follower.__new__(Follower)
    f.short_name = "belay"
    f.oams_idx = 9
    f.fps_name = "fps1"
    f.oid = 5
    f.steps_per_mm = steps_per_mm
    f.max_speed = 300.0
    f.path_length = 500.0
    f.ff_sample_period = 0.05
    f.ff_horizon = 1.5
    f.ff_interval = 0.1
    f.fps_forward_interval = 0.1
    f.protocol_version = 1
    f.status_loading = follower_mod.FOLLOWER_STATUS_LOADING
    f.status_unloading = follower_mod.FOLLOWER_STATUS_UNLOADING
    f.status_error = follower_mod.FOLLOWER_STATUS_ERROR
    f.f1s_hes_value = [0, 0, 0, 0]
    f.hub_hes_value = [0, 0, 0, 0]
    f.encoder_clicks = 0
    f.velocity = 0
    f.fps_stale = False
    f.ff_underrun = False
    f.error_latched = False
    f.current_spool = None
    f._following = False
    f._direction = S.FOLLOWER_FORWARD
    f._op_in_flight = False
    f._ff_last_pt = None
    f._ff_last_v = None
    f._ff_last_send = 0.0
    f._ff_capable = True
    f._local_change_time = -10.0
    f.reactor = FakeReactor()
    f.mcu = FakeMcu()
    f._toolhead = None
    f._extruder = None
    f._fps = None
    f.cmd_load = FakeCmd()
    f.cmd_unload = FakeCmd()
    f.cmd_load_cancel = FakeCmd()
    f.cmd_set = FakeCmd()
    f.cmd_fps = FakeCmd()
    f.cmd_ff = FakeCmd()
    f.cmd_clear = FakeCmd()
    f.printer = types.SimpleNamespace(command_error=FakeError,
                                      config_error=FakeError)
    f.completions = []
    f.on_action_complete = lambda code, value, gen: f.completions.append(
        (code, value, gen))
    f.runtime = object()
    return f


class ConversionTests(unittest.TestCase):
    def test_steps_and_fps16(self):
        f = make_follower(steps_per_mm=100.0)
        self.assertEqual(f._steps(500.0), 50000)
        self.assertEqual(f._steps_s(300.0), 30000)
        self.assertEqual(f._fps16(0.0), 0)
        self.assertEqual(f._fps16(1.0), 65535)
        self.assertEqual(f._fps16(2.0), 65535)          # clamped
        self.assertEqual(f._fps16(0.5), 32768)

    def test_gain_q12(self):
        f = make_follower(steps_per_mm=135.0)
        # 40 mm/s per unit FPS error -> steps/s per fps16 count, Q12
        expect = round(40.0 * 135.0 / 65535.0 * 4096.0)
        self.assertEqual(f._gain_q12(40.0), expect)


class SenderTests(unittest.TestCase):
    def test_load_sends_oid_and_gen_bay0_only(self):
        f = make_follower()
        f.start_load_spool(0, gen=7)
        self.assertEqual(f.cmd_load.sent, [[5, 7]])
        self.assertTrue(f._op_in_flight)
        with self.assertRaises(FakeError):
            f.start_load_spool(1, gen=8)                # single-bay unit

    def test_unload_and_cancel(self):
        f = make_follower()
        f.start_unload_spool(gen=3)
        self.assertEqual(f.cmd_unload.sent, [[5, 3]])
        f.load_spool_cancel()
        self.assertEqual(f.cmd_load_cancel.sent, [[5]])

    def test_calibrate_rejects_via_failing_completion(self):
        f = make_follower()
        async_cbs = []
        f.reactor = types.SimpleNamespace(
            register_async_callback=lambda cb: async_cbs.append(cb))
        f.start_calibrate("ptfe", 0, gen=4)
        self.assertEqual(f.completions, [])             # deferred
        for cb in async_cbs:
            cb(0.0)
        self.assertEqual(f.completions,
                         [(S.OAMS_OP_CODE_ERROR_UNSPECIFIED, None, 4)])

    def test_set_follower_gates_streams_and_primes_fps(self):
        f = make_follower()
        f._fps = types.SimpleNamespace(get_value=lambda: 0.5)
        f.set_oams_follower(1, S.FOLLOWER_FORWARD)
        self.assertEqual(f.cmd_set.sent, [[5, 1, S.FOLLOWER_FORWARD]])
        self.assertTrue(f._following)
        self.assertEqual(f.cmd_fps.sent, [[5, 32768]])  # primed immediately


class CompletionTests(unittest.TestCase):
    def test_load_success_mirrors_state_and_forwards_gen(self):
        f = make_follower()
        f._apply_action_status({"action": f.status_loading,
                                "code": S.OAMS_OP_CODE_SUCCESS,
                                "value": 123, "gen": 6})
        self.assertEqual(f.completions, [(S.OAMS_OP_CODE_SUCCESS, 123, 6)])
        self.assertEqual(f.current_spool, 0)
        # firmware auto-starts forward following on load success (mirrored)
        self.assertTrue(f._following)
        self.assertFalse(f._op_in_flight)

    def test_unload_success_clears_spool_and_follow(self):
        f = make_follower()
        f.current_spool = 0
        f._following = True
        f._apply_action_status({"action": f.status_unloading,
                                "code": S.OAMS_OP_CODE_SUCCESS,
                                "value": 0, "gen": 7})
        self.assertIsNone(f.current_spool)
        self.assertFalse(f._following)

    def test_non_terminal_status_dropped(self):
        f = make_follower()
        f._apply_action_status({"action": 2,        # FORWARD_FOLLOWING
                                "code": S.OAMS_OP_CODE_ERROR_BUSY,
                                "value": 0, "gen": 6})
        self.assertEqual(f.completions, [])

    def test_stats_update_mirrors(self):
        f = make_follower()
        f._stats_received({"oid": 5, "pre": 1, "post": 0,
                           "flags": follower_mod.FLAG_FPS_STALE
                           | follower_mod.FLAG_FF_UNDERRUN,
                           "step_count": -1234, "velocity": 400})
        self.assertEqual(f.f1s_hes_value, [1, 0, 0, 0])
        self.assertEqual(f.hub_hes_value, [0, 0, 0, 0])
        self.assertEqual(f.encoder_clicks, -1234)
        self.assertTrue(f.fps_stale)
        self.assertTrue(f.ff_underrun)


class ReviewRegressionTests(unittest.TestCase):
    # Fixes from the adversarial review of the first implementation.

    def test_busy_rejection_keeps_op_in_flight(self):
        # H1: a BUSY status rejects a NEW op while the old one still runs;
        # clearing _op_in_flight would stop the FPS stream and let the
        # firmware stale-watchdog kill the healthy op.
        f = make_follower()
        f._op_in_flight = True
        f._apply_action_status({"action": f.status_loading,
                                "code": S.OAMS_OP_CODE_ERROR_BUSY,
                                "value": 0, "gen": 9})
        self.assertTrue(f._op_in_flight)
        self.assertEqual(f.completions[-1][0], S.OAMS_OP_CODE_ERROR_BUSY)

    def test_stats_flags_reconcile_mirrors(self):
        # H2: telemetry carries the firmware's own following/op/direction
        # truth; the host adopts it, self-healing drift (e.g. a lost
        # terminal status leaving _op_in_flight stuck True).
        f = make_follower()
        f._op_in_flight = True                    # stuck: status was lost
        f._following = True
        f._stats_received({"oid": 5, "pre": 1, "post": 1,
                           "flags": 0,            # firmware: idle
                           "step_count": 0, "velocity": 0})
        self.assertFalse(f._op_in_flight)
        self.assertFalse(f._following)

    def test_stats_reconcile_defers_to_recent_local_change(self):
        f = make_follower()
        f._local_change_time = f.reactor.now - 0.2   # just changed locally
        f._following = True
        f._stats_received({"oid": 5, "pre": 0, "post": 0,
                           "flags": 0, "step_count": 0, "velocity": 0})
        self.assertTrue(f._following)                # old report ignored

    def test_follower_stop_skipped_during_op(self):
        # M1: firmware cmd_set enable=0 hard-aborts an op; broadcast stops
        # (e.g. _stop_followers) must not cancel an in-flight load/reload.
        f = make_follower()
        f._op_in_flight = True
        f.set_oams_follower(0, S.FOLLOWER_REVERSE)
        self.assertEqual(f.cmd_set.sent, [])

    def test_load_refused_without_path_length(self):
        # M5: firmware phase-2 budget would be 0 -> immediate TIMEOUT with
        # filament stuck between the switches; refuse with a clear error.
        f = make_follower()
        f.path_length = 0.0
        with self.assertRaises(FakeError) as cm:
            f.start_load_spool(0, gen=1)
        self.assertIn("path_length", str(cm.exception))
        self.assertEqual(f.cmd_load.sent, [])

    def test_tmc_register_map_has_ifcnt(self):
        # C2: MCU_TMC_uart.set_register() reads IFCNT back after every write.
        self.assertIn("IFCNT", follower_mod._TmcUartInit._REGS)

    def test_clear_errors_mirrors_firmware_hard_stop(self):
        f = make_follower()
        f._following = True
        f._op_in_flight = True
        f.clear_errors()
        self.assertFalse(f._following)
        self.assertFalse(f._op_in_flight)
        self.assertEqual(f.cmd_clear.sent, [[5]])


class FpsForwardTests(unittest.TestCase):
    def test_forward_only_while_active(self):
        f = make_follower()
        f._fps = types.SimpleNamespace(get_value=lambda: 0.25)
        f._fps_forward_event(10.0)
        self.assertEqual(f.cmd_fps.sent, [])            # idle: nothing sent
        f._op_in_flight = True
        f._fps_forward_event(10.1)
        self.assertEqual(f.cmd_fps.sent, [[5, f._fps16(0.25)]])


class FeedForwardTests(unittest.TestCase):
    def _streaming(self, extruder, flushed_pt, now=10.0):
        f = make_follower(steps_per_mm=100.0)
        f._following = True
        f._extruder = extruder
        f._toolhead = FakeToolhead(flushed_pt)
        f._stream_feed_forward(now)
        return f

    def test_constant_velocity_collapses_to_one_segment(self):
        # 5 mm/s commanded, plenty of PLANNED lookahead: delta suppression
        # sends exactly one segment of 500 steps/s, and the window is bounded
        # by the GENERATED horizon (0.35 s), not the planned print_time.
        f = self._streaming(LinearExtruder(5.0), flushed_pt=11.0)
        self.assertEqual(len(f.cmd_ff.sent), 1)
        oid, clock, v = f.cmd_ff.sent[0]
        self.assertEqual(v, 500)
        self.assertEqual(clock, int(10.0 * 1e6))        # segment starts "now"
        cap = 10.0 + follower_mod.FF_GENERATED_HORIZON
        self.assertGreaterEqual(f._ff_last_pt, cap - f.ff_sample_period - 1e-9)
        self.assertLessEqual(f._ff_last_pt, cap + 1e-9)

    def test_velocity_step_produces_second_segment(self):
        # velocity step INSIDE the generated horizon
        f = self._streaming(LinearExtruder(5.0, v2=10.0, t_step=10.2),
                            flushed_pt=11.0)
        vs = [v for (_o, _c, v) in f.cmd_ff.sent]
        self.assertEqual(vs[0], 500)
        self.assertIn(1000, vs)                          # new rate streamed

    def test_never_samples_past_flushed_horizon(self):
        f = self._streaming(LinearExtruder(5.0), flushed_pt=10.2)
        self.assertLessEqual(f._ff_last_pt, 10.2 + 1e-9)

    def test_velocity_clamped_to_max_speed(self):
        f = self._streaming(LinearExtruder(1000.0), flushed_pt=11.0)
        for (_o, _c, v) in f.cmd_ff.sent:
            self.assertLessEqual(abs(v), f._steps_s(f.max_speed))

    def test_empty_window_sends_keepalive(self):
        f = make_follower()
        f._following = True
        f._extruder = LinearExtruder(0.0)
        f._toolhead = FakeToolhead(flushed_pt=10.0)     # nothing flushed ahead
        f._ff_last_send = 0.0
        f._stream_feed_forward(10.0)
        self.assertEqual(len(f.cmd_ff.sent), 1)
        self.assertEqual(f.cmd_ff.sent[0][2], 0)        # v=0 keepalive

    def test_suppressed_cruise_still_refreshes(self):
        # H3: the firmware holds the last segment and treats ~2 s of host
        # silence as underrun, so a long constant-velocity cruise must
        # refresh the (suppressed) velocity at least every 0.5 s.
        f = make_follower(steps_per_mm=100.0)
        f._following = True
        f._extruder = LinearExtruder(5.0)
        f._toolhead = FakeToolhead(flushed_pt=100.0)
        f._stream_feed_forward(10.0)
        self.assertEqual(len(f.cmd_ff.sent), 1)          # initial segment
        f._stream_feed_forward(10.7)                     # > FF_REFRESH_INTERVAL
        self.assertEqual(len(f.cmd_ff.sent), 2)          # refreshed, same v
        self.assertEqual(f.cmd_ff.sent[1][2], 500)

    def test_window_bounded_by_generated_horizon(self):
        # C1: find_past_position goes FLAT past the step-generation horizon;
        # sampling there would stream spurious v=0. The window must never
        # reach past now + FF_GENERATED_HORIZON even with plenty of planned
        # print_time, so no zero-velocity segment is ever produced from the
        # flat region.
        class SaturatingExtruder:
            def find_past_position(self, print_time):
                capped = min(print_time, 10.4)           # generation stopped
                return 5.0 * capped
        f = make_follower(steps_per_mm=100.0)
        f._following = True
        f._extruder = SaturatingExtruder()
        f._toolhead = FakeToolhead(flushed_pt=12.0)      # planned way ahead
        f._stream_feed_forward(10.0)
        self.assertLessEqual(f._ff_last_pt,
                             10.0 + follower_mod.FF_GENERATED_HORIZON + 1e-9)
        for (_o, _c, v) in f.cmd_ff.sent:
            self.assertGreater(v, 0)                     # no flat-region zeros

    def test_window_resumes_where_it_left_off(self):
        ext = LinearExtruder(5.0)
        f = self._streaming(ext, flushed_pt=10.6)
        first_end = f._ff_last_pt
        f._toolhead = FakeToolhead(flushed_pt=11.2)
        f._stream_feed_forward(10.1)
        self.assertGreater(f._ff_last_pt, first_end)


class ProtocolTests(unittest.TestCase):
    def _resolve(self, consts):
        f = make_follower()
        f.mcu = types.SimpleNamespace(get_constants=lambda: consts,
                                      get_name=lambda: "belay_mcu")
        f.name = "follower belay"
        f._resolve_protocol()
        return f

    def test_missing_version_is_hard_error(self):
        with self.assertRaises(FakeError) as cm:
            self._resolve({})
        self.assertIn("FOLLOWER_PROTOCOL_VERSION", str(cm.exception))

    def test_version_and_action_enum_adopted(self):
        f = self._resolve({"FOLLOWER_PROTOCOL_VERSION": 1,
                           "FOLLOWER_STATUS_LOADING": 10,
                           "FOLLOWER_STATUS_ERROR": 17})
        self.assertEqual(f.protocol_version, 1)
        self.assertEqual(f.status_loading, 10)
        self.assertEqual(f.status_error, 17)
        self.assertTrue(f.firmware_owns_liveness)

    def test_old_version_is_hard_error(self):
        with self.assertRaises(FakeError):
            self._resolve({"FOLLOWER_PROTOCOL_VERSION": 0})


if __name__ == "__main__":
    unittest.main(verbosity=2)
