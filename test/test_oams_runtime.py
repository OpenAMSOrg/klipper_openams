# Unit tests for the OpenAMS runtime (oams_runtime.py) using a fake reactor.
#
# No Klipper, no hardware, no pytest required:
#   cd klipper_openams && python3 -m unittest discover -s test -v

import importlib
import os
import sys
import types
import unittest

# oams_runtime uses package-relative imports (it is installed into
# klippy/extras as part of a package), so expose src/ as a synthetic package.
_SRC = os.path.join(os.path.dirname(__file__), "..", "src")
if "oamspkg" not in sys.modules:
    _pkg = types.ModuleType("oamspkg")
    _pkg.__path__ = [_SRC]
    sys.modules["oamspkg"] = _pkg

S = importlib.import_module("oamspkg.oams_state")
oams_runtime = importlib.import_module("oamspkg.oams_runtime")
Runtime = oams_runtime.Runtime


FPS = "fps1"
UNIT = (1, 0)


class FakeCompletion:
    def __init__(self, events):
        self._events = events
        self._done = False
        self._result = None

    def test(self):
        return self._done

    def complete(self, result):
        self._done = True
        self._result = result
        self._events.append(("settle", result))

    def wait(self, *args):
        return self._result


class FakeReactor:
    NEVER = 9e99

    def __init__(self, events):
        self._events = events
        self.now = 0.0
        self.timers = []          # mutable [callback, waketime] pairs
        self.async_cbs = []

    def monotonic(self):
        return self.now

    def completion(self):
        return FakeCompletion(self._events)

    def register_timer(self, callback, waketime):
        timer = [callback, waketime]
        self.timers.append(timer)
        return timer

    def unregister_timer(self, timer):
        if timer in self.timers:
            self.timers.remove(timer)

    def register_async_callback(self, callback):
        self.async_cbs.append(callback)

    def run_async_callbacks(self):
        callbacks, self.async_cbs = self.async_cbs, []
        for callback in callbacks:
            callback(self.now)

    def fire_due_timers(self):
        for timer in list(self.timers):
            if timer[1] <= self.now:
                timer[1] = timer[0](self.now)


class FakeGcode:
    def __init__(self, events):
        self._events = events

    def run_script(self, script):
        self._events.append(("script", script))


class FakePrinter:
    def __init__(self, reactor, gcode):
        self._reactor = reactor
        self._gcode = gcode

    def get_reactor(self):
        return self._reactor

    def lookup_object(self, name):
        if name == "gcode":
            return self._gcode
        raise KeyError(name)


class FakeOam:
    def __init__(self):
        self.encoder_clicks = 0
        self.calls = []

    def start_load_spool(self, bay, gen=None):
        self.calls.append(("load", bay, gen))

    def start_unload_spool(self, gen=None):
        self.calls.append(("unload", gen))

    def start_calibrate(self, kind, bay, gen=None):
        self.calls.append(("calibrate", kind, bay, gen))

    def load_spool_cancel(self):
        self.calls.append(("cancel_load",))

    def set_oams_follower(self, enable, direction):
        self.calls.append(("follower", enable, direction))

    def set_led_error(self, idx, value):
        self.calls.append(("led", idx, value))


def make_runtime(build_world=None, oam=None):
    events = []
    reactor = FakeReactor(events)
    printer = FakePrinter(reactor, FakeGcode(events))
    if build_world is None:
        build_world = lambda now: S.World(lanes={FPS: S.LaneWorld()})
    if oam is None:
        oam = FakeOam()
    runtime = Runtime(printer, [FPS],
                      build_world, lambda idx: oam if idx == UNIT[0] else None)
    return runtime, reactor, oam, events


class DispatchSafetyTests(unittest.TestCase):
    def test_reducer_exception_settles_pending_waiter(self):
        # A gcode handler blocked in request(...).wait() holds the gcode mutex;
        # a dispatch that dies must fail the op, never leave it hanging.
        def broken_world(now):
            raise RuntimeError("boom")

        runtime, reactor, oam, events = make_runtime(build_world=broken_world)
        completion = runtime.request(FPS, S.LoadBay(FPS, UNIT))
        self.assertTrue(completion.test())
        self.assertFalse(completion.wait().ok)

    def test_start_effect_failure_fails_op(self):
        # resolve_oam returning None (bad oams_idx / not configured) must fail
        # the op promptly instead of stranding the lane until the deadline.
        runtime, reactor, oam, events = make_runtime()
        completion = runtime.request(FPS, S.LoadBay(FPS, (99, 0)))
        self.assertFalse(completion.test())     # failure is deferred one cycle
        reactor.run_async_callbacks()
        self.assertTrue(completion.test())
        self.assertFalse(completion.wait().ok)
        self.assertEqual(runtime.get_state().lanes[FPS].op, S.OP_UNLOADED)

    def test_world_only_built_for_world_actions(self):
        # A transient world-build failure must not be able to kill the
        # reduction of a completion, so completions must not build the world.
        calls = []

        def counting_world(now):
            calls.append(now)
            return S.World(lanes={FPS: S.LaneWorld()})

        runtime, reactor, oam, events = make_runtime(build_world=counting_world)
        runtime.request(FPS, S.LoadBay(FPS, UNIT))
        self.assertEqual(len(calls), 1)                   # LoadBay reads it
        gen = oam.calls[-1][2]
        runtime.dispatch(S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, gen=gen))
        self.assertEqual(len(calls), 1)                   # OpCompleted does not
        self.assertEqual(runtime.get_state().lanes[FPS].op, S.OP_LOADED)

    def test_superseded_request_settles_previous_waiter(self):
        runtime, reactor, oam, events = make_runtime()
        first = runtime.request(FPS, S.LoadBay(FPS, UNIT))
        self.assertFalse(first.test())          # op in flight
        second = runtime.request(FPS, S.LoadBay(FPS, UNIT))
        self.assertTrue(first.test())           # old waiter not leaked
        self.assertFalse(first.wait().ok)


class CompletionTests(unittest.TestCase):
    def test_completion_with_matching_gen_settles_ok(self):
        runtime, reactor, oam, events = make_runtime()
        completion = runtime.request(FPS, S.LoadBay(FPS, UNIT))
        gen = oam.calls[-1][2]
        runtime.dispatch(S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, gen=gen))
        self.assertTrue(completion.wait().ok)
        self.assertEqual(runtime.get_state().lanes[FPS].op, S.OP_LOADED)
        self.assertEqual(runtime._deadlines, {})       # timer cleaned up
        self.assertEqual(reactor.timers, [])

    def test_completion_with_stale_gen_is_ignored(self):
        runtime, reactor, oam, events = make_runtime()
        completion = runtime.request(FPS, S.LoadBay(FPS, UNIT))
        gen = oam.calls[-1][2]
        runtime.dispatch(S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, gen=gen - 1))
        self.assertFalse(completion.test())
        self.assertEqual(runtime.get_state().lanes[FPS].op, S.OP_LOADING)

    def test_deadline_times_out_and_cancels_firmware_load(self):
        runtime, reactor, oam, events = make_runtime()
        completion = runtime.request(FPS, S.LoadBay(FPS, UNIT))
        reactor.now = S.OAMS_ACTION_TIMEOUT + 1.0
        reactor.fire_due_timers()
        self.assertTrue(completion.test())
        self.assertFalse(completion.wait().ok)
        self.assertEqual(runtime.get_state().lanes[FPS].op, S.OP_UNLOADED)
        self.assertIn(("cancel_load",), oam.calls)


class StallTests(unittest.TestCase):
    def _start_stalled_load(self):
        runtime, reactor, oam, events = make_runtime()
        completion = runtime.request(FPS, S.LoadBay(FPS, UNIT))
        reactor.now = oams_runtime.STALL_AFTER + 3.0   # past the grace window
        return runtime, reactor, oam, events, completion

    def test_stall_fails_op_and_pauses(self):
        runtime, reactor, oam, events, completion = self._start_stalled_load()
        runtime.tick()                                 # first encoder sample
        self.assertFalse(completion.test())
        runtime.tick()                                 # second identical sample
        self.assertTrue(completion.test())
        self.assertFalse(completion.wait().ok)
        scripts = [e[1] for e in events if e[0] == "script"]
        self.assertIn("PAUSE", scripts)

    def test_stall_settles_waiter_before_pausing(self):
        # The waiter may hold the gcode mutex PAUSE needs; the settle must come
        # first or the monitor wedges until the op deadline.
        runtime, reactor, oam, events, completion = self._start_stalled_load()
        runtime.tick()
        runtime.tick()
        kinds = [e[0] for e in events]
        self.assertIn("settle", kinds)
        self.assertLess(kinds.index("settle"),
                        kinds.index("script"))

    def test_moving_encoder_is_not_a_stall(self):
        runtime, reactor, oam, events, completion = self._start_stalled_load()
        runtime.tick()
        oam.encoder_clicks += 100
        runtime.tick()
        self.assertFalse(completion.test())

    def test_runout_reload_stall_cancels_and_pauses(self):
        # Legacy firmware: a jammed runout auto-reload (op stays LOADED,
        # runout==RUNOUT_LOADING) must be caught by the host stall detector,
        # not left to feed the print a dwindling tail until the deadline.
        events = []
        reactor = FakeReactor(events)
        printer = FakePrinter(reactor, FakeGcode(events))
        oam = FakeOam()
        lw = S.LaneWorld(extruder_pos=1000.0, printing=True,
                         loaded={UNIT: False, (1, 3): True},
                         ready={(1, 3): True},
                         group_bays={"G": (UNIT, (1, 3))},
                         path_len={1: 600.0})
        runtime = Runtime(printer, [FPS],
                          lambda now: S.World(lanes={FPS: lw}),
                          lambda idx: oam if idx == UNIT[0] else None)
        reloading = S.LaneState(op=S.OP_LOADED, group="G", unit=UNIT,
                                runout=S.RUNOUT_LOADING, reload_target=(1, 3),
                                op_gen=4, since=0.0)
        runtime.system = S.SystemState(lanes={FPS: reloading})
        reactor.now = oams_runtime.STALL_AFTER + 3.0
        runtime.tick()                                 # first encoder sample
        runtime.tick()                                 # identical -> stall
        lane = runtime.get_state().lanes[FPS]
        self.assertEqual(lane.op, S.OP_UNLOADED)       # reload failed over
        self.assertIn(("cancel_load",), oam.calls)     # firmware op cancelled
        scripts = [e[1] for e in events if e[0] == "script"]
        self.assertEqual(scripts.count("PAUSE"), 1)    # paused exactly once


class LivenessTests(unittest.TestCase):
    def test_coarse_deadline_armed_when_firmware_owns_liveness(self):
        runtime, reactor, oam, events = make_runtime()
        runtime.set_firmware_liveness(True)
        runtime.request(FPS, S.LoadBay(FPS, UNIT))
        self.assertEqual(len(reactor.timers), 1)
        waketime = reactor.timers[0][1]
        self.assertAlmostEqual(waketime, reactor.now + S.OAMS_DISCONNECT_BACKSTOP)

    def test_authoritative_deadline_armed_for_legacy(self):
        runtime, reactor, oam, events = make_runtime()
        runtime.request(FPS, S.LoadBay(FPS, UNIT))
        waketime = reactor.timers[0][1]
        self.assertAlmostEqual(waketime, reactor.now + S.OAMS_ACTION_TIMEOUT)

    def test_stall_check_skipped_when_firmware_owns_liveness(self):
        runtime, reactor, oam, events = make_runtime()
        runtime.set_firmware_liveness(True)
        completion = runtime.request(FPS, S.LoadBay(FPS, UNIT))
        reactor.now = oams_runtime.STALL_AFTER + 3.0     # past the grace window
        runtime.tick()
        runtime.tick()                                   # would stall if checked
        self.assertFalse(completion.test())
        self.assertEqual(runtime.get_state().lanes[FPS].op, S.OP_LOADING)


if __name__ == "__main__":
    unittest.main(verbosity=2)
