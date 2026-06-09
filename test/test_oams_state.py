# Pure unit tests for the OpenAMS reducer (oams_state.py).
#
# No Klipper, no hardware, no pytest required:
#   cd klipper_openams && python3 -m unittest discover -s test -v
#   (or: python3 test/test_oams_state.py)

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import oams_state as S


FPS = "fps1"
GROUP = "T0"
BAY_A = (1, 0)   # (oams_idx, bay)
BAY_B = (1, 3)


def world(extruder_pos=0.0, printing=False, loaded=None, ready=None,
          path_len=600.0, reload_before=0.0):
    lw = S.LaneWorld(
        extruder_pos=extruder_pos,
        printing=printing,
        loaded=loaded if loaded is not None else {},
        ready=ready if ready is not None else {},
        group_bays={GROUP: (BAY_A, BAY_B)},
        path_len={1: path_len},
        reload_before=reload_before,
    )
    return S.World(lanes={FPS: lw})


def system(lane):
    return S.SystemState(lanes={FPS: lane})


def lane_of(sysstate):
    return sysstate.lanes[FPS]


def find(effects, cls):
    return [e for e in effects if isinstance(e, cls)]


def has(effects, cls):
    return bool(find(effects, cls))


class LoadTests(unittest.TestCase):
    def test_load_ready_bay_starts_loading(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADED))
        w = world(ready={BAY_A: True})
        sys1, fx = S.reduce(sys0, S.Load(FPS, GROUP), w, now=10.0)
        lane = lane_of(sys1)
        self.assertEqual(lane.op, S.OP_LOADING)
        self.assertEqual(lane.unit, BAY_A)
        self.assertEqual(find(fx, S.StartLoad)[0].unit, BAY_A)
        self.assertTrue(has(fx, S.ArmDeadline))

    def test_load_none_ready_stays_unloaded(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADED))
        w = world(ready={BAY_A: False, BAY_B: False})
        sys1, fx = S.reduce(sys0, S.Load(FPS, GROUP), w, now=0.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertFalse(has(fx, S.StartLoad))
        self.assertFalse(find(fx, S.Settle)[0].result.ok)

    def test_load_completion_success(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS),
                            world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADED)
        self.assertEqual(lane_of(sys1).unit, BAY_A)
        self.assertTrue(find(fx, S.Settle)[0].result.ok)
        self.assertTrue(has(fx, S.CancelDeadline))

    def test_load_completion_error(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_ERROR_BUSY),
                            world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertIsNone(lane_of(sys1).unit)
        self.assertFalse(find(fx, S.Settle)[0].result.ok)

    def test_cancel_then_completed_is_not_loaded(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.Cancel(FPS), world(), now=1.0)
        self.assertEqual(find(fx, S.CancelLoad)[0].unit, BAY_A)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADING)  # still loading until ack
        sys2, fx2 = S.reduce(sys1, S.OpCompleted(FPS, S.OAMS_OP_CODE_CANCEL),
                             world(), now=2.0)
        self.assertEqual(lane_of(sys2).op, S.OP_UNLOADED)
        self.assertIsNone(lane_of(sys2).unit)          # NOT recorded as loaded
        self.assertFalse(find(fx2, S.Settle)[0].result.ok)

    def test_load_timeout(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.Timeout(FPS), world(), now=999.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertFalse(find(fx, S.Settle)[0].result.ok)
        self.assertFalse(has(fx, S.SetFollower))  # nothing to rewind on a load


class LoadBayTests(unittest.TestCase):
    def test_load_bay_starts_and_resolves_group(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADED))
        sys1, fx = S.reduce(sys0, S.LoadBay(FPS, BAY_B), world(), now=0.0)
        lane = lane_of(sys1)
        self.assertEqual(lane.op, S.OP_LOADING)
        self.assertEqual(lane.unit, BAY_B)
        self.assertEqual(lane.group, GROUP)              # resolved from group_bays
        self.assertEqual(find(fx, S.StartLoad)[0].unit, BAY_B)

    def test_load_bay_completion(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_B, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS),
                            world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADED)
        self.assertEqual(lane_of(sys1).unit, BAY_B)

    def test_load_bay_rejected_when_busy(self):
        sys0 = system(S.LaneState(op=S.OP_LOADED, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.LoadBay(FPS, BAY_B), world(), now=0.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADED)   # unchanged
        self.assertEqual(lane_of(sys1).unit, BAY_A)
        self.assertFalse(has(fx, S.StartLoad))
        self.assertFalse(find(fx, S.Settle)[0].result.ok)


class UnloadTests(unittest.TestCase):
    def test_unload_starts(self):
        sys0 = system(S.LaneState(op=S.OP_LOADED, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.Unload(FPS), world(), now=0.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADING)
        self.assertEqual(find(fx, S.StartUnload)[0].unit, BAY_A)

    def test_unload_success(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS),
                            world(), now=0.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertIsNone(lane_of(sys1).unit)
        self.assertTrue(find(fx, S.Settle)[0].result.ok)

    def test_unload_failure_stops_follower_and_stays_loaded(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_ERROR_BUSY),
                            world(), now=0.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADED)
        sf = find(fx, S.SetFollower)
        self.assertTrue(sf and sf[0].enable == 0)


class CalibrateTests(unittest.TestCase):
    def test_calibrate_async_and_return_to_prior(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADED))
        sys1, fx = S.reduce(sys0, S.Calibrate(FPS, oams_idx=1, bay=0, kind="ptfe"),
                            world(), now=0.0)
        self.assertEqual(lane_of(sys1).op, S.OP_CALIBRATING)
        self.assertEqual(find(fx, S.StartCalibrate)[0].kind, "ptfe")
        sys2, fx2 = S.reduce(sys1, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, value=1234),
                             world(), now=1.0)
        self.assertEqual(lane_of(sys2).op, S.OP_UNLOADED)
        res = find(fx2, S.Settle)[0].result
        self.assertTrue(res.ok)
        self.assertEqual(res.value, 1234)


class RunoutTests(unittest.TestCase):
    def _loaded_lane(self):
        return S.LaneState(op=S.OP_LOADED, group=GROUP, unit=BAY_A)

    def test_happy_path(self):
        lane = self._loaded_lane()
        # 1) runout detected -> PAUSING
        w = world(extruder_pos=1000.0, printing=True,
                  loaded={BAY_A: False, BAY_B: True},
                  ready={BAY_A: False, BAY_B: True})
        s, fx = S.reduce(system(lane), S.Tick(), w, now=1.0)
        self.assertEqual(lane_of(s).runout, S.RUNOUT_PAUSING)

        # 2) advance >= PAUSE_DISTANCE -> COASTING + coast follower off
        w = world(extruder_pos=1000.0 + S.PAUSE_DISTANCE, printing=True,
                  loaded={BAY_A: False, BAY_B: True},
                  ready={BAY_A: False, BAY_B: True})
        s, fx = S.reduce(s, S.Tick(), w, now=2.0)
        self.assertEqual(lane_of(s).runout, S.RUNOUT_COASTING)
        self.assertTrue(has(fx, S.SetFollower))

        # 3) consume beyond path_len/FACTOR -> LOADING the next group bay
        far = 1000.0 + S.PAUSE_DISTANCE + (600.0 / S.FILAMENT_PATH_LENGTH_FACTOR)
        w = world(extruder_pos=far + 1.0, printing=True,
                  loaded={BAY_A: False, BAY_B: True},
                  ready={BAY_A: False, BAY_B: True})
        s, fx = S.reduce(s, S.Tick(), w, now=3.0)
        self.assertEqual(lane_of(s).runout, S.RUNOUT_LOADING)
        self.assertEqual(find(fx, S.StartLoad)[0].unit, BAY_B)

        # 4) reload completes -> unit switched, back to idle
        s, fx = S.reduce(s, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS), w, now=4.0)
        self.assertEqual(lane_of(s).op, S.OP_LOADED)
        self.assertEqual(lane_of(s).unit, BAY_B)
        self.assertEqual(lane_of(s).runout, S.RUNOUT_IDLE)

    def test_uncalibrated_pauses(self):
        lane = S.LaneState(op=S.OP_LOADED, group=GROUP, unit=BAY_A,
                           runout=S.RUNOUT_COASTING, coast_origin=0.0)
        w = world(extruder_pos=10.0, printing=True,
                  loaded={BAY_A: False}, ready={BAY_B: True}, path_len=0.0)
        s, fx = S.reduce(system(lane), S.Tick(), w, now=1.0)
        self.assertTrue(has(fx, S.Pause))
        self.assertEqual(lane_of(s).op, S.OP_UNLOADED)

    def test_no_spare_pauses(self):
        lane = S.LaneState(op=S.OP_LOADED, group=GROUP, unit=BAY_A,
                           runout=S.RUNOUT_COASTING, coast_origin=0.0)
        far = S.PAUSE_DISTANCE + 600.0 / S.FILAMENT_PATH_LENGTH_FACTOR + 1.0
        # only the ran-out bay looks "ready" -> must NOT be chosen
        w = world(extruder_pos=far, printing=True,
                  loaded={BAY_A: False}, ready={BAY_A: True, BAY_B: False})
        s, fx = S.reduce(system(lane), S.Tick(), w, now=1.0)
        self.assertFalse(has(fx, S.StartLoad))
        self.assertTrue(has(fx, S.Pause))
        self.assertEqual(lane_of(s).op, S.OP_UNLOADED)

    def test_reload_failure_pauses(self):
        lane = S.LaneState(op=S.OP_LOADED, group=GROUP, unit=BAY_A,
                           runout=S.RUNOUT_LOADING, reload_target=BAY_B)
        s, fx = S.reduce(system(lane),
                         S.OpCompleted(FPS, S.OAMS_OP_CODE_ERROR_BUSY),
                         world(), now=1.0)
        self.assertTrue(has(fx, S.Pause))
        self.assertEqual(lane_of(s).op, S.OP_UNLOADED)

    def test_idle_tick_is_noop(self):
        lane = self._loaded_lane()
        w = world(extruder_pos=5.0, printing=True, loaded={BAY_A: True})
        s, fx = S.reduce(system(lane), S.Tick(), w, now=1.0)
        self.assertEqual(fx, [])
        self.assertEqual(lane_of(s).runout, S.RUNOUT_IDLE)


class ClearErrorsTests(unittest.TestCase):
    def test_resync_loaded(self):
        sys0 = system(S.LaneState(op=S.OP_PAUSED))
        w = world(loaded={BAY_B: True})
        s, fx = S.reduce(sys0, S.ClearErrors(), w, now=1.0)
        self.assertEqual(lane_of(s).op, S.OP_LOADED)
        self.assertEqual(lane_of(s).unit, BAY_B)
        self.assertEqual(lane_of(s).group, GROUP)

    def test_resync_unloaded(self):
        sys0 = system(S.LaneState(op=S.OP_LOADED, unit=BAY_A))
        w = world(loaded={BAY_A: False, BAY_B: False})
        s, fx = S.reduce(sys0, S.ClearErrors(), w, now=1.0)
        self.assertEqual(lane_of(s).op, S.OP_UNLOADED)
        self.assertIsNone(lane_of(s).unit)


class MultiLaneTests(unittest.TestCase):
    # fps1 owns group T0 (bays on oams 1); fps2 owns group T1 (bays on oams 2).
    def _world(self, fps1_kwargs=None, fps2_kwargs=None):
        lw1 = S.LaneWorld(group_bays={"T0": ((1, 0), (1, 3))}, path_len={1: 600.0},
                          **(fps1_kwargs or {}))
        lw2 = S.LaneWorld(group_bays={"T1": ((2, 0),)}, path_len={2: 600.0},
                          **(fps2_kwargs or {}))
        return S.World(lanes={"fps1": lw1, "fps2": lw2})

    def _system(self, l1, l2):
        return S.SystemState(lanes={"fps1": l1, "fps2": l2})

    def test_load_routes_to_correct_lane(self):
        sys0 = self._system(S.LaneState(), S.LaneState())
        w = self._world(fps2_kwargs={"ready": {(2, 0): True}})
        sys1, fx = S.reduce(sys0, S.Load("fps2", "T1"), w, now=0.0)
        self.assertEqual(sys1.lanes["fps2"].op, S.OP_LOADING)
        self.assertEqual(sys1.lanes["fps2"].unit, (2, 0))
        self.assertEqual(sys1.lanes["fps1"].op, S.OP_UNLOADED)   # untouched
        self.assertEqual(find(fx, S.StartLoad)[0].unit, (2, 0))

    def test_tick_isolation(self):
        # fps1 is mid-runout (PAUSING); fps2 is idle-LOADED and must be untouched.
        l1 = S.LaneState(op=S.OP_LOADED, group="T0", unit=(1, 0),
                         runout=S.RUNOUT_PAUSING, pause_origin=0.0)
        l2 = S.LaneState(op=S.OP_LOADED, group="T1", unit=(2, 0))
        w = self._world(
            fps1_kwargs={"extruder_pos": S.PAUSE_DISTANCE + 1, "printing": True,
                         "loaded": {(1, 0): False}, "ready": {(1, 3): True}},
            fps2_kwargs={"extruder_pos": 0.0, "printing": True,
                         "loaded": {(2, 0): True}})
        s, fx = S.reduce(self._system(l1, l2), S.Tick(), w, now=1.0)
        self.assertEqual(s.lanes["fps1"].runout, S.RUNOUT_COASTING)  # advanced
        self.assertEqual(s.lanes["fps2"], l2)                        # identical


if __name__ == "__main__":
    unittest.main(verbosity=2)
