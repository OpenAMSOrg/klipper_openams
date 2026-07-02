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
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, gen=0),
                            world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADED)
        self.assertEqual(lane_of(sys1).unit, BAY_A)
        self.assertTrue(find(fx, S.Settle)[0].result.ok)
        self.assertTrue(has(fx, S.CancelDeadline))

    def test_load_completion_error(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_ERROR_BUSY, gen=0),
                            world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertIsNone(lane_of(sys1).unit)
        self.assertFalse(find(fx, S.Settle)[0].result.ok)

    def test_cancel_then_completed_is_not_loaded(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.Cancel(FPS), world(), now=1.0)
        self.assertEqual(find(fx, S.CancelLoad)[0].unit, BAY_A)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADING)  # still loading until ack
        sys2, fx2 = S.reduce(sys1, S.OpCompleted(FPS, S.OAMS_OP_CODE_CANCEL, gen=0),
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
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, gen=0),
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
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, gen=0),
                            world(), now=0.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertIsNone(lane_of(sys1).unit)
        self.assertTrue(find(fx, S.Settle)[0].result.ok)

    def test_unload_failure_stops_follower_and_stays_loaded(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_ERROR_BUSY, gen=0),
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
        sys2, fx2 = S.reduce(sys1, S.OpCompleted(
            FPS, S.OAMS_OP_CODE_SUCCESS, value=1234, gen=lane_of(sys1).op_gen),
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

        # 4) reload completes -> unit switched, back to idle (the reload bumped
        # the lane's op generation, so the completion must carry it)
        s, fx = S.reduce(s, S.OpCompleted(
            FPS, S.OAMS_OP_CODE_SUCCESS, gen=lane_of(s).op_gen), w, now=4.0)
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
                         S.OpCompleted(FPS, S.OAMS_OP_CODE_ERROR_BUSY, gen=0),
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
        # Start from a stale in-flight state; resync trusts the hub HES only.
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A))
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


class OpGenerationTests(unittest.TestCase):
    def test_start_load_stamps_generation(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADED, op_gen=4))
        sys1, fx = S.reduce(sys0, S.Load(FPS, GROUP), world(ready={BAY_A: True}),
                            now=0.0)
        self.assertEqual(lane_of(sys1).op_gen, 5)
        self.assertEqual(find(fx, S.StartLoad)[0].gen, 5)

    def test_generation_wraps_at_one_byte(self):
        # The firmware echoes gen as a single byte, so op_gen must stay in
        # 0..255. From 255 the next op wraps to 0, and a completion echoing 0
        # must match it.
        sys0 = system(S.LaneState(op=S.OP_UNLOADED, op_gen=255))
        sys1, fx = S.reduce(sys0, S.Load(FPS, GROUP), world(ready={BAY_A: True}),
                            now=0.0)
        self.assertEqual(lane_of(sys1).op_gen, 0)
        self.assertEqual(find(fx, S.StartLoad)[0].gen, 0)
        sys2, fx2 = S.reduce(sys1, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS,
                                                 gen=0), world(), now=1.0)
        self.assertEqual(lane_of(sys2).op, S.OP_LOADED)

    def test_stale_generation_completion_is_ignored(self):
        # A reply from a previous (timed-out) op, or from another OAMS unit on
        # the lane, carries an old gen and must not complete the current op.
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP,
                                  op_gen=7))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS,
                                                gen=6), world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADING)   # still in flight
        self.assertEqual(fx, [])

    def test_matching_generation_completes(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP,
                                  op_gen=7))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS,
                                                gen=7), world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADED)

    def test_unknown_generation_is_rejected(self):
        # gen=None marks an unsolicited firmware status (e.g. from an idle
        # unit on the lane that never started an op); it must not complete
        # the op in flight.
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP,
                                  op_gen=7))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS),
                            world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADING)
        self.assertEqual(fx, [])


class TimeoutHardeningTests(unittest.TestCase):
    def test_load_timeout_cancels_firmware_op(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.Timeout(FPS), world(), now=999.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertEqual(find(fx, S.CancelLoad)[0].unit, BAY_A)

    def test_load_error_does_not_cancel(self):
        # A real firmware error reply means the op already ended on the MCU.
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_ERROR_BUSY, gen=0),
                            world(), now=1.0)
        self.assertFalse(has(fx, S.CancelLoad))

    def test_unload_timeout_stops_follower(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADING, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.Timeout(FPS), world(), now=999.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADED)
        sf = find(fx, S.SetFollower)
        self.assertTrue(sf and sf[0].enable == 0)

    def test_reload_timeout_cancels_firmware_op(self):
        lane = S.LaneState(op=S.OP_LOADED, group=GROUP, unit=BAY_A,
                           runout=S.RUNOUT_LOADING, reload_target=BAY_B)
        s, fx = S.reduce(system(lane), S.Timeout(FPS), world(), now=999.0)
        self.assertEqual(find(fx, S.CancelLoad)[0].unit, BAY_B)
        self.assertTrue(has(fx, S.Pause))

    def test_tick_enforces_missed_deadline(self):
        # Belt-and-braces: even if the runtime's deadline timer was lost, a
        # Tick past op_deadline must fail the op instead of wedging the lane.
        lane = S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP,
                           op_deadline=100.0)
        s, fx = S.reduce(system(lane), S.Tick(), world(), now=101.0)
        self.assertEqual(lane_of(s).op, S.OP_UNLOADED)
        self.assertFalse(find(fx, S.Settle)[0].result.ok)

    def test_tick_before_deadline_does_not_fail(self):
        lane = S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP,
                           op_deadline=100.0)
        s, fx = S.reduce(system(lane), S.Tick(), world(), now=99.0)
        self.assertEqual(lane_of(s).op, S.OP_LOADING)
        self.assertEqual(fx, [])


class HubOccupiedTests(unittest.TestCase):
    def test_load_rejected_when_hub_occupied(self):
        # Store says UNLOADED (e.g. after a timeout) but a hub HES still sees
        # filament: starting another load would collide with it.
        sys0 = system(S.LaneState(op=S.OP_UNLOADED))
        w = world(ready={BAY_B: True}, loaded={BAY_A: True})
        sys1, fx = S.reduce(sys0, S.Load(FPS, GROUP), w, now=0.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertFalse(has(fx, S.StartLoad))
        self.assertFalse(find(fx, S.Settle)[0].result.ok)

    def test_load_bay_rejected_when_hub_occupied(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADED))
        w = world(loaded={BAY_A: True})
        sys1, fx = S.reduce(sys0, S.LoadBay(FPS, BAY_B), w, now=0.0)
        self.assertFalse(has(fx, S.StartLoad))
        self.assertFalse(find(fx, S.Settle)[0].result.ok)


class FollowTests(unittest.TestCase):
    def test_follow_updates_state_and_drives_hardware(self):
        sys0 = system(S.LaneState(op=S.OP_LOADED, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.Follow(FPS, 1, S.FOLLOWER_REVERSE),
                            world(), now=0.0)
        self.assertTrue(lane_of(sys1).following)
        self.assertEqual(lane_of(sys1).direction, S.FOLLOWER_REVERSE)
        sf = find(fx, S.SetFollower)[0]
        self.assertEqual((sf.unit, sf.enable, sf.direction),
                         (BAY_A, 1, S.FOLLOWER_REVERSE))

    def test_follow_without_unit_is_noop(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADED))
        sys1, fx = S.reduce(sys0, S.Follow(FPS, 1, S.FOLLOWER_FORWARD),
                            world(), now=0.0)
        self.assertFalse(lane_of(sys1).following)
        self.assertEqual(fx, [])

    def test_load_failure_stops_follower_enabled_mid_load(self):
        # A follower enabled while a load is in flight must not outlive the
        # lane's knowledge of its unit when the load fails.
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP,
                                  following=True, direction=S.FOLLOWER_FORWARD))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_ERROR_BUSY,
                                                gen=0), world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertFalse(lane_of(sys1).following)
        sf = find(fx, S.SetFollower)
        self.assertTrue(sf and sf[0].enable == 0 and sf[0].unit == BAY_A)

    def test_load_cancel_stops_follower_enabled_mid_load(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP,
                                  following=True, direction=S.FOLLOWER_FORWARD))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_CANCEL,
                                                gen=0), world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertFalse(lane_of(sys1).following)
        sf = find(fx, S.SetFollower)
        self.assertTrue(sf and sf[0].enable == 0)

    def test_coasting_clears_following(self):
        lane = S.LaneState(op=S.OP_LOADED, group=GROUP, unit=BAY_A,
                           following=True, runout=S.RUNOUT_PAUSING,
                           pause_origin=0.0)
        w = world(extruder_pos=S.PAUSE_DISTANCE + 1, printing=True,
                  loaded={BAY_A: False}, ready={BAY_B: True})
        s, fx = S.reduce(system(lane), S.Tick(), w, now=1.0)
        self.assertEqual(lane_of(s).runout, S.RUNOUT_COASTING)
        self.assertFalse(lane_of(s).following)


class StrayCompletionTests(unittest.TestCase):
    def test_completion_with_nothing_in_flight_is_ignored(self):
        sys0 = system(S.LaneState(op=S.OP_LOADED, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, gen=0),
                            world(), now=1.0)
        self.assertEqual(lane_of(sys1), lane_of(sys0))
        self.assertEqual(fx, [])

    def test_cancel_when_not_loading_is_noop(self):
        sys0 = system(S.LaneState(op=S.OP_LOADED, unit=BAY_A, group=GROUP))
        sys1, fx = S.reduce(sys0, S.Cancel(FPS), world(), now=1.0)
        self.assertEqual(lane_of(sys1), lane_of(sys0))
        self.assertEqual(fx, [])


class FirmwareTimeoutTests(unittest.TestCase):
    # Protocol v3: the firmware's no-progress watchdog stops the motors and
    # completes the op with code TIMEOUT(7). The host treats it as an ordinary
    # failure and must NOT send a cancel (the hardware is already stopped).
    def test_load_timeout_is_failure_without_cancel(self):
        sys0 = system(S.LaneState(op=S.OP_LOADING, unit=BAY_A, group=GROUP,
                                  op_gen=3))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_TIMEOUT,
                                                gen=3), world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_UNLOADED)
        self.assertFalse(find(fx, S.Settle)[0].result.ok)
        self.assertFalse(has(fx, S.CancelLoad))
        self.assertIn("progress", find(fx, S.Settle)[0].result.message)

    def test_unload_timeout_stops_follower_without_cancel(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADING, unit=BAY_A, group=GROUP,
                                  op_gen=2))
        sys1, fx = S.reduce(sys0, S.OpCompleted(FPS, S.OAMS_OP_CODE_TIMEOUT,
                                                gen=2), world(), now=1.0)
        self.assertEqual(lane_of(sys1).op, S.OP_LOADED)
        sf = find(fx, S.SetFollower)
        self.assertTrue(sf and sf[0].enable == 0)
        self.assertFalse(has(fx, S.CancelLoad))

    def test_describe_timeout_is_human_readable(self):
        self.assertIn("progress", S.describe_code(S.OAMS_OP_CODE_TIMEOUT))

    def test_reload_timeout_code_pauses_without_cancel(self):
        # Firmware TIMEOUT(7) during a runout auto-reload: the firmware
        # already stopped, so pause but do NOT send a cancel.
        lane = S.LaneState(op=S.OP_LOADED, group=GROUP, unit=BAY_A,
                           runout=S.RUNOUT_LOADING, reload_target=BAY_B,
                           op_gen=4)
        s, fx = S.reduce(system(lane),
                         S.OpCompleted(FPS, S.OAMS_OP_CODE_TIMEOUT, gen=4),
                         world(), now=1.0)
        self.assertTrue(has(fx, S.Pause))
        self.assertFalse(has(fx, S.CancelLoad))
        self.assertEqual(lane_of(s).op, S.OP_UNLOADED)


class RunoutReloadGuardTests(unittest.TestCase):
    # While a runout auto-reload firmware op is in flight (op stays LOADED,
    # runout == RUNOUT_LOADING), starting another op would bump op_gen and
    # orphan the reload's completion — Unload/Calibrate must be rejected.
    def _reloading_lane(self):
        return S.LaneState(op=S.OP_LOADED, group=GROUP, unit=BAY_A,
                           runout=S.RUNOUT_LOADING, reload_target=BAY_B,
                           op_gen=4)

    def test_unload_rejected_during_reload(self):
        s, fx = S.reduce(system(self._reloading_lane()), S.Unload(FPS),
                         world(), now=1.0)
        self.assertEqual(lane_of(s).op, S.OP_LOADED)
        self.assertEqual(lane_of(s).runout, S.RUNOUT_LOADING)  # untouched
        self.assertEqual(lane_of(s).op_gen, 4)                 # not bumped
        self.assertFalse(has(fx, S.StartUnload))
        self.assertFalse(find(fx, S.Settle)[0].result.ok)

    def test_calibrate_rejected_during_reload(self):
        s, fx = S.reduce(system(self._reloading_lane()),
                         S.Calibrate(FPS, oams_idx=1, bay=2, kind="ptfe"),
                         world(), now=1.0)
        self.assertEqual(lane_of(s).op, S.OP_LOADED)           # not CALIBRATING
        self.assertEqual(lane_of(s).op_gen, 4)
        self.assertFalse(has(fx, S.StartCalibrate))
        # ...and the reload can still complete normally afterwards
        s2, fx2 = S.reduce(s, S.OpCompleted(FPS, S.OAMS_OP_CODE_SUCCESS, gen=4),
                           world(), now=2.0)
        self.assertEqual(lane_of(s2).unit, BAY_B)
        self.assertEqual(lane_of(s2).runout, S.RUNOUT_IDLE)


class LivenessTests(unittest.TestCase):
    def test_coarse_deadline_when_firmware_owns_liveness(self):
        sys0 = S.set_liveness(system(S.LaneState(op=S.OP_UNLOADED)), True)
        sys1, fx = S.reduce(sys0, S.Load(FPS, GROUP), world(ready={BAY_A: True}),
                            now=100.0)
        self.assertEqual(lane_of(sys1).op_deadline,
                         100.0 + S.OAMS_DISCONNECT_BACKSTOP)
        self.assertEqual(find(fx, S.ArmDeadline)[0].seconds,
                         S.OAMS_DISCONNECT_BACKSTOP)

    def test_authoritative_deadline_for_legacy_firmware(self):
        sys0 = system(S.LaneState(op=S.OP_UNLOADED))   # fw_owns_liveness=False
        sys1, fx = S.reduce(sys0, S.Load(FPS, GROUP), world(ready={BAY_A: True}),
                            now=100.0)
        self.assertEqual(find(fx, S.ArmDeadline)[0].seconds,
                         S.OAMS_ACTION_TIMEOUT)

    def test_liveness_flag_survives_every_reduce_path(self):
        sys0 = S.set_liveness(
            system(S.LaneState(op=S.OP_LOADED, unit=BAY_A, group=GROUP)), True)
        sys1, _ = S.reduce(sys0, S.Tick(), world(loaded={BAY_A: True}), now=1.0)
        self.assertTrue(sys1.fw_owns_liveness)
        sys2, _ = S.reduce(sys1, S.Follow(FPS, 1, S.FOLLOWER_FORWARD),
                           world(), now=2.0)
        self.assertTrue(sys2.fw_owns_liveness)
        sys3, _ = S.reduce(sys2, S.ClearErrors(), world(loaded={BAY_A: True}),
                           now=3.0)
        self.assertTrue(sys3.fw_owns_liveness)


if __name__ == "__main__":
    unittest.main(verbosity=2)
