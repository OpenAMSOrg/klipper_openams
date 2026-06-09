# OpenAMS — pure state machine (store core)
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This module is INTENTIONALLY PURE: it imports nothing from Klipper (no mcu, no
# reactor, no gcode) and performs no I/O. It defines the immutable state, the
# events (actions), the side effects (as data), and a pure reducer
#   reduce(system, action, world, now) -> (new_system, [effects])
# so the orchestration logic can be unit-tested with no hardware. The runtime
# (oams_runtime.py) is the only place that executes effects and touches Klipper.

from dataclasses import dataclass, field, replace
from typing import Mapping, Optional, Tuple

# --- firmware result codes (SUCCESS is 0 -> always compare, never test truthy) -
OAMS_OP_CODE_SUCCESS = 0
OAMS_OP_CODE_ERROR_UNSPECIFIED = 1
OAMS_OP_CODE_ERROR_BUSY = 2
OAMS_OP_CODE_SPOOL_ALREADY_IN_BAY = 3
OAMS_OP_CODE_NO_SPOOL_IN_BAY = 4
OAMS_OP_CODE_ERROR_KLIPPER_CALL = 5
OAMS_OP_CODE_CANCEL = 6

# --- follower directions (firmware convention) ---
FOLLOWER_REVERSE = 0
FOLLOWER_FORWARD = 1

# --- top-level operating states ---
OP_UNLOADED = "UNLOADED"
OP_LOADING = "LOADING"
OP_LOADED = "LOADED"
OP_UNLOADING = "UNLOADING"
OP_CALIBRATING = "CALIBRATING"
OP_PAUSED = "PAUSED"

# --- runout sub-state (only meaningful while OP_LOADED) ---
RUNOUT_IDLE = "idle"
RUNOUT_PAUSING = "pausing"      # feeding PAUSE_DISTANCE before coasting
RUNOUT_COASTING = "coasting"    # follower coasting, consuming the old tail
RUNOUT_LOADING = "loading"      # next-spool load in flight (non-blocking)

# --- tunables (decision constants; shared with the runtime) ---
PAUSE_DISTANCE = 60.0
FILAMENT_PATH_LENGTH_FACTOR = 1.14
OAMS_ACTION_TIMEOUT = 120.0
POLL_INTERVAL = 0.1
MONITOR_INTERVAL = 1.0


# ====================================================================== state

@dataclass(frozen=True)
class LaneState:
    """Immutable per-FPS-lane state. Transition with dataclasses.replace()."""
    op: str = OP_UNLOADED
    group: Optional[str] = None
    unit: Optional[Tuple[int, int]] = None      # (oams_idx, bay)
    following: bool = False
    direction: int = FOLLOWER_FORWARD
    # runout sub-machine
    runout: str = RUNOUT_IDLE
    pause_origin: Optional[float] = None
    coast_origin: Optional[float] = None
    reload_target: Optional[Tuple[int, int]] = None
    # async-op bookkeeping (one firmware op in flight per lane)
    op_deadline: Optional[float] = None
    prior_op: Optional[str] = None              # op to return to after CALIBRATING
    since: float = 0.0
    message: Optional[str] = None


@dataclass(frozen=True)
class SystemState:
    lanes: Mapping[str, LaneState] = field(default_factory=dict)  # fps_name -> LaneState


# ====================================================================== world

@dataclass(frozen=True)
class LaneWorld:
    """Read-only hardware snapshot the reducer is allowed to see, per lane."""
    extruder_pos: float = 0.0
    printing: bool = False
    loaded: Mapping[Tuple[int, int], bool] = field(default_factory=dict)  # hub HES
    ready: Mapping[Tuple[int, int], bool] = field(default_factory=dict)   # f1s HES
    group_bays: Mapping[str, Tuple[Tuple[int, int], ...]] = field(default_factory=dict)
    path_len: Mapping[int, float] = field(default_factory=dict)           # oams_idx -> clicks
    reload_before: float = 0.0


@dataclass(frozen=True)
class World:
    lanes: Mapping[str, LaneWorld] = field(default_factory=dict)

    def lane(self, fps):
        return self.lanes.get(fps, LaneWorld())


# ===================================================================== actions

@dataclass(frozen=True)
class Tick:
    now: float = 0.0

@dataclass(frozen=True)
class Load:
    fps: str = ""
    group: str = ""

@dataclass(frozen=True)
class Unload:
    fps: str = ""

@dataclass(frozen=True)
class Cancel:
    fps: str = ""

@dataclass(frozen=True)
class Calibrate:
    fps: str = ""
    oams_idx: int = 0
    bay: int = 0
    kind: str = "ptfe"          # "ptfe" | "hub_hes"

@dataclass(frozen=True)
class OpCompleted:
    fps: str = ""
    code: int = OAMS_OP_CODE_SUCCESS
    value: Optional[int] = None

@dataclass(frozen=True)
class Timeout:
    fps: str = ""

@dataclass(frozen=True)
class ClearErrors:
    pass


# ===================================================================== effects

@dataclass(frozen=True)
class OpResult:
    ok: bool
    code: Optional[int]
    message: str
    value: Optional[int] = None

@dataclass(frozen=True)
class StartLoad:
    unit: Tuple[int, int]

@dataclass(frozen=True)
class StartUnload:
    unit: Tuple[int, int]

@dataclass(frozen=True)
class StartCalibrate:
    unit: Tuple[int, int]
    kind: str

@dataclass(frozen=True)
class CancelLoad:
    unit: Tuple[int, int]

@dataclass(frozen=True)
class SetFollower:
    unit: Tuple[int, int]
    enable: int
    direction: int

@dataclass(frozen=True)
class Led:
    unit: Tuple[int, int]
    on: int

@dataclass(frozen=True)
class Pause:
    fps: str
    reason: str

@dataclass(frozen=True)
class ArmDeadline:
    fps: str
    seconds: float

@dataclass(frozen=True)
class CancelDeadline:
    fps: str

@dataclass(frozen=True)
class Settle:
    fps: str
    result: OpResult


# ===================================================================== reducer

def initial_system(fps_names):
    return SystemState(lanes={name: LaneState() for name in fps_names})


def reduce(system, action, world, now):
    """Pure top-level reducer. Returns (new_system, [effects])."""
    if isinstance(action, ClearErrors):
        lanes = {fps: _resync_lane(world.lane(fps), now)
                 for fps in system.lanes}
        return SystemState(lanes=lanes), []

    if isinstance(action, Tick):
        lanes = dict(system.lanes)
        effects = []
        for fps, lane in system.lanes.items():
            nl, fx = _reduce_lane(lane, action, world.lane(fps), now, fps)
            lanes[fps] = nl
            effects.extend(fx)
        return SystemState(lanes=lanes), effects

    # lane-scoped action
    fps = getattr(action, "fps", None)
    if fps is None or fps not in system.lanes:
        return system, []
    nl, fx = _reduce_lane(system.lanes[fps], action, world.lane(fps), now, fps)
    lanes = dict(system.lanes)
    lanes[fps] = nl
    return SystemState(lanes=lanes), fx


def _resync_lane(lw, now):
    """Recompute a lane purely from the hardware snapshot (ClearErrors/startup)."""
    for group, bays in lw.group_bays.items():
        for unit in bays:
            if lw.loaded.get(unit):
                return LaneState(op=OP_LOADED, group=group, unit=unit, since=now)
    return LaneState(op=OP_UNLOADED, since=now)


def _reduce_lane(lane, action, lw, now, fps):
    op = lane.op

    if isinstance(action, Load):
        if op != OP_UNLOADED:
            return lane, [Settle(fps, OpResult(False, None,
                          "lane busy (%s)" % op))]
        for unit in lw.group_bays.get(action.group, ()):
            if lw.ready.get(unit):
                nl = replace(lane, op=OP_LOADING, group=action.group, unit=unit,
                             runout=RUNOUT_IDLE, pause_origin=None,
                             coast_origin=None, reload_target=None,
                             op_deadline=now + OAMS_ACTION_TIMEOUT,
                             since=now, message=None)
                return nl, [StartLoad(unit), ArmDeadline(fps, OAMS_ACTION_TIMEOUT)]
        return lane, [Settle(fps, OpResult(False, None,
                      "no ready spool in group %s" % action.group))]

    if isinstance(action, Unload):
        if op != OP_LOADED or lane.unit is None:
            return lane, [Settle(fps, OpResult(False, None, "nothing loaded"))]
        nl = replace(lane, op=OP_UNLOADING, runout=RUNOUT_IDLE,
                     pause_origin=None, coast_origin=None, reload_target=None,
                     op_deadline=now + OAMS_ACTION_TIMEOUT, since=now)
        return nl, [StartUnload(lane.unit), ArmDeadline(fps, OAMS_ACTION_TIMEOUT)]

    if isinstance(action, Calibrate):
        if op in (OP_LOADING, OP_UNLOADING, OP_CALIBRATING):
            return lane, [Settle(fps, OpResult(False, None,
                          "busy, cannot calibrate now"))]
        unit = (action.oams_idx, action.bay)
        nl = replace(lane, op=OP_CALIBRATING, prior_op=op,
                     op_deadline=now + OAMS_ACTION_TIMEOUT, since=now)
        return nl, [StartCalibrate(unit, action.kind),
                    ArmDeadline(fps, OAMS_ACTION_TIMEOUT)]

    if isinstance(action, Cancel):
        if op == OP_LOADING and lane.unit is not None:
            return lane, [CancelLoad(lane.unit)]   # await OpCompleted(CANCEL)
        return lane, []

    if isinstance(action, OpCompleted):
        return _complete(lane, action.code, action.value, now, fps)

    if isinstance(action, Timeout):
        return _complete(lane, OAMS_OP_CODE_ERROR_UNSPECIFIED, None, now, fps,
                         timed_out=True)

    if isinstance(action, Tick):
        if op == OP_LOADED:
            return _runout_tick(lane, lw, now, fps)
        return lane, []

    return lane, []


def _complete(lane, code, value, now, fps, timed_out=False):
    op = lane.op
    ok = (code == OAMS_OP_CODE_SUCCESS)

    # Runout auto-reload completion (op stays LOADED while reloading).
    if op == OP_LOADED and lane.runout == RUNOUT_LOADING:
        if ok:
            nl = replace(lane, unit=lane.reload_target, runout=RUNOUT_IDLE,
                         pause_origin=None, coast_origin=None,
                         reload_target=None, op_deadline=None,
                         message="next spool loaded")
            return nl, [CancelDeadline(fps)]
        nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                     runout=RUNOUT_IDLE, pause_origin=None, coast_origin=None,
                     reload_target=None, op_deadline=None,
                     message="reload failed")
        reason = ("timed out loading next spool" if timed_out
                  else "failed to load next spool (code %s)" % (code,))
        return nl, [CancelDeadline(fps), Pause(fps, reason)]

    if op == OP_LOADING:
        if ok:
            nl = replace(lane, op=OP_LOADED, op_deadline=None,
                         message="loaded")
            return nl, [CancelDeadline(fps),
                        Settle(fps, OpResult(True, code, "Spool loaded successfully"))]
        if code == OAMS_OP_CODE_CANCEL:
            nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                         op_deadline=None, message="cancelled")
            return nl, [CancelDeadline(fps),
                        Settle(fps, OpResult(False, code, "Spool loading cancelled"))]
        nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                     op_deadline=None, message="load failed")
        msg = ("timed out loading spool" if timed_out
               else "Spool loading failed (code %s)" % (code,))
        return nl, [CancelDeadline(fps), Settle(fps, OpResult(False, code, msg))]

    if op == OP_UNLOADING:
        if ok:
            nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                         following=False, op_deadline=None, message="unloaded")
            return nl, [CancelDeadline(fps),
                        Settle(fps, OpResult(True, code, "Spool unloaded successfully"))]
        # Unload failed: still loaded; stop the follower so it can't keep rewinding.
        nl = replace(lane, op=OP_LOADED, following=False, op_deadline=None,
                     message="unload failed")
        msg = ("timed out unloading spool" if timed_out
               else "Spool unloading failed (code %s)" % (code,))
        effects = [CancelDeadline(fps)]
        if lane.unit is not None:
            effects.append(SetFollower(lane.unit, 0, FOLLOWER_REVERSE))
        effects.append(Settle(fps, OpResult(False, code, msg)))
        return nl, effects

    if op == OP_CALIBRATING:
        target = lane.prior_op or OP_UNLOADED
        nl = replace(lane, op=target, prior_op=None, op_deadline=None,
                     message=("calibrated" if ok else "calibration failed"))
        msg = ("Calibration complete" if ok
               else ("timed out calibrating" if timed_out
                     else "Calibration failed (code %s)" % (code,)))
        return nl, [CancelDeadline(fps),
                    Settle(fps, OpResult(ok, code, msg, value))]

    # Stray completion with nothing in flight — ignore.
    return lane, []


def _runout_tick(lane, lw, now, fps):
    r = lane.runout

    if r == RUNOUT_IDLE:
        if (lw.printing and lane.unit is not None
                and not lw.loaded.get(lane.unit, False)):
            return replace(lane, runout=RUNOUT_PAUSING,
                           pause_origin=lw.extruder_pos), []
        return lane, []

    # Later phases need a loaded unit; bail safely if it vanished.
    if lane.unit is None:
        return replace(lane, runout=RUNOUT_IDLE, pause_origin=None,
                       coast_origin=None, reload_target=None), []

    if r == RUNOUT_PAUSING:
        if lw.extruder_pos - lane.pause_origin >= PAUSE_DISTANCE:
            return (replace(lane, runout=RUNOUT_COASTING,
                            coast_origin=lw.extruder_pos),
                    [SetFollower(lane.unit, 0, FOLLOWER_FORWARD)])
        return lane, []

    if r == RUNOUT_COASTING:
        path_len = lw.path_len.get(lane.unit[0], 0.0)
        if path_len <= 0:
            nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                         runout=RUNOUT_IDLE, pause_origin=None,
                         coast_origin=None, message="ptfe_length uncalibrated")
            return nl, [Pause(fps, "ptfe_length is not calibrated (0); cannot"
                              " auto-load the next spool. Run"
                              " OAMS_CALIBRATE_PTFE_LENGTH.")]
        consumed = lw.extruder_pos - lane.coast_origin
        if consumed + PAUSE_DISTANCE + lw.reload_before > path_len / FILAMENT_PATH_LENGTH_FACTOR:
            for unit in lw.group_bays.get(lane.group, ()):
                if unit == lane.unit:
                    continue  # never reload the bay that just ran out
                if lw.ready.get(unit):
                    nl = replace(lane, runout=RUNOUT_LOADING, reload_target=unit,
                                 op_deadline=now + OAMS_ACTION_TIMEOUT)
                    return nl, [StartLoad(unit),
                                ArmDeadline(fps, OAMS_ACTION_TIMEOUT)]
            nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                         runout=RUNOUT_IDLE, pause_origin=None,
                         coast_origin=None, message="no spare spool")
            return nl, [Pause(fps, "filament runout on group %s and no spare"
                              " spool available" % lane.group)]
        return lane, []

    # RUNOUT_LOADING: waiting on OpCompleted/Timeout (handled in _complete).
    return lane, []
