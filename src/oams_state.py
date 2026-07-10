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
# The full firmware enum is kept here even where the host never branches on a
# code, so "(code N)" in user-facing messages can be looked up.
OAMS_OP_CODE_SUCCESS = 0
OAMS_OP_CODE_ERROR_UNSPECIFIED = 1
OAMS_OP_CODE_ERROR_BUSY = 2
OAMS_OP_CODE_SPOOL_ALREADY_IN_BAY = 3
OAMS_OP_CODE_NO_SPOOL_IN_BAY = 4
OAMS_OP_CODE_ERROR_KLIPPER_CALL = 5
OAMS_OP_CODE_CANCEL = 6
OAMS_OP_CODE_TIMEOUT = 7


def describe_code(code):
    """Human-readable form of a firmware result code for user-facing
    messages. Verified against firmware 2.0.25: BUSY is op_begin's rejection
    of a concurrent op, ALREADY_IN_BAY is the load coroutine's occupied-hub
    rejection, NO_SPOOL_IN_BAY is unload-with-nothing-loaded, KLIPPER_CALL is
    a calibration aborted via emergency stop. TIMEOUT (protocol v3+) is the
    firmware's own no-progress watchdog: it has already stopped the motors."""
    if code == OAMS_OP_CODE_ERROR_BUSY:
        return "OAMS is busy with another operation"
    if code == OAMS_OP_CODE_SPOOL_ALREADY_IN_BAY:
        return "filament already detected in the hub"
    if code == OAMS_OP_CODE_NO_SPOOL_IN_BAY:
        return "no spool present in the bay"
    if code == OAMS_OP_CODE_ERROR_KLIPPER_CALL:
        return "stopped by klipper monitor"
    if code == OAMS_OP_CODE_TIMEOUT:
        return "no filament progress (jam, dead motor, or missing sensor)"
    if code == OAMS_OP_CODE_CANCEL:
        return "cancelled"
    return "code %s" % (code,)

# --- follower directions (firmware convention) ---
FOLLOWER_REVERSE = 0
FOLLOWER_FORWARD = 1

# --- top-level operating states ---
OP_UNLOADED = "UNLOADED"
OP_LOADING = "LOADING"
OP_LOADED = "LOADED"
OP_UNLOADING = "UNLOADING"
OP_CALIBRATING = "CALIBRATING"

# --- runout sub-state (only meaningful while OP_LOADED) ---
RUNOUT_IDLE = "idle"
RUNOUT_PAUSING = "pausing"      # feeding PAUSE_DISTANCE before coasting
RUNOUT_COASTING = "coasting"    # follower coasting, consuming the old tail
RUNOUT_LOADING = "loading"      # next-spool load in flight (non-blocking)

# --- tunables (decision constants; shared with the runtime) ---
PAUSE_DISTANCE = 60.0
# Authoritative per-op deadline used ONLY against firmware that makes no
# liveness promise (protocol < 3 / legacy). Protocol >= 3 firmware runs its own
# no-progress watchdog and always completes an op, so the host downgrades this
# to a coarse disconnect backstop (OAMS_DISCONNECT_BACKSTOP) that exists only to
# release a blocked GCode wait() if the MCU dies entirely.
OAMS_ACTION_TIMEOUT = 120.0
OAMS_DISCONNECT_BACKSTOP = 300.0
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
    # Monotonic per-lane op generation. Incremented every time a firmware op is
    # started; Start* effects carry it and the driver echoes it back in
    # OpCompleted, so a stale reply (late after a timeout, or from a different
    # OAMS unit on the same lane) cannot complete the wrong op.
    op_gen: int = 0
    prior_op: Optional[str] = None              # op to return to after CALIBRATING
    since: float = 0.0
    message: Optional[str] = None


@dataclass(frozen=True)
class SystemState:
    lanes: Mapping[str, LaneState] = field(default_factory=dict)  # fps_name -> LaneState
    # True once every bound OAMS reports protocol >= 3, i.e. the firmware owns
    # per-op liveness (its own no-progress watchdog). The host then uses only a
    # coarse disconnect backstop instead of an authoritative per-op deadline.
    fw_owns_liveness: bool = False


def set_liveness(system, owns):
    """Return system with the firmware-owns-liveness flag set (called once at
    ready, after every unit's protocol version is known)."""
    return replace(system, fw_owns_liveness=bool(owns))


# ====================================================================== world

@dataclass(frozen=True)
class LaneWorld:
    """Read-only hardware snapshot the reducer is allowed to see, per lane."""
    extruder_pos: float = 0.0
    printing: bool = False
    loaded: Mapping[Tuple[int, int], bool] = field(default_factory=dict)  # hub HES
    ready: Mapping[Tuple[int, int], bool] = field(default_factory=dict)   # f1s HES
    group_bays: Mapping[str, Tuple[Tuple[int, int], ...]] = field(default_factory=dict)
    path_len: Mapping[int, float] = field(default_factory=dict)           # unit idx -> mm
    # path_len is the filament path from the unit's 'loaded' sensor to the
    # extruder, in millimetres. Unit conversion (e.g. OAMS encoder clicks)
    # happens in the drivers, so the reducer has exactly one unit to reason in.
    reload_before: float = 0.0


@dataclass(frozen=True)
class World:
    lanes: Mapping[str, LaneWorld] = field(default_factory=dict)

    def lane(self, fps):
        return self.lanes.get(fps, LaneWorld())


# ===================================================================== actions

@dataclass(frozen=True)
class Tick:
    pass

@dataclass(frozen=True)
class Load:
    fps: str = ""
    group: str = ""

@dataclass(frozen=True)
class LoadBay:
    # Low-level: load a specific (oams_idx, bay) directly (OAMS_LOAD_SPOOL),
    # bypassing group selection. The group, if the bay belongs to one, is
    # resolved so runout protection still applies.
    fps: str = ""
    unit: Tuple[int, int] = (0, 0)

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
    # Echo of the op_gen the driver was started with. The reducer only accepts
    # a completion whose gen matches the lane's op_gen exactly: every op the
    # store starts carries an int gen, so gen=None (an unsolicited firmware
    # status from a unit with no op in flight) never completes anything.
    gen: Optional[int] = None

@dataclass(frozen=True)
class Timeout:
    fps: str = ""

@dataclass(frozen=True)
class Follow:
    # Enable/disable the follower on the lane's loaded unit, through the store
    # so LaneState.following/direction stay truthful.
    fps: str = ""
    enable: int = 0
    direction: int = FOLLOWER_FORWARD

@dataclass(frozen=True)
class ClearErrors:
    pass


# Actions whose reduction reads the hardware snapshot. The runtime only builds
# a World for these; everything else (completions, timeouts, follower toggles)
# is decided from LaneState alone. Keep this in sync with the reducer: a branch
# that starts reading `lw` for a new action type must be added here.
WORLD_ACTIONS = (Load, LoadBay, Tick, ClearErrors)


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
    fps: str = ""
    gen: int = 0

@dataclass(frozen=True)
class StartUnload:
    unit: Tuple[int, int]
    fps: str = ""
    gen: int = 0

@dataclass(frozen=True)
class StartCalibrate:
    unit: Tuple[int, int]
    kind: str
    fps: str = ""
    gen: int = 0

@dataclass(frozen=True)
class CancelLoad:
    unit: Tuple[int, int]

@dataclass(frozen=True)
class SetFollower:
    unit: Tuple[int, int]
    enable: int
    direction: int

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
    # Per-op deadline: authoritative for legacy firmware, a coarse disconnect
    # backstop once the firmware owns liveness (protocol >= 3).
    deadline = (OAMS_DISCONNECT_BACKSTOP if system.fw_owns_liveness
                else OAMS_ACTION_TIMEOUT)

    if isinstance(action, ClearErrors):
        lanes = {fps: _resync_lane(world.lane(fps), now)
                 for fps in system.lanes}
        return replace(system, lanes=lanes), []

    if isinstance(action, Tick):
        lanes = dict(system.lanes)
        effects = []
        for fps, lane in system.lanes.items():
            nl, fx = _reduce_lane(lane, action, world.lane(fps), now, fps,
                                  deadline)
            lanes[fps] = nl
            effects.extend(fx)
        return replace(system, lanes=lanes), effects

    # lane-scoped action
    fps = getattr(action, "fps", None)
    if fps is None or fps not in system.lanes:
        return system, []
    nl, fx = _reduce_lane(system.lanes[fps], action, world.lane(fps), now, fps,
                          deadline)
    lanes = dict(system.lanes)
    lanes[fps] = nl
    return replace(system, lanes=lanes), fx


def _resync_lane(lw, now):
    """Recompute a lane purely from the hardware snapshot (ClearErrors/startup)."""
    for group, bays in lw.group_bays.items():
        for unit in bays:
            if lw.loaded.get(unit):
                return LaneState(op=OP_LOADED, group=group, unit=unit, since=now)
    return LaneState(op=OP_UNLOADED, since=now)


def _group_of(lw, unit):
    """The group on this lane that contains `unit`, or None."""
    for group, bays in lw.group_bays.items():
        if unit in bays:
            return group
    return None


def _hub_occupied(lw):
    """True when any hub HES on the lane still sees filament."""
    return any(lw.loaded.values())


def _load_guard(lane, lw, fps):
    """Shared pre-load safety checks for Load and LoadBay. Returns the Settle
    effects rejecting the request, or None when loading may start."""
    if lane.op != OP_UNLOADED:
        return [Settle(fps, OpResult(False, None, "lane busy (%s)" % lane.op))]
    if _hub_occupied(lw):
        return [Settle(fps, OpResult(False, None,
                "filament still detected in a hub on this lane;"
                " unload it first"))]
    return None


def _begin_op(lane, now, fps, deadline, effect_for_gen, **fields):
    """Single chokepoint for starting a firmware op: bumps the lane's op_gen,
    stamps the deadline, and arms the runtime timer, so the gen/deadline
    invariant cannot be forgotten at an individual call site. `deadline` is the
    authoritative per-op timeout for legacy firmware, or a coarse disconnect
    backstop once the firmware owns liveness.

    op_gen is kept in 0..255 because the firmware echoes it back as a single
    byte (oams_cmd_*2 gen=%c / oams_action_status2 gen=%c). Wrap collisions
    would need 256 ops on one lane inside the op window, which is physically
    impossible, and ordering is still enforced by the FIFO/echo."""
    gen = (lane.op_gen + 1) & 0xFF
    nl = replace(lane, op_gen=gen, op_deadline=now + deadline,
                 since=now, **fields)
    return nl, [effect_for_gen(gen), ArmDeadline(fps, deadline)]


def _reduce_lane(lane, action, lw, now, fps, deadline):
    op = lane.op

    if isinstance(action, Load):
        rejected = _load_guard(lane, lw, fps)
        if rejected is not None:
            return lane, rejected
        for unit in lw.group_bays.get(action.group, ()):
            if lw.ready.get(unit):
                return _begin_op(
                    lane, now, fps, deadline,
                    lambda gen, unit=unit: StartLoad(unit, fps, gen),
                    op=OP_LOADING, group=action.group, unit=unit,
                    runout=RUNOUT_IDLE, pause_origin=None, coast_origin=None,
                    reload_target=None, message=None)
        return lane, [Settle(fps, OpResult(False, None,
                      "no ready spool in group %s" % action.group))]

    if isinstance(action, LoadBay):
        rejected = _load_guard(lane, lw, fps)
        if rejected is not None:
            return lane, rejected
        unit = action.unit
        return _begin_op(
            lane, now, fps, deadline, lambda gen: StartLoad(unit, fps, gen),
            op=OP_LOADING, group=_group_of(lw, unit), unit=unit,
            runout=RUNOUT_IDLE, pause_origin=None, coast_origin=None,
            reload_target=None, message=None)

    if isinstance(action, Unload):
        if op != OP_LOADED or lane.unit is None:
            return lane, [Settle(fps, OpResult(False, None, "nothing loaded"))]
        if lane.runout == RUNOUT_LOADING:
            # A runout auto-reload firmware op is in flight on this lane.
            # Starting another op would bump op_gen (orphaning the reload's
            # completion) and leave that unit feeding unmonitored.
            return lane, [Settle(fps, OpResult(False, None,
                          "lane is auto-loading the next spool after a runout;"
                          " wait for it to finish"))]
        unit = lane.unit
        return _begin_op(
            lane, now, fps, deadline, lambda gen: StartUnload(unit, fps, gen),
            op=OP_UNLOADING, runout=RUNOUT_IDLE, pause_origin=None,
            coast_origin=None, reload_target=None)

    if isinstance(action, Calibrate):
        if (op in (OP_LOADING, OP_UNLOADING, OP_CALIBRATING)
                or (op == OP_LOADED and lane.runout == RUNOUT_LOADING)):
            # RUNOUT_LOADING included: calibrating over an in-flight reload
            # would restore op=LOADED with a bumped gen and no deadline,
            # wedging the lane until CLEAR_ERRORS.
            return lane, [Settle(fps, OpResult(False, None,
                          "busy, cannot calibrate now"))]
        unit = (action.oams_idx, action.bay)
        return _begin_op(
            lane, now, fps, deadline,
            lambda gen: StartCalibrate(unit, action.kind, fps, gen),
            op=OP_CALIBRATING, prior_op=op)

    if isinstance(action, Cancel):
        if op == OP_LOADING and lane.unit is not None:
            return lane, [CancelLoad(lane.unit)]   # await OpCompleted(CANCEL)
        return lane, []

    if isinstance(action, Follow):
        if lane.unit is None:
            return lane, []
        nl = replace(lane, following=bool(action.enable),
                     direction=action.direction)
        return nl, [SetFollower(lane.unit, action.enable, action.direction)]

    if isinstance(action, OpCompleted):
        if action.gen != lane.op_gen:
            # Stale reply (late after a timeout), a reply from another OAMS
            # unit on this lane, or an unsolicited status (gen=None) — it does
            # not belong to the op in flight.
            return lane, []
        return _complete(lane, action.code, action.value, now, fps)

    if isinstance(action, Timeout):
        return _complete(lane, OAMS_OP_CODE_ERROR_UNSPECIFIED, None, now, fps,
                         timed_out=True)

    if isinstance(action, Tick):
        # Belt-and-braces deadline backing up the runtime's reactor timer in
        # case it was ever lost. For legacy firmware this enforces the 120 s
        # op timeout; once the firmware owns liveness op_deadline is the long
        # disconnect backstop, so this only fires if the MCU went silent.
        if (lane.op_deadline is not None and now > lane.op_deadline
                and (op in (OP_LOADING, OP_UNLOADING, OP_CALIBRATING)
                     or (op == OP_LOADED and lane.runout == RUNOUT_LOADING))):
            return _complete(lane, OAMS_OP_CODE_ERROR_UNSPECIFIED, None, now,
                             fps, timed_out=True)
        if op == OP_LOADED:
            return _runout_tick(lane, lw, now, fps, deadline)
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
                  else "failed to load next spool (%s)" % describe_code(code))
        effects = [CancelDeadline(fps)]
        if timed_out and lane.reload_target is not None:
            # The firmware op is still running; tell it to stop feeding.
            effects.append(CancelLoad(lane.reload_target))
        effects.append(Pause(fps, reason))
        return nl, effects

    if op == OP_LOADING:
        if ok:
            nl = replace(lane, op=OP_LOADED, op_deadline=None,
                         message="loaded")
            return nl, [CancelDeadline(fps),
                        Settle(fps, OpResult(True, code, "Spool loaded successfully"))]
        if code == OAMS_OP_CODE_CANCEL:
            nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                         following=False, op_deadline=None, message="cancelled")
            effects = [CancelDeadline(fps)]
            if lane.following and lane.unit is not None:
                # A follower enabled mid-load would otherwise outlive the lane's
                # knowledge of its unit and become unstoppable via the store.
                effects.append(SetFollower(lane.unit, 0, lane.direction))
            effects.append(Settle(fps, OpResult(False, code,
                                                "Spool loading cancelled")))
            return nl, effects
        nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                     following=False, op_deadline=None, message="load failed")
        msg = ("timed out loading spool" if timed_out
               else "Spool loading failed (%s)" % describe_code(code))
        effects = [CancelDeadline(fps)]
        if timed_out and lane.unit is not None:
            # The firmware op is still running; tell it to stop feeding rather
            # than abandoning it mid-load.
            effects.append(CancelLoad(lane.unit))
        if lane.following and lane.unit is not None:
            effects.append(SetFollower(lane.unit, 0, lane.direction))
        effects.append(Settle(fps, OpResult(False, code, msg)))
        return nl, effects

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
               else "Spool unloading failed (%s)" % describe_code(code))
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
                     else "Calibration failed (%s)" % describe_code(code)))
        return nl, [CancelDeadline(fps),
                    Settle(fps, OpResult(ok, code, msg, value))]

    # Stray completion with nothing in flight — ignore.
    return lane, []


def _runout_tick(lane, lw, now, fps, deadline):
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
                            coast_origin=lw.extruder_pos, following=False),
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
        if consumed + PAUSE_DISTANCE + lw.reload_before > path_len:
            for unit in lw.group_bays.get(lane.group, ()):
                if unit == lane.unit:
                    continue  # never reload the bay that just ran out
                if lw.ready.get(unit):
                    return _begin_op(
                        lane, now, fps, deadline,
                        lambda gen, unit=unit: StartLoad(unit, fps, gen),
                        runout=RUNOUT_LOADING, reload_target=unit)
            nl = replace(lane, op=OP_UNLOADED, group=None, unit=None,
                         runout=RUNOUT_IDLE, pause_origin=None,
                         coast_origin=None, message="no spare spool")
            return nl, [Pause(fps, "filament runout on group %s and no spare"
                              " spool available" % lane.group)]
        return lane, []

    # RUNOUT_LOADING: waiting on OpCompleted/Timeout (handled in _complete).
    return lane, []
