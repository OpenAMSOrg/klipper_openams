# OpenAMS — runtime (store + effect executor + completion/timeout plumbing)
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The runtime is the ONLY place that performs side effects and touches the
# reactor/gcode. It holds the immutable SystemState, runs the pure reducer from
# oams_state.py on every action, and executes the returned effects. All dispatch
# happens on the reactor thread (the OAMS driver marshals firmware replies there
# via register_async_callback), so the store needs no locks.

import logging
from collections import deque

from . import oams_state as S

# Stall detection (encoder-didn't-move) while a load/unload op is in flight.
STALL_SAMPLES = 2
MIN_ENCODER_DIFF = 1
STALL_AFTER = 2.0   # seconds after op start before sampling begins


class Runtime:
    def __init__(self, printer, fps_names, build_world, resolve_oam):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.build_world = build_world      # callable(now) -> S.World
        self.resolve_oam = resolve_oam      # callable(oams_idx) -> oam | None
        self.system = S.initial_system(fps_names)
        self._pending = {}                  # fps -> ReactorCompletion
        self._deadlines = {}                # fps -> reactor timer handle
        self._stall = {}                    # fps -> deque(maxlen=STALL_SAMPLES)

    # -------------------------------------------------------------- dispatch

    def get_state(self):
        return self.system

    def set_firmware_liveness(self, owns):
        """Record whether the firmware owns per-op liveness (protocol >= 3).
        When it does, the host stops running its own encoder-stall watchdog and
        downgrades its op deadline to a coarse disconnect backstop."""
        self.system = S.set_liveness(self.system, owns)

    def dispatch(self, action):
        prev = self.system
        try:
            now = self.reactor.monotonic()
            # Only build the hardware snapshot for actions that read it: a
            # transient world-build failure must not be able to kill the
            # reduction of an unrelated completion, and completions/timeouts
            # fire far more often than world-consuming actions.
            world = (self.build_world(now)
                     if isinstance(action, S.WORLD_ACTIONS) else S.World())
            self.system, effects = S.reduce(prev, action, world, now)
        except Exception:
            logging.exception("OAMS: reducer failed on %s", type(action).__name__)
            # Never leave a gcode waiter blocked on the op-starting dispatch it
            # is waiting for. Crashes on OTHER actions (Cancel, a completion,
            # a tick) must NOT fail an unrelated op in flight — its own
            # completion or deadline will settle it.
            if isinstance(action, (S.Load, S.LoadBay, S.Unload, S.Calibrate)):
                self._settle(action.fps, S.OpResult(False, None,
                             "internal error (see klippy.log)"))
            return self.system
        self._log_transitions(prev, self.system, action)
        for e in effects:
            try:
                self._apply(e)
            except Exception:
                logging.exception("OAMS: effect %s failed", type(e).__name__)
                self._effect_failed(e)
        return self.system

    def request(self, fps, action):
        """Start an op and return a ReactorCompletion the caller can wait() on.
        The completion is settled by a later OpCompleted/Timeout (or immediately
        if the reducer rejects the request)."""
        # A waiter normally holds the gcode mutex, so two concurrent requests on
        # one lane should be impossible — but never leak a previous waiter.
        self._settle(fps, S.OpResult(False, None, "superseded by a new request"))
        completion = self.reactor.completion()
        self._pending[fps] = completion
        self.dispatch(action)
        return completion

    def tick(self):
        """Called once per monitor interval from the manager's reactor timer."""
        self.dispatch(S.Tick())
        self._check_stalls()

    def _log_transitions(self, prev, cur, action):
        for fps, lane in cur.lanes.items():
            pl = prev.lanes.get(fps)
            if pl is None or pl.op != lane.op or pl.runout != lane.runout:
                logging.info("OAMS[%s] %s/%s --%s--> %s/%s%s", fps,
                             None if pl is None else pl.op,
                             None if pl is None else pl.runout,
                             type(action).__name__, lane.op, lane.runout,
                             "" if not lane.message else " (%s)" % lane.message)

    # --------------------------------------------------------------- effects

    def _apply(self, e):
        if isinstance(e, S.StartLoad):
            self._start_oam(e).start_load_spool(e.unit[1], gen=e.gen)
        elif isinstance(e, S.StartUnload):
            self._start_oam(e).start_unload_spool(gen=e.gen)
        elif isinstance(e, S.StartCalibrate):
            self._start_oam(e).start_calibrate(e.kind, e.unit[1], gen=e.gen)
        elif isinstance(e, S.CancelLoad):
            oam = self.resolve_oam(e.unit[0])
            if oam is None:
                logging.warning("OAMS: cannot cancel load, unit %s unknown",
                                e.unit[0])
            else:
                oam.load_spool_cancel()
        elif isinstance(e, S.SetFollower):
            oam = self.resolve_oam(e.unit[0])
            if oam is None:
                logging.warning("OAMS: cannot set follower, unit %s unknown",
                                e.unit[0])
            else:
                oam.set_oams_follower(e.enable, e.direction)
        elif isinstance(e, S.Pause):
            self._pause(e.reason)
        elif isinstance(e, S.ArmDeadline):
            self._arm_deadline(e.fps, e.seconds)
        elif isinstance(e, S.CancelDeadline):
            self._cancel_deadline(e.fps)
        elif isinstance(e, S.Settle):
            self._settle(e.fps, e.result)

    def _start_oam(self, e):
        """Resolve the OAMS unit a Start* effect targets, or raise so the
        failure is surfaced now instead of as a generic timeout 120 s later."""
        oam = self.resolve_oam(e.unit[0])
        if oam is None:
            raise RuntimeError("OAMS unit %s is not configured" % (e.unit[0],))
        return oam

    def _effect_failed(self, e):
        # A Start* effect that raised (unit missing, MCU not connected, send
        # failure) leaves the lane in an in-flight op state that nothing will
        # ever complete before the deadline. Fail it now — deferred one reactor
        # iteration so the current dispatch (including a trailing ArmDeadline)
        # finishes first.
        if not isinstance(e, (S.StartLoad, S.StartUnload, S.StartCalibrate)):
            return
        fps, gen = e.fps, e.gen

        def fail(eventtime):
            self.dispatch(S.OpCompleted(
                fps, S.OAMS_OP_CODE_ERROR_KLIPPER_CALL, gen=gen))

        self.reactor.register_async_callback(fail)

    def _pause(self, reason):
        try:
            gcode = self.printer.lookup_object("gcode")
            gcode.run_script("M118 OAMS: %s" % reason)
            gcode.run_script("M117 OAMS paused")
            gcode.run_script("PAUSE")
        except Exception:
            logging.exception("OAMS: failed to issue PAUSE")

    def _arm_deadline(self, fps, seconds):
        self._cancel_deadline(fps)

        def fire(eventtime, fps=fps):
            self.dispatch(S.Timeout(fps))
            return self.reactor.NEVER

        self._deadlines[fps] = self.reactor.register_timer(
            fire, self.reactor.monotonic() + seconds)

    def _cancel_deadline(self, fps):
        timer = self._deadlines.pop(fps, None)
        if timer is not None:
            self.reactor.unregister_timer(timer)

    def _settle(self, fps, result):
        completion = self._pending.pop(fps, None)
        if completion is not None and not completion.test():
            completion.complete(result)

    # ---------------------------------------------------------- stall detect

    def _check_stalls(self):
        # Protocol >= 3 firmware runs its own no-progress watchdog (and stops
        # the motors, completing the op with code TIMEOUT), so the host's
        # encoder-stall detection is redundant and disabled.
        if self.system.fw_owns_liveness:
            return
        now = self.reactor.monotonic()
        for fps, lane in list(self.system.lanes.items()):
            if lane.op not in (S.OP_LOADING, S.OP_UNLOADING) or lane.unit is None:
                self._stall.pop(fps, None)
                continue
            if now - lane.since <= STALL_AFTER:
                self._stall.pop(fps, None)   # reset window at op start
                continue
            oam = self.resolve_oam(lane.unit[0])
            if oam is None:
                continue
            samples = self._stall.setdefault(fps, deque(maxlen=STALL_SAMPLES))
            samples.append(oam.encoder_clicks)
            if len(samples) < STALL_SAMPLES:
                continue
            if abs(samples[-1] - samples[0]) < MIN_ENCODER_DIFF:
                logging.info("OAMS[%s]: %s stall detected", fps, lane.op)
                unit, op = lane.unit, lane.op
                self._stall.pop(fps, None)
                # Fail the op FIRST (settles any waiter via the unified timeout
                # path). The waiter may hold the gcode mutex that _pause's
                # run_script("PAUSE") needs, so pausing before settling would
                # block this tick until the op deadline rescued it.
                self.dispatch(S.Timeout(fps))
                oam.set_led_error(unit[1], 1)
                self._pause("%s speed too low" % op.lower())
