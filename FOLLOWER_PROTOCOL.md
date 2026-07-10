# OpenAMS inline-follower firmware protocol (v1)

This document is the COMPLETE contract between the `klipper_openams` host
plugin (`src/follower.py`) and the follower MCU firmware. The firmware is a
custom Klipper MCU C module living in the Klipper fork
**github.com/jrlomas/klipper** (branch off `master`; the fork's
`rt-comparator` branch — `src/stm32/comp.c` + `klippy/extras/window_comparator.py`
— is the in-fork precedent for a custom module and its Makefile hook, but the
follower module must be portable C in `src/follower.c`, gpio/timer/sched APIs
only, Kconfig-gated for 32-bit MCUs). A firmware implementation session needs
this file and nothing else.

## 1. Design rationale — why not rotation_distance / motion-queue sync

Grounded in source (mainline klipper @ c707dd1; Happy-Hare @ current master):

- Mainline requires a FULL queue flush around every rotation-distance change:
  `SET_E_ROTATION_DISTANCE` calls `toolhead.flush_step_generation()` before
  `stepper.set_rotation_distance()` (`klippy/kinematics/extruder.py:121-123`);
  `flush_step_generation` = `_flush_lookahead()` + `flush_all_steps()`
  (`klippy/toolhead.py:317-319`) — the planner drains, lookahead cannot blend
  across the change, and all pending steps are compiled and posted. The
  setter itself (`klippy/stepper.py:138-143`) re-derives the step distance
  and rebases the kinematics: the queued motion math is invalidated and must
  be recomputed from the change point. Doing this at correction cadence means
  repeated planner stalls.
- Happy Hare's sync-feedback: a host-side controller updates on sensor-state
  EDGES and once per `sync_feedback_extrude_threshold` (default **5 mm of
  extrusion**) (`extras/mmu/mmu_sync_feedback_manager.py:480-512`); a
  SyncController (EKF or two-level branch, ±5%/±25% speed envelopes,
  `extras/mmu/mmu_sync_controller.py:191-193,1135-1217`) recomputes
  `rd_current`; actuation is `steppers[0].set_rotation_distance(rd)`
  (`extras/mmu/mmu.py:6483-6487`) — WITHOUT the flush mainline requires,
  trading the stall for silently rebasing position bookkeeping
  mid-generation. Structural limits either way: corrections only at event
  cadence (open-loop between events), rd-space quantization, host+transport
  latency inside the loop, and filament-path state coupled into the
  extruder's motion pipeline.
- This design instead NEVER touches the motion queue: extruder kinematics are
  constant, so lookahead/pressure-advance stay valid forever. The follower
  MCU closed-loops at ~1 kHz: host-streamed FEED-FORWARD (the motion queue is
  known ahead of real time — its knowledge is used without mutating it)
  carries the fast dynamics, and the FPS buffer-pressure signal continuously
  trims the residual. The controlled variable is PRESSURE, not inferred
  position: steps/mm error, slip and PTFE-length-dependent elasticity are
  absorbed by the buffer spring and regulated to a setpoint. The authority
  holding the motor is on the MCU, with watchdogs that stop it if the host
  dies.

## 2. Conventions

- **Integers only** on the wire (Klipper MCU C is float-free): distances in
  µsteps, velocities in steps/s, accelerations in steps/s², FPS values as
  u16 (host float 0..1 → 0..65535), PID gains as Q12 fixed point, times in
  ms. The HOST owns every mm↔steps conversion; `steps_per_mm` never crosses
  the wire.
- All commands are `oid`-scoped: several followers may share one MCU.
- Constants are published with `DECL_CONSTANT`; commands with `DECL_COMMAND`;
  responses via `sendf` FROM TASK CONTEXT ONLY (timers set flags +
  `sched_wake_task`; see §7).
- `gen` is a one-byte op generation the host stamps on every op command; the
  firmware ECHOES it verbatim in the op's terminal status. It is opaque to
  the firmware.

## 3. Dictionary constants

```
FOLLOWER_PROTOCOL_VERSION = 1        # bump on ANY contract change

# op result codes — SAME VALUES as the OAMS protocol so the host reducer
# enums apply unchanged:
FOLLOWER_OP_CODE_SUCCESS = 0
FOLLOWER_OP_CODE_ERROR_UNSPECIFIED = 1
FOLLOWER_OP_CODE_ERROR_BUSY = 2        # rejection: an op already in flight
FOLLOWER_OP_CODE_SPOOL_ALREADY_IN_BAY = 3   # load with POST already made
FOLLOWER_OP_CODE_NO_SPOOL_IN_BAY = 4        # load with PRE open / unload with POST open
FOLLOWER_OP_CODE_ERROR_KLIPPER_CALL = 5     # reserved, unused in v1
FOLLOWER_OP_CODE_CANCEL_LOAD_SPOOL = 6
FOLLOWER_OP_CODE_TIMEOUT = 7                # a firmware watchdog fired

# action ids for follower_action_status — same values as OAMS_STATUS_*:
FOLLOWER_STATUS_LOADING = 0
FOLLOWER_STATUS_UNLOADING = 1
FOLLOWER_STATUS_FORWARD_FOLLOWING = 2
FOLLOWER_STATUS_REVERSE_FOLLOWING = 3
FOLLOWER_STATUS_COASTING = 4         # never emitted (enum parity only)
FOLLOWER_STATUS_STOPPED = 5
FOLLOWER_STATUS_CALIBRATING = 6      # reserved (no calibrate op in v1)
FOLLOWER_STATUS_ERROR = 7

FOLLOWER_REVERSE = 0
FOLLOWER_FORWARD = 1
```

## 4. Config commands (received during MCU config phase)

```
config_follower oid=%c step_pin=%u dir_pin=%u enable_pin=%u flags=%c
    # flags: bit0 invert_step, bit1 invert_dir, bit2 invert_enable
config_follower_switches oid=%c pre_pin=%u pre_pullup=%c pre_invert=%c
    post_pin=%u post_pullup=%c post_invert=%c debounce_ms=%u
config_follower_tuning oid=%c kp=%u ki=%u kd=%u fps_target=%u
    fps_lower=%u fps_upper=%u fps_reversed=%c
    # kp/ki/kd: Q12, steps/s of trim per COUNT of fps16 error
    #   (ki per second of accumulated error, kd per second of error slope);
    # fps_*: u16 on the 0..65535 scale
config_follower_limits oid=%c max_v=%u accel=%u load_v=%u unload_v=%u
    # steps/s and steps/s^2. The firmware MUST hard-clamp every commanded
    # velocity to ±max_v and slew by accel, and MUST shutdown() at config
    # time if max_v exceeds its step-generation budget (config error, not a
    # runtime surprise).
config_follower_geometry oid=%c path_steps=%u switch_travel_steps=%u
    park_extra_steps=%u
config_follower_watchdog oid=%c fps_stale_ms=%u telemetry_ms=%u
```

The module OWNS step/dir/enable and the two switch pins directly (raw GPIO +
its own timer-driven step generation). It is deliberately NOT a Klipper
stepper object — no move queue, no `stepper_enable` registration — so there
is no motion-queue conflict. (Consequence on the host side: TMC drivers are
configured by the plugin over UART or run standalone; a plain `[tmc2209]`
section cannot attach.)

## 5. Runtime commands (host → firmware)

```
follower_cmd_load oid=%c gen=%c
follower_cmd_unload oid=%c gen=%c
follower_cmd_load_cancel oid=%c
follower_cmd_set oid=%c enable=%c direction=%c   # the follow-loop enable
follower_cmd_fps oid=%c value=%u                 # forwarded FPS sample, u16
follower_cmd_ff oid=%c clock=%u velocity=%i      # feed-forward segment
follower_cmd_clear_errors oid=%c                 # stop motor, clear error
                                                 # latch, flush the ff queue
```

Feed-forward semantics: from MCU clock `clock` (low 32 bits; wraparound
comparison is safe because segments are only scheduled ~1–2 s ahead), the
commanded extruder velocity is `velocity` steps/s (signed; retractions are
negative), piecewise-constant until the next segment. Keep a ring of ≥64
segments. The host sends ≤ ~20 segments/s with delta suppression and a v=0
keepalive (~1 Hz) when the print queue is idle.

## 6. Responses (firmware → host)

```
follower_action_status oid=%c action=%c code=%c value=%u gen=%c
    # EXACTLY ONE completion-class status per op, in op order, gen echoed.
    # value = µsteps moved during the op (diagnostics).
follower_stats oid=%c pre=%c post=%c flags=%c step_count=%i velocity=%i
    # periodic every telemetry_ms (default 500)
    # flags: bit0 following, bit1 direction, bit2 op_in_flight,
    #        bit3 fps_stale, bit4 ff_underrun, bit5 error_latched
    # step_count: signed cumulative steps; velocity: current steps/s signed
```

## 7. Semantic guarantees (all mandatory at v1)

1. **One terminal status per op.** Every `follower_cmd_load`/`_unload`
   produces exactly one `follower_action_status` with
   `action ∈ {LOADING, UNLOADING, ERROR}` and the op's echoed `gen` —
   SUCCESS, a failure code, CANCEL_LOAD_SPOOL, or TIMEOUT. Never zero, never
   two. `ERROR_BUSY` is the rejection of a second op while one is in flight
   and carries the REJECTED op's gen.
2. **Load sequence**: reject `NO_SPOOL_IN_BAY` if PRE open; reject
   `SPOOL_ALREADY_IN_BAY` if POST already made. Phase 1: feed at `load_v`
   until POST makes — TIMEOUT if not within `switch_travel_steps`. Phase 2:
   continue at `load_v`, slowing near the end, until the forwarded FPS
   reaches `fps_upper` (filament pressed into the extruder gears); total
   budget 1.2 × `path_steps` else TIMEOUT. On SUCCESS **auto-start forward
   following** (OAMS parity — the store expects a loaded lane to follow).
3. **Unload sequence**: reject `NO_SPOOL_IN_BAY` if POST open. Reverse at
   `unload_v` until POST clears (budget 1.2 × `path_steps` else TIMEOUT),
   then reverse `park_extra_steps` more and stop → SUCCESS. PRE should stay
   made (the bay remains "ready" as a runout spare); do not eject past PRE.
4. **Cancel**: `follower_cmd_load_cancel` mid-load stops the motor and
   completes that op with `CANCEL_LOAD_SPOOL` (gen echoed). A cancel with no
   load in flight is a silent no-op (no status).
5. **Follow loop** (`follower_cmd_set enable=1`):
   `v = slew(clamp(ff(t) + PID(fps_target − fps), ±max_v), accel)`.
   `direction=FORWARD` uses ff + trim as above. `direction=REVERSE` (used by
   unload-assist macros): reverse at `unload_v` while fps > `fps_lower`,
   stop below it (buffer emptying from the toolhead side raises FPS) — flag
   for hardware tuning. `enable=0` stops the motor and flushes the ff queue.
   Follow-state changes emit NO action_status (the enum values 2/3/5 exist
   for parity; the host drops them if ever sent).
6. **Watchdogs** (the firmware owns liveness; the host keeps only a coarse
   300 s disconnect backstop):
   - FPS staleness: while following or an op is in flight, if no
     `follower_cmd_fps` arrives within `fps_stale_ms`, ramp velocity to 0
     within ~100 ms and set the `fps_stale` flag. This is the HOST-DEATH
     protection. If staleness persists > 5 s during an op, abort the op with
     TIMEOUT. Idle followers never arm this watchdog.
   - No-progress: op distance budgets above; there is no encoder, so
     progress is switch/step-count based.
   - ff underrun: playback passing the last received segment sets
     `ff_underrun`, holds ff at 0 and continues on FPS trim alone
     (degraded, not fatal).
7. **Ops ignore ff segments** (they use their own speed profile), but FPS
   forwarding stays active during ops — load termination depends on it.
8. `DECL_SHUTDOWN` handler: stop the step timer and de-assert enable — the
   motor must be safe on any MCU shutdown.

## 8. Implementation blueprint (maps to proven in-tree patterns)

- `struct follower { struct timer step_timer; struct timer control_timer; … }`
  per oid (`oid_alloc`, like `src/stepper.c:222`).
- **Step generator**: self-rescheduling `step_timer` with
  `interval = timer_freq / |v_cmd|` (0 → idle; min interval = the step-rate
  budget). Same timer technique as `stepper_event` (`src/stepper.c:139-210`)
  minus the move queue — velocity is a variable, not queued moves. Maintain
  the signed `int32_t step_count` here.
- **Control tick** at 1 kHz: debounced switch sampling (pattern:
  `src/buttons.c:28-68`); ff ring playback selected by `timer_is_before`;
  Q12 PID with clamped integrator; velocity slew; op state machine advanced
  on switch edges + step budgets; watchdog counters.
- **Task context** (`DECL_TASK` + `sched_wake_task`) does every `sendf`
  (action_status, telemetry) — never from timer context (discipline of
  `src/buttons.c` / `src/adccmds.c` reporting).
- Budget: 1 kHz control + ≤50–100 k steps/s stepping is within what
  stepper.c sustains on 32-bit targets.

## 9. Host behavior the firmware may rely on

- The host feature-gates on `FOLLOWER_PROTOCOL_VERSION` (absent = config
  error host-side; there is no legacy mode) and validates the op-code enum
  values against its own.
- FPS samples arrive at ~10–30 Hz while following or an op is in flight
  (one immediately on follow-enable), never while idle.
- ff segments are pre-clamped to ±max_v host-side too (belt and braces),
  time-stamped with the follower MCU's own clock via Klipper's standard
  clock sync, and only ever scheduled inside the already-generated (flushed)
  motion window.
- The host treats `follower_stats` as the truth for switch state
  (world model: PRE = "ready", POST = "loaded"), `step_count` as its
  encoder mirror, and TIMEOUT as an ordinary op failure requiring NO cancel
  (the firmware already stopped).

## 10. Hardware validation checklist (Phase C)

1. `OAMSM_SELFTEST` shows the follower connected, `protocol=FOLLOWER v1`,
   live PRE/POST toggling, and warns on `path_length: 0`.
2. `FOLLOWER_LOAD_SPOOL` / `FOLLOWER_UNLOAD_SPOOL`: exactly one completion
   each; unload parks between the switches (PRE still made).
3. Load with PRE open → `NO_SPOOL_IN_BAY`; load with POST made →
   `SPOOL_ALREADY_IN_BAY`; double-load → second gets `ERROR_BUSY`.
4. Jam a load → firmware TIMEOUT, motor stopped, no host cancel, no double
   completion.
5. Steady print with follow enabled: `fps_stale`/`ff_underrun` clear, FPS
   holds near target, trim component small (ff carries the bulk).
6. **Host-death test**: kill klippy mid-follow → motor stops within
   `fps_stale_ms` + ramp.
7. Mixed group (`oams1-0, follower-0`): runout on the OAMS bay auto-reloads
   through the follower; runout with the follower as the runner-out pauses
   (or reloads onto an OAMS spare).
8. Tune PID defaults and the reverse-follow behavior (§7.5); bake results
   into `follower.py` defaults and this document.
