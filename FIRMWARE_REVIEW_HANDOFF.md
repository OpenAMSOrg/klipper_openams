# Session Handoff: spooling_bug_fixes hardening + mainboard-firmware review agenda

This file is a working handoff for continuing a review/fix session in a fresh
context. It captures (1) the state of this branch, (2) what was changed and the
reasoning, and (3) the host<->firmware contract questions that the paired
`OpenAMSOrg/mainboard-firmware` review (STM32F072RBT "ACE Board", C++) needs to
answer. Delete this file before merging.

---

## 1. Branch state

- Branch: `claude/review-spooling-bug-fixes-44lk61`
- Based on: `spooling_bug_fixes` @ `ee08e06` ("Route OAMS_LOAD_SPOOL /
  OAMS_UNLOAD_SPOOL through the store"), which itself is 4 commits ahead of
  `master` and rewrites the host modules into an event-driven state machine.
- Commits added by this session:
  - `6caa9a9` — "Harden state machine: op identity, fail-safe dispatch,
    multi-FPS follower stop" (fixes from a full review of spooling_bug_fixes)
  - `a671eab` — "Close generation-tracking gaps, fix macro-facing regressions
    from review" (fixes from a second adversarial review of 6caa9a9)
- Tests: 51, all passing. Run with:
  `cd klipper_openams && python3 -m unittest discover -s test`
  No pytest/Klipper needed. `test_oams_state.py` tests the pure reducer;
  `test_oams_runtime.py` tests the effect executor with a fake reactor (note
  its synthetic-package import trick for the relative imports).

## 2. Architecture on this branch (post-rewrite)

- `src/oams_state.py` — pure reducer, zero Klipper imports. Immutable
  `LaneState` per FPS lane; actions in, `(new_state, [effects])` out. Owns all
  decision logic: load/unload/calibrate lifecycle, runout auto-reload
  sub-machine (PAUSING -> COASTING -> LOADING), deadlines, op generations.
- `src/oams_runtime.py` — the ONLY place with side effects. Executes effects,
  arms/cancels deadline timers, settles `ReactorCompletion`s gcode handlers
  wait on, runs encoder-stall detection off the 1 s monitor tick.
- `src/oams.py` — per-unit MCU driver. Sends firmware commands, mirrors
  telemetry, converts `oams_action_status` replies into `OpCompleted` actions.
- `src/oams_manager.py` — Klipper adapter: config topology (FPS lanes, OAMS
  units, filament groups), gcode commands (`OAMSM_*`), webhooks, world builder.
- All dispatch happens on the reactor thread (serial-thread replies are
  marshalled via `register_async_callback`), so the store needs no locks.

### Op generations (the key mechanism added this session)

Firmware does not echo any correlation id, so the host stamps each op:
- `LaneState.op_gen` increments at every op start (single chokepoint:
  `_begin_op` in oams_state.py).
- `Start*` effects carry the gen; the driver queues it (`OAMS._gen_queue`,
  FIFO) and pairs each completion-class reply with the OLDEST pending gen.
- The reducer accepts an `OpCompleted` only when `gen == lane.op_gen` exactly;
  `gen=None` (unsolicited firmware status, empty FIFO) is always rejected.
- Known residual limitation: if a firmware reply is truly LOST, the FIFO
  pairs the next reply with the lost op's gen and drops it; that op then fails
  via its 120 s deadline instead of completing — safe but slow. Only a
  firmware-echoed correlation id can fully fix this (see agenda below).

## 3. What was fixed (condensed; see the two commit messages for full detail)

Commit `6caa9a9`:
- Spurious firmware statuses no longer treated as op completions; completions
  carry op generations (cross-unit/stale-reply protection).
- `Runtime.dispatch`/`request` can no longer leave a gcode waiter blocked
  forever (reducer exceptions settle the waiter; effect failures fail the op
  via a deferred `OpCompleted`; superseded waiters are settled).
- Stall detection settles the waiter BEFORE issuing PAUSE (gcode-mutex
  deadlock-by-ordering, rescued only by the 120 s deadline before).
- Load timeouts emit `CancelLoad` (firmware op no longer abandoned mid-feed);
  `Tick` enforces `op_deadline` as a backstop if the reactor timer is lost.
- Multi-FPS `OAMSM_UNLOAD_FILAMENT` with nothing loaded stops followers on all
  lanes; `_resolve_fps` raises on ambiguity instead of `respond_info`.
- Load refuses to start while any hub HES on the lane sees filament (restored
  master's `is_printer_loaded` semantics, now in the reducer via the world).
- Follower state made truthful: `Follow` action routes `OAMSM_FOLLOWER` /
  `OAMS_FOLLOWER` through the store; coasting clears `following`.
- fps.py Kalico support restored; extruder ref validated at connect; HDC1080
  reset-bit/escalation; external_irq per-MCU dispatcher with (port,pin)
  attribution; misc None-guards (`_require_cmd`) and dead-code removal.

Commit `a671eab` (issues found reviewing 6caa9a9 itself):
- gen=None no longer bypasses the gen check (it WAS the cross-unit hole).
- Driver gen FIFO replaces the single mutable `_op_gen` slot (late reply
  after timeout could be stamped with the retry's gen).
- World snapshot built only for `WORLD_ACTIONS` (Load, LoadBay, Tick,
  ClearErrors) — a world-build failure can no longer kill an unrelated
  completion, and completions stop paying O(units x bays) per dispatch.
- Settle-on-reducer-exception only for op-starting actions (a crash reducing
  webhook `Cancel` no longer fails an unrelated in-flight op).
- Followers enabled mid-load are stopped when the load fails/cancels;
  `_stop_followers()` helper keeps store + hardware in sync on all
  UNLOAD no-op paths; ambiguous multi-FPS unload stops store-known rewinding
  lanes before raising.
- Macro compat: `OAMSM_LOAD_FILAMENT_CANCEL` degrades to info (cleanup macros
  must not abort); `OAMSM_FOLLOWER ENABLE=0` broadcasts when ambiguous
  (stop is safe), `ENABLE=1` still raises; `OAMSM_LOAD_FILAMENT`/
  `OAMSM_UNLOAD_FILAMENT` raise on failure so toolchanges stop.
- fps.py feature-detects the MCU_adc API (`hasattr setup_adc_sample`) instead
  of trusting `use_kalico` (verified: mainline = setup_adc_sample + callback
  receiving a samples list; Kalico = setup_minmax + setup_adc_callback
  (report_time, cb) receiving scalars).
- HDC1080: after 6 consecutive failed read cycles, log + report 0.0 (so a
  user-configured min_temp decides) instead of unilateral invoke_shutdown.

### Deliberate non-changes (reviewed, kept, with reasons)

- `code == OAMS_OP_CODE_ERROR_KLIPPER_CALL` still accepted as a completion
  regardless of `action` — faithful to master's handler; needs firmware
  confirmation (agenda item 2).
- Dual timeout mechanism (per-op reactor timer + Tick backstop) kept as
  defense in depth; double-fire verified idempotent (CancelDeadline +
  stray-completion ignore).
- Reserved fps config options (`max_speed`, `accel`, `set_point`,
  `use_kalico`) still parsed so existing user configs don't fail Klipper's
  unused-option check.
- Unused firmware op-code constants kept as protocol documentation.

### Known open items (host side, small)

- `oams_sample.cfg` doesn't yet document that `use_kalico` is now ignored
  (auto-detected) — doc-only update.
- `_result_message` (oams.py) duplicates reducer message strings — cosmetic.
- Manager-level commands have no unit-test harness (reducer + runtime do).

## 4. Firmware session outcome (2026-06: build/linker work — protocol agenda still OPEN)

A session on `openamsorg/mainboard-firmware` produced branch
`claude/firmware-sysvar-review-tb2n6x` (commit `6783c2d`). Scope was
build/linker only — NO application source, protocol, or codegen changes, so
the Klipper command/response interface (generated `klipper_impl.hpp`) is
unchanged. App version 2.0.25, bootloader 1.4.0. **None of the contract
questions in section 5 below were investigated; they remain open.**

What changed (firmware side):
- Four build artifacts instead of three:
  - `kancan_1.4.0.bin` — bootloader only, flashed at 0x08000000.
  - `oams_2.0.25.bin` — app linked at offset 0x4000, runs behind the
    bootloader (the Katapult/CAN-update target).
  - `kancan_1.4.0_oams_2.0.25.bin` — combined image at 0x08000000.
  - NEW `oams_standalone_2.0.25.bin` — app linked at 0x08000000, no
    bootloader, flashed via DFU/ST-Link only. No metadata footer; CANNOT be
    updated over CAN/Katapult. Opt-in build: `pio run -e standalone`.
- Linker maps hardened so the app stack cannot clobber the
  reset-into-bootloader magic word at SRAM 0x20003FFC (more reliable
  handoff; no host-visible change).
- Memory map: bootloader active 0x08000000-0x08002000 (8KB), self-update
  staging 0x08002000-0x08004000, app 0x08004000-0x08020000. Standalone app
  occupies 0x08000000-0x08020000.
- NOT yet build- or hardware-verified (registry was network-blocked): the
  8KB bootloader link-time ASSERT and the standalone link location need a
  real `pio run`.

Host-side consequences applied on this branch:
- `scripts/flash_bootloader.py` now requires an explicit confirmation
  (skippable with `-y`) before writing: its staging (0x08002000) and commit
  (0x08000000) regions are application code on a standalone device, and the
  variant cannot be detected over CAN, so flashing a standalone board would
  brick it until SWD/DFU reflash.

Firmware follow-up status (second firmware session, branch now at `c666498`):
- F1 build verification: STILL BLOCKED — the remote env's network policy
  blocks api.registry.platformio.org, so `pio run` could not install the
  ststm32 platform. Needs a LOCAL `pio run` for all 4 envs (8KB bootloader
  link ASSERT, standalone link address, and the new standalone guard are all
  unbuilt). The standalone env now defines `-DOAMS_STANDALONE_BUILD=1`.
- F2 DONE: all five admin update handlers in src/bootloader_update.cpp begin
  with BL_REFUSE_IF_STANDALONE — on standalone builds they reply
  <cmd, ADMIN_RESP_ERROR=0x01> on 0x3f1 and touch no flash. Detection:
  `firmware_is_standalone()` = OAMS_STANDALONE_BUILD flag with a
  link-address fallback (&fn < APP_START_ADDR).
- F3 DONE: new admin command ADMIN_CMD_QUERY_VARIANT = 0x30
  (src/bootloader_update.cpp::process_admin_query_variant, dispatched in
  main.cpp). Read-only, safe on every build; only the addressed UUID
  replies. Request [0x30][UUID x6][reserved 0]; response
  [0x30][variant 0=bootloader-based 1=standalone][ver major][minor][patch]
  [app-offset high byte (0x40 => 0x4000, 0x00 => standalone)][reserved]
  [status 0x00=OK].
  HOST SIDE DONE on this branch: scripts/flash_bootloader.py probes 0x30
  before any flash-touching command, hard-refuses standalone targets, and
  falls back to the operator prompt only for firmware too old to answer
  (-y skips only that fallback, never a positive standalone report).

Useful firmware-source facts from that session:
- Active app source is the `_clang` variants (klipper_oams_clang.hpp +
  generated klipper_impl_clang.hpp); src/lark_impl/* is EXCLUDED from the
  build — don't review the wrong implementation.
- The firmware op state machine is already refactored to a single `g_op`
  with op_begin/op_finish chokepoints in src/sysvars.cpp — start there for
  the section 5 agenda.
- The section 5 agenda was ANSWERED (firmware 2.0.25 source, file:line
  evidence relayed 2026-06-12) — see verdicts inline in section 5 below.
  Net: no completion-attribution bugs firmware-side; the op_begin/op_finish
  refactor (src/sysvars.cpp) already guarantees the invariants the host's
  gen-FIFO depends on. T4 shipped no firmware changes; T2+T3 did.

## 5. Mainboard firmware protocol review agenda — ANSWERED (2026-06-12)

Verdicts from the firmware review (2.0.25, `_clang` implementation), keyed to
the questions below. Host-side consequences are marked [HOST].

1. CONFIRMED: exactly one completion per op on every path; op_finish is
   idempotent (sysvars.cpp:49, `if (!g_op.busy) return;`). The historical
   ERROR-then-CANCEL double is structurally prevented. Gen-FIFO assumption
   holds.
2. CONFIRMED SAFE: code 5 is attached in exactly one place
   (calibrate-hes cancel via emergency stop, main.cpp:283), always with
   action=CALIBRATING. Follower paths use BUSY/NO_SPOOL_IN_BAY, never 5.
   [HOST: the `or code==5` clause in oams.py _apply_action_status is
   redundant for 2.0.25 — kept, commented, for other firmware versions.]
3. CONFIRMED: no spontaneous status is op-terminating. Follower watchdog
   stop and follower_stop emit nothing; COASTING(4) is never emitted at all;
   actions 2/3/5 appear only as direct oams_cmd_follower responses (already
   dropped host-side).
4. CONFIRMED SAFE: load-cancel is a no-op (no status, no wedge) unless a
   load task is enabled. Minor gap: cancel during the ptfe-calibration
   unload phase is not honored — irrelevant today because the host only
   cancels loads (the reducer's Cancel requires OP_LOADING; calibrate
   timeouts emit no CancelLoad). Do not change that without firmware work.
5. CONFIRMED: firmware-side watchdog stops a REVERSE follower after 10 s
   without get_clock (host-death proxy; main.cpp:1028-1046). Forward
   following deliberately keeps running to avoid starving a print.
6. ASSESSED, NOT IMPLEMENTED: nonce protocol sketched (~30-40 LOC firmware,
   new oams_cmd_*2 / oams_action_status2, host feature-detects via
   lookup_command). DECISION PENDING — with 1/7 confirmed, the FIFO's only
   residual exposure is a genuinely lost CAN reply (fails by deadline,
   safe-but-slow), so the nonce is nice-to-have, not required.
7. CONFIRMED both interlocks: concurrent op -> op_begin replies BUSY(2)
   with the REQUESTED action (completion-class); occupied hub ->
   SPOOL_ALREADY_IN_BAY(3). [HOST: a BUSY rejection is consumed by the
   completion filter — correctly attributed because the store enforces one
   in-flight op per lane/unit; keep that invariant. describe_code() now
   renders codes 2/3/4 readably in user messages.]
8. CONFIRMED: encoder_clicks is a 32-bit software accumulator; 1 s diffs in
   the host stall detector are safe.
9. CONFIRMED: ptfe_length unit is encoder clicks end-to-end; 0 is safe
   (only gates the slow-down point; no div-by-zero). Host's /1.14 factor is
   host-side only.
10. CONFIRMED: stats every 450 ms, copied in one pass under a cooperative
    scheduler -> each oams_cmd_stats is an internally-coherent snapshot
    (host reads once per second).
11. MOSTLY SOLID: CRC-gated commit, page-ACK flow, out-of-order chunk
    rejection, retry-able copy failure. Residual: power loss during the
    active-region erase/copy is unrecoverable (no fallback bootloader).
    [HOST: flash_bootloader.py now prints a do-not-power-off warning at
    commit.]

Original agenda (kept for context; all items now answered above):

Repo: `OpenAMSOrg/mainboard-firmware` (private, C++, STM32F072RBT). It speaks
Klipper's MCU protocol over CAN. Review it against the host-side contract
below; every "verify" item changes host behavior if the answer is unexpected.

### Protocol surface the host uses (grep targets in firmware)

Commands (host -> firmware):
- `oams_cmd_load_spool spool=%c`
- `oams_cmd_unload_spool`
- `oams_cmd_load_spool_cancel` (optional; host degrades if missing)
- `oams_cmd_follower enable=%c direction=%c`
- `oams_cmd_calibrate_ptfe_length spool=%c`
- `oams_cmd_calibrate_hub_hes spool=%c`
- `oams_cmd_pid kp=%u ki=%u kd=%u target=%u` (floats bit-cast to u32)
- `oams_set_led_error idx=%c value=%c`
- `oams_cmd_query_spool` -> `oams_query_response_spool spool=%u`
- config: `config_oams_buffer`, `config_oams_f1s_hes`, `config_oams_hub_hes`,
  `config_oams_pid`, `config_oams_ptfe length=%u`, `config_oams_current_pid`,
  `config_oams_logger idx=%u`, `config_ext_irq oid=%d irq_port=%d irq_pin=%d`
Responses (firmware -> host):
- `oams_action_status action=%c code=%c value=%u`
- `oams_cmd_stats fps_value=%u hub_hes_value_0..3=%c f1s_hes_value_0..3=%c
  encoder_clicks=%u` (fps_value is a float bit-cast to u32)
- `oams_cmd_current_status current_value=%u`
- `ext_irq_trigger port=%c pin=%c`
Host-side enums (must match firmware):
- action: 0 LOADING, 1 UNLOADING, 2 FORWARD_FOLLOWING, 3 REVERSE_FOLLOWING,
  4 COASTING, 5 STOPPED, 6 CALIBRATING, 7 ERROR
- code: 0 SUCCESS, 1 ERROR_UNSPECIFIED, 2 BUSY, 3 SPOOL_ALREADY_IN_BAY,
  4 NO_SPOOL_IN_BAY, 5 ERROR_KLIPPER_CALL, 6 CANCEL
- follower direction: 0 reverse(rewind), 1 forward

### Contract questions to verify in firmware (priority order)

1. **One completion per op, in order.** The host's gen FIFO assumes each
   load/unload/calibrate produces EXACTLY ONE completion-class
   `oams_action_status` (action in {LOADING, UNLOADING, CALIBRATING, ERROR}
   or code==5), and that replies arrive in op order. Verify: can an op emit
   two (e.g. ERROR then CANCEL after `load_spool_cancel`, or a retry inside
   firmware)? Can it emit zero (error paths that bail without reporting)?
   Every zero/duplicate path breaks host attribution and should be fixed
   firmware-side or compensated on the host.
2. **Semantics of code 5 (ERROR_KLIPPER_CALL).** The host treats ANY status
   with code 5 as an op completion, regardless of action (inherited from
   master). Verify which actions firmware attaches code 5 to. If it can
   appear on follower/coast/stop notifications (actions 2-5) while a
   load/unload is in flight, that's a false completion on the host — then the
   host filter must be narrowed and/or firmware should reserve code 5 for op
   results.
3. **Spontaneous statuses.** Which `oams_action_status` messages does firmware
   emit unprompted (follower state changes, coast transitions, errors)? The
   host now drops actions 2-5 (and unknown) as non-completions — confirm
   nothing op-terminating is in that set.
4. **`oams_cmd_load_spool_cancel` edge cases.** Host now sends it on every
   load timeout and stall. Verify firmware behavior when cancel arrives:
   (a) mid-load (expected: stop motors, emit action=LOADING/ERROR code=6),
   (b) after the load already completed, (c) when no load ever ran. (b)/(c)
   must not emit a stray completion-class status (would consume the host's
   FIFO head) and must not wedge the firmware op state machine.
5. **Follower runaway protection.** What stops the follower firmware-side if
   the host dies while `enable=1 direction=reverse` (the rewind case)? Is
   there a watchdog tied to Klipper's connection/config state? The host
   review fixed several runaway paths, but host-side fixes cannot cover a
   dead host.
6. **Correlation id (firmware improvement proposal).** Adding an echo field,
   e.g. host sends `oams_cmd_load_spool spool=%c nonce=%c` and firmware
   replies `oams_action_status action=%c code=%c value=%u nonce=%c`, would
   eliminate the host's FIFO heuristic and its lost-reply limitation
   entirely. Backward-compatible variant: new message name, host feature-
   detects via `mcu.lookup_command` try/except (pattern already used for
   `load_spool_cancel`). Assess cost in firmware.
7. **Busy/occupied guards.** Does firmware reject `oams_cmd_load_spool` while
   an op is running or the hub HES sees filament (BUSY / SPOOL_ALREADY_IN_BAY)?
   The host re-added its own guard, but defense in depth matters here —
   verify the firmware's own interlocks and which code it returns.
8. **`encoder_clicks` width/wrap.** Host stall detection diffs successive
   `encoder_clicks` (%u). Verify counter width and wrap behavior; a 16-bit
   wrap inside a 1 s window would defeat `abs(a-b) < 1` only pathologically,
   but confirm.
9. **`config_oams_ptfe length=%u`.** Host sends `ptfe_length` (float config,
   may be 0 on fresh installs) as %u. Verify firmware handles 0 sanely and
   what unit it expects (the host's runout math divides by
   FILAMENT_PATH_LENGTH_FACTOR=1.14 and compares against extruder mm — these
   are 'clicks'; confirm the calibration round-trips through
   `oams_cmd_calibrate_ptfe_length`'s `value` reply in the same unit).
10. **Stats cadence & atomicity.** `oams_cmd_stats` publication rate (host
    polls nothing; it reads the latest mirror once per second for runout
    decisions) and whether the HES values in one message are sampled
    coherently.
11. **Bootloader/CAN admin channel.** `scripts/flash_bootloader.py` and
    `scripts/canbus_logger.py` use admin CAN IDs 0x3f0/0x3f1 — review the
    firmware-side bootloader handler for bricking risks (the klipper_openams
    commit 99beece "Improve bootloader flashing logic" touched the host side
    of this in the branch history).

### How to resume efficiently

1. New session on `mainboard-firmware` (or both repos if multi-repo scope is
   available). Read THIS file first (it's on the
   `claude/review-spooling-bug-fixes-44lk61` branch of klipper_openams, which
   is public).
2. Locate the firmware's command handlers (grep for the `oams_cmd_*` /
   `oams_action_status` strings above — Klipper firmware macros like
   `DECL_COMMAND`/`sendf` or this project's CAN equivalent).
3. Work the agenda top-down; items 1-5 are correctness of the EXISTING host
   branch, 6 is a proposed improvement, 7-11 are robustness checks.
4. Host-side consequences discovered during the firmware review should be
   applied to this branch (klipper_openams), keeping the 51-test suite green.
