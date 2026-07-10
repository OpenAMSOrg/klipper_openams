# OpenAMS Manager — the system adapter
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The manager is the thin Klipper-facing layer of the system. It builds the FPS
# lane topology from config, owns the runtime (the store + effect executor),
# exposes the OAMSM_* gcode commands and webhooks, and drives the monitor tick.
# All decision logic lives in the pure reducer (oams_state.py); all side effects
# run in the runtime (oams_runtime.py).

import logging
import os

from . import oams_state as S
from . import oams_topology as T
from . import oams_config_io
from .oams_runtime import Runtime


class OAMSManager:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.reload_before = config.getfloat("reload_before_toolhead_distance", 0.0)
        # How often the runout/health monitor runs. Defaulted so it need not
        # appear in the config; lower = snappier runout reaction, more CPU.
        self.monitor_interval = config.getfloat(
            "monitor_interval", S.MONITOR_INTERVAL, above=0.0)
        # File that runtime writeback (group edits, calibration results) is
        # written to — the OpenAMS config holding the [oams ...] and
        # [filament_group ...] sections. Defaults to oams.cfg next to the main
        # printer config; SAVE_CONFIG can't reach an included subfile, so the
        # plugin edits this file in place.
        self.openams_config_path = os.path.expanduser(config.get(
            "openams_config_path", self._default_config_path()))

        self._build_topology()

        self.idle_timeout = None
        self.monitor_timer = None
        self.ready = False

        # The store + effect executor. Each OAMS is bound so its firmware replies
        # become OpCompleted actions on its lane.
        self.runtime = Runtime(self.printer, list(self.fpss.keys()),
                               self.build_world, self.oam_by_idx.get)
        for oam in self.oams.values():
            oam.bind_runtime(self.runtime, oam.fps_name)

        webhooks = self.printer.lookup_object("webhooks")
        webhooks.register_endpoint("openams/status", self._webhook_status)
        webhooks.register_endpoint("openams/cancel_load", self._webhook_cancel_load)
        webhooks.register_endpoint("openams/topology", self._webhook_topology)

        self.register_commands()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    # --------------------------------------------------------------- topology

    def _default_config_path(self):
        """oams.cfg next to the main printer config, derived from Klipper's
        start args so it works regardless of the user's config directory."""
        try:
            main = self.printer.get_start_args().get("config_file")
        except Exception:
            main = None
        if main:
            return os.path.join(os.path.dirname(main), "oams.cfg")
        return "~/printer_data/config/oams.cfg"

    def _force_load_sections(self):
        """Instantiate every fps / oams / filament_group section up front so
        topology validation does not depend on Klipper's config load order.
        load_object is idempotent (returns the already-loaded object), and
        filament_group no longer cross-references OAMS at load time, so the
        order we load these in does not matter."""
        for section in self.config.get_prefix_sections(''):
            name = section.get_name()
            if (name == "fps" or name.startswith("fps ")
                    or name.startswith("oams ")
                    or name.startswith("follower ")
                    or name == "filament_group"
                    or name.startswith("filament_group ")):
                self.printer.load_object(self.config, name)

    def _build_topology(self):
        self._force_load_sections()

        # Klipper object maps (objects can't live in the pure model). self.oams
        # is really "units by short name": OAMS mainboards AND inline followers
        # present the same driver interface, so everything downstream (idx
        # maps, world builder, effects) treats them uniformly.
        self.fpss = {fps.fps_name: fps
                     for _name, fps in self.printer.lookup_objects(module="fps")}
        self.oams = {full.split()[-1]: oam
                     for full, oam in self.printer.lookup_objects(module="oams")}
        followers = {full.split()[-1]: f
                     for full, f in self.printer.lookup_objects(
                         module="follower")}
        self._groups = {full.split()[-1]: g
                        for full, g in self.printer.lookup_objects(
                            module="filament_group")}

        # Validate all relationships in the pure model (raises with a clear,
        # user-facing message that we surface as a Klipper config error).
        oams_specs = [T.OamsSpec(name=short, idx=oam.oams_idx, fps=oam.fps_name)
                      for short, oam in self.oams.items()]
        oams_specs += [T.OamsSpec(name=short, idx=f.oams_idx, fps=f.fps_name,
                                  kind="follower")
                       for short, f in followers.items()]
        self.oams.update(followers)
        group_specs = [(short, list(g.bay_specs))
                       for short, g in self._groups.items()]
        try:
            self.topo = T.build_topology(list(self.fpss.keys()),
                                         oams_specs, group_specs)
        except T.TopologyError as e:
            raise self.config.error("OpenAMS configuration error: %s" % (e,))

        # Tell each OAMS its resolved lane (used by bind_runtime and requests).
        for short, oam in self.oams.items():
            oam.fps_name = self.topo.lane_of_oams(short)
        self._rebuild_derived()

    def _rebuild_derived(self):
        """(Re)compute the object-keyed maps the runtime/world/handlers use from
        the current topology. Called at build and after every runtime edit."""
        self.oam_by_idx = {oam.oams_idx: oam for oam in self.oams.values()}
        self.lane_oams = {lane: [self.oams[n] for n in self.topo.oams_on_lane(lane)]
                          for lane in self.topo.fps_names}
        self.group_lane = {}
        self.groups_by_lane = {lane: {} for lane in self.topo.fps_names}
        for gname in self.topo.groups:
            lane = self.topo.lane_of_group(gname)
            if lane is None:
                continue  # empty group (created at runtime, not yet populated)
            self.group_lane[gname] = lane
            self.groups_by_lane[lane][gname] = self.topo.group_bays_idx(gname)

    # ------------------------------------------------------------- world build

    def _is_printing(self, now):
        if self.idle_timeout is None:
            return False
        return self.idle_timeout.get_status(now)["state"] == "Printing"

    def build_world(self, now):
        printing = self._is_printing(now)
        lanes = {}
        for fps_name, fps in self.fpss.items():
            loaded, ready, path_len = {}, {}, {}
            for oam in self.lane_oams[fps_name]:
                for bay in range(4):
                    key = (oam.oams_idx, bay)
                    loaded[key] = bool(oam.hub_hes_value[bay])
                    ready[key] = bool(oam.f1s_hes_value[bay])
                path_len[oam.oams_idx] = oam.path_length_mm
            epos = fps.extruder.last_position if fps.extruder is not None else 0.0
            lanes[fps_name] = S.LaneWorld(
                extruder_pos=epos, printing=printing, loaded=loaded, ready=ready,
                group_bays=self.groups_by_lane.get(fps_name, {}),
                path_len=path_len, reload_before=self.reload_before)
        return S.World(lanes=lanes)

    # ----------------------------------------------------------------- ready

    def handle_ready(self):
        self.idle_timeout = self.printer.lookup_object("idle_timeout")
        # The firmware owns per-op liveness only if EVERY bound unit guarantees
        # it (protocol >= 3); otherwise the host keeps its own op deadline as
        # the conservative choice. Versions are resolved at connect, so this is
        # known by ready.
        owns = bool(self.oams) and all(
            oam.firmware_owns_liveness for oam in self.oams.values())
        self.runtime.set_firmware_liveness(owns)
        logging.info("OAMS: firmware owns per-op liveness: %s", owns)
        # Resync every lane from the hub HES hardware (the source of truth).
        self.runtime.dispatch(S.ClearErrors())
        self.start_monitor()
        self.ready = True

    def start_monitor(self):
        self.stop_monitor()
        self.monitor_timer = self.reactor.register_timer(
            self._monitor, self.reactor.NOW)
        logging.info("OAMS: monitor started")

    def stop_monitor(self):
        if self.monitor_timer is not None:
            self.reactor.unregister_timer(self.monitor_timer)
            self.monitor_timer = None

    def _monitor(self, eventtime):
        try:
            self.runtime.tick()
        except Exception:
            logging.exception("OAMS: monitor tick failed")
        return eventtime + self.monitor_interval

    # --------------------------------------------------------------- status

    def _lane_status(self, lane):
        return {"op": lane.op, "group": lane.group, "unit": lane.unit,
                "runout": lane.runout, "following": lane.following,
                "since": lane.since, "message": lane.message}

    def get_status(self, eventtime):
        state = self.runtime.get_state()
        lanes = {fps: self._lane_status(lane) for fps, lane in state.lanes.items()}
        # Back-compat: current_group is the group on the first loaded lane.
        current_group = None
        for lane in state.lanes.values():
            if lane.op == S.OP_LOADED and lane.group:
                current_group = lane.group
                break
        return {"current_group": current_group, "lanes": lanes}

    def _webhook_status(self, request):
        state = self.runtime.get_state()
        status = {"ready": self.ready, "units": len(self.oams),
                  "fps": {}, "lanes": {}, "filament_groups": {}}
        for fps_name, fps in self.fpss.items():
            status["fps"][fps_name] = {"value": fps.get_value(),
                                       "extruder": fps.extruder_name}
        for fps_name, lane in state.lanes.items():
            status["lanes"][fps_name] = self._lane_status(lane)
        for gname in self.topo.groups:
            status["filament_groups"][gname] = {
                "lane": self.group_lane.get(gname),
                "bays": ["%s-%d" % (oams_name, bay)
                         for (oams_name, bay) in self.topo.groups[gname]]}
        request.send({"status": {"openams": status}})

    def _webhook_topology(self, request):
        # Read-only model for a UI to render lanes/OAMS/groups and build edits.
        t = self.topo
        request.send({"topology": {
            "fps": list(t.fps_names),
            "oams": {n: {"idx": t.idx_of(n), "lane": t.lane_of_oams(n),
                         "kind": t.kind_of(n), "bays": t.bays_of(n)}
                     for n in t.oams},
            "groups": {g: {"lane": t.lane_of_group(g),
                           "bays": ["%s-%d" % e for e in t.groups[g]]}
                       for g in t.groups},
        }})

    def _webhook_cancel_load(self, request):
        state = self.runtime.get_state()
        loading = [fps for fps, lane in state.lanes.items()
                   if lane.op == S.OP_LOADING]
        if loading:
            for fps in loading:
                self.runtime.dispatch(S.Cancel(fps))
            request.send({"result": "cancel requested"})
        else:
            request.send({"result": "no load in progress"})

    # --------------------------------------------------------------- commands

    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        for name, handler, desc in (
            ("OAMSM_UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT,
             self.cmd_UNLOAD_FILAMENT_help),
            ("OAMSM_LOAD_FILAMENT", self.cmd_LOAD_FILAMENT,
             self.cmd_LOAD_FILAMENT_help),
            ("OAMSM_FOLLOWER", self.cmd_FOLLOWER, self.cmd_FOLLOWER_help),
            ("OAMSM_CURRENT_LOADED_GROUP", self.cmd_CURRENT_LOADED_GROUP,
             self.cmd_CURRENT_LOADED_GROUP_help),
            ("OAMSM_CLEAR_ERRORS", self.cmd_CLEAR_ERRORS,
             self.cmd_CLEAR_ERRORS_help),
            ("OAMSM_LOAD_FILAMENT_CANCEL", self.cmd_LOAD_FILAMENT_CANCEL,
             self.cmd_LOAD_FILAMENT_CANCEL_help),
            ("OAMSM_CREATE_GROUP", self.cmd_CREATE_GROUP,
             self.cmd_CREATE_GROUP_help),
            ("OAMSM_DELETE_GROUP", self.cmd_DELETE_GROUP,
             self.cmd_DELETE_GROUP_help),
            ("OAMSM_ASSIGN_BAY", self.cmd_ASSIGN_BAY, self.cmd_ASSIGN_BAY_help),
            ("OAMSM_UNASSIGN_BAY", self.cmd_UNASSIGN_BAY,
             self.cmd_UNASSIGN_BAY_help),
            ("OAMSM_SELFTEST", self.cmd_SELFTEST, self.cmd_SELFTEST_help),
        ):
            gcode.register_command(name, handler, desc=desc)

    cmd_SELFTEST_help = "Report OpenAMS wiring/health (read-only diagnostics)"

    def cmd_SELFTEST(self, gcmd):
        """Non-destructive bring-up check: confirms every OAMS connected, the
        negotiated protocol per unit, sensor readings, the validated topology
        and live lane state, and flags anything suspicious. Moves no filament."""
        lines = ["OpenAMS self-test:"]
        warns = []
        for name, fps in self.fpss.items():
            lines.append("  FPS %s: value=%.3f extruder=%s"
                         % (name, fps.get_value(),
                            getattr(fps, "extruder_name", "?")))
            if getattr(fps, "extruder", None) is None:
                warns.append("FPS %s extruder unresolved" % name)
        oams_versions = set()
        for short, oam in sorted(self.oams.items()):
            kind = self.topo.kind_of(short)
            bays = self.topo.bays_of(short)
            connected = oam.connected
            ready = [b for b in range(bays) if oam.f1s_hes_value[b]]
            loaded = [b for b in range(bays) if oam.hub_hes_value[b]]
            lines.append(
                "  %s %s (idx %s) lane %s: %s, %s"
                % (kind.upper(), short, oam.oams_idx, oam.fps_name,
                   "connected" if connected else "NOT CONNECTED",
                   oam.protocol_summary()))
            if kind == "follower":
                lines.append("    pre(ready)=%s post(loaded)=%s"
                             " path_length=%.1fmm fps_stale=%s ff_underrun=%s"
                             % (oam.f1s_hes_value[0], oam.hub_hes_value[0],
                                oam.path_length_mm, oam.fps_stale,
                                oam.ff_underrun))
                if oam.path_length_mm <= 0:
                    warns.append("follower %s path_length is 0 (runout will"
                                 " pause instead of auto-reloading)" % short)
            else:
                oams_versions.add(oam.protocol_version)
                lines.append("    bays ready=%s loaded=%s"
                             % (ready or "-", loaded or "-"))
            if not connected:
                warns.append("%s %s firmware commands not resolved"
                             % (kind, short))
        if len(oams_versions) > 1:
            warns.append("mixed OAMS firmware protocol versions %s"
                         % sorted(str(v) for v in oams_versions))
        for g in self.topo.groups:
            bays = ",".join("%s-%d" % e for e in self.topo.groups[g])
            lines.append("  group %s: lane=%s bays=%s"
                         % (g, self.topo.lane_of_group(g), bays or "(empty)"))
        for fps, ls in self.runtime.get_state().lanes.items():
            lines.append("  lane %s: op=%s group=%s unit=%s following=%s"
                         % (fps, ls.op, ls.group, ls.unit, ls.following))
        lines.append("  RESULT: %s"
                     % ("PASS" if not warns else "WARN -> " + "; ".join(warns)))
        gcmd.respond_info("\n".join(lines))

    def _resolve_fps(self, gcmd, prefer_op=None, required=True):
        """Resolve which FPS lane a command targets. Optional FPS= parameter;
        defaults to the sole lane, or (when ambiguous) the single lane currently
        in prefer_op. Raises a gcode error when ambiguous (so macros do not sail
        past it), unless required=False, in which case None is returned."""
        name = gcmd.get("FPS", None)
        if name is not None:
            if name not in self.fpss:
                raise gcmd.error("Unknown FPS '%s'" % name)
            return name
        if len(self.fpss) == 1:
            return next(iter(self.fpss))
        if prefer_op is not None:
            state = self.runtime.get_state()
            match = [fps for fps, lane in state.lanes.items()
                     if lane.op == prefer_op]
            if len(match) == 1:
                return match[0]
        if required:
            raise gcmd.error("Multiple FPS present; specify FPS=<name>")
        return None

    def _stop_followers(self, fps_names):
        """Stop followers on the given lanes, both in the store (so
        LaneState.following stays truthful) and on every unit of the lane (to
        cover followers enabled directly on idle units)."""
        for fps in fps_names:
            self.runtime.dispatch(S.Follow(fps, 0, S.FOLLOWER_REVERSE))
            for oam in self.lane_oams[fps]:
                oam.set_oams_follower(0, S.FOLLOWER_REVERSE)

    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"

    def cmd_CLEAR_ERRORS(self, gcmd):
        self.stop_monitor()
        for oam in self.oams.values():
            oam.clear_errors()
        self.runtime.dispatch(S.ClearErrors())
        self.start_monitor()

    cmd_CURRENT_LOADED_GROUP_help = "Report the currently loaded group(s)"

    def cmd_CURRENT_LOADED_GROUP(self, gcmd):
        state = self.runtime.get_state()
        loaded = {fps: lane.group for fps, lane in state.lanes.items()
                  if lane.op == S.OP_LOADED and lane.group}
        if not loaded:
            gcmd.respond_info("No group is currently loaded")
        elif len(loaded) == 1:
            gcmd.respond_info(next(iter(loaded.values())))
        else:
            gcmd.respond_info(", ".join("%s=%s" % (f, g)
                                        for f, g in loaded.items()))

    cmd_FOLLOWER_help = "Enable the follower on whichever OAMS is loaded on a lane"

    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int("ENABLE", minval=0, maxval=1)
        direction = gcmd.get_int("DIRECTION", minval=0, maxval=1)
        # Starting a follower needs an unambiguous lane (raise rather than
        # guess); stopping is always safe to broadcast, and stop commands run
        # in cleanup/error macros that must not abort.
        fps = self._resolve_fps(gcmd, prefer_op=S.OP_LOADED,
                                required=bool(enable))
        if fps is None:
            self._stop_followers(self.fpss)
            gcmd.respond_info("Followers stopped on all lanes")
            return
        lane = self.runtime.get_state().lanes[fps]
        if lane.unit is None:
            if not enable:
                self._stop_followers([fps])
            gcmd.respond_info("No spool is currently loaded on lane %s" % fps)
            return
        # Through the store so LaneState.following/direction stay truthful.
        self.runtime.dispatch(S.Follow(fps, enable, direction))

    cmd_LOAD_FILAMENT_CANCEL_help = "Cancel the in-flight load on a lane"

    def cmd_LOAD_FILAMENT_CANCEL(self, gcmd):
        # Cancel runs from cleanup/error macros; never abort them. With no
        # FPS= and no unambiguous loading lane there is nothing to cancel.
        fps = self._resolve_fps(gcmd, prefer_op=S.OP_LOADING, required=False)
        if fps is not None and self.runtime.get_state().lanes[fps].op == S.OP_LOADING:
            self.runtime.dispatch(S.Cancel(fps))
            gcmd.respond_info("Load cancel requested on lane %s" % fps)
        else:
            gcmd.respond_info("No load filament operation is in progress")

    cmd_UNLOAD_FILAMENT_help = "Unload the spool loaded on a lane"

    def cmd_UNLOAD_FILAMENT(self, gcmd):
        # SAFE_UNLOAD_FILAMENT enables the follower (rewind) before calling us,
        # so every path below that does not unload MUST stop followers — this
        # is the runaway-rewind case this command exists to prevent.
        fps = self._resolve_fps(gcmd, prefer_op=S.OP_LOADED, required=False)
        if fps is None:
            state = self.runtime.get_state()
            if any(lane.op == S.OP_LOADED for lane in state.lanes.values()):
                # Can't guess which loaded lane to unload — but before
                # aborting, stop any lane the store knows is rewinding
                # (a forward follower on a printing lane is left alone).
                rewinding = [f for f, lane in state.lanes.items()
                             if lane.following
                             and lane.direction == S.FOLLOWER_REVERSE]
                self._stop_followers(rewinding)
                raise gcmd.error(
                    "Multiple FPS have a spool loaded; specify FPS=<name>")
            # Multiple FPS, no FPS= given and nothing loaded anywhere: there is
            # no lane to unload, but a follower may still be rewinding. Stop
            # them all rather than leaving one running.
            self._stop_followers(self.fpss)
            gcmd.respond_info("No spool is loaded on any lane (followers"
                              " stopped); specify FPS=<name> to target a lane")
            return
        lane = self.runtime.get_state().lanes[fps]
        if lane.op != S.OP_LOADED:
            self._stop_followers([fps])
            gcmd.respond_info("No spool is loaded on lane %s (follower stopped)"
                              % fps)
            return
        result = self.runtime.request(fps, S.Unload(fps)).wait()
        if not result.ok:
            # Raise so SAFE_UNLOAD-style macros stop instead of carrying on
            # with a spool still (partially) loaded.
            raise gcmd.error(result.message)
        gcmd.respond_info(result.message)

    cmd_LOAD_FILAMENT_help = "Load a spool from a group (lane inferred from group)"

    def cmd_LOAD_FILAMENT(self, gcmd):
        group = gcmd.get("GROUP")
        fps = self.group_lane.get(group)
        if fps is None:
            if group in self.topo.groups:
                raise gcmd.error("Group %s has no bays assigned" % group)
            raise gcmd.error("Group %s does not exist" % group)
        result = self.runtime.request(fps, S.Load(fps, group)).wait()
        if result.ok or result.code == S.OAMS_OP_CODE_CANCEL:
            gcmd.respond_info(result.message)
        else:
            # Raise so toolchange macros stop instead of printing without
            # filament loaded.
            raise gcmd.error(result.message)

    # ------------------------------------------------ runtime group editing
    # Create groups and reassign bays at runtime (for the UI). Each change
    # validates through the pure model, writes the [filament_group] sections of
    # the OpenAMS config file in place, and then swaps the live model — so the
    # change takes effect immediately AND survives a restart, with no
    # SAVE_CONFIG (which can only rewrite printer.cfg, not an included subfile).
    # Edits are refused on a lane that is mid-op or printing from the affected
    # group, so the model cannot change underneath an in-flight operation.

    def _assert_editable(self, lanes, groups=()):
        """Refuse edits on a busy/runout lane, and refuse changing the
        membership of any group in `groups` that is currently loaded — the
        runout auto-reload picks its spare from the loaded group's bays, so
        mutating it mid-print would silently change what gets loaded next."""
        state = self.runtime.get_state()
        for lane in lanes:
            ls = state.lanes.get(lane)
            if ls is None:
                continue
            if ls.op in (S.OP_LOADING, S.OP_UNLOADING, S.OP_CALIBRATING):
                raise self.printer.command_error(
                    "Cannot edit filament groups while lane %s is busy (%s)."
                    % (lane, ls.op))
            if ls.op == S.OP_LOADED and ls.runout != S.RUNOUT_IDLE:
                raise self.printer.command_error(
                    "Cannot edit filament groups while lane %s is handling a"
                    " runout." % lane)
            if ls.op == S.OP_LOADED and ls.group and ls.group in groups:
                raise self.printer.command_error(
                    "Cannot edit group '%s' while it is loaded on lane %s;"
                    " unload it first." % (ls.group, lane))

    def _persist_and_apply(self, new_topo):
        """Persist the difference between the current and new topology to the
        config file, then swap in the new model. File first: if the write
        fails the running model is left untouched, so saved and live state can
        never diverge."""
        old = self.topo
        edits = []
        must_exist = []
        for g in new_topo.groups:
            if old.groups.get(g) != new_topo.groups[g]:
                edits.append((g, new_topo.group_config_value(g)))
                if g in old.groups:
                    must_exist.append(g)   # pre-existing group: update in place
        for g in old.groups:
            if g not in new_topo.groups:
                edits.append((g, None))           # delete the section
                must_exist.append(g)

        def transform(text):
            # A pre-existing group whose section is NOT in this file lives in
            # another config file (e.g. printer.cfg). Appending a copy here
            # would create a duplicate section; Klipper merges duplicates with
            # later-wins semantics, so the edit would silently apply-or-not
            # depending on include order — refuse with a pointer at the knob.
            for g in must_exist:
                if not oams_config_io.has_group(text, g):
                    raise self.printer.command_error(
                        "[filament_group %s] is not defined in '%s'; move the"
                        " section into that file or point [oams_manager]"
                        " openams_config_path at the file that holds it."
                        % (g, self.openams_config_path))
            return oams_config_io.apply_group_edits(text, edits)

        self._rewrite_config(transform)
        self.topo = new_topo
        self._rebuild_derived()

    def persist_config_option(self, section, option, value):
        """Set one option in one section of the OpenAMS config file in place
        (used by calibration writeback). Like group edits, this targets the
        included subfile SAVE_CONFIG cannot reach. Refuses (rather than
        appending a duplicate section) when the section lives in a different
        file. Raises command_error on any failure."""
        def transform(text):
            if not oams_config_io.has_section(text, section):
                raise self.printer.command_error(
                    "[%s] is not defined in '%s'; move the section into that"
                    " file or point [oams_manager] openams_config_path at the"
                    " file that holds it." % (section, self.openams_config_path))
            return oams_config_io.set_option(text, section, option, value)

        self._rewrite_config(transform)

    def _rewrite_config(self, transform):
        """Atomically read -> transform -> write the OpenAMS config file."""
        path = self.openams_config_path
        try:
            with open(path) as f:
                text = f.read()
        except OSError as e:
            raise self.printer.command_error(
                "Cannot read OpenAMS config '%s' to persist the change"
                " (set [oams_manager] openams_config_path): %s" % (path, e))
        new_text = transform(text)
        tmp = path + ".tmp"
        try:
            with open(tmp, "w") as f:
                f.write(new_text)
            os.replace(tmp, path)                 # atomic
        except OSError as e:
            raise self.printer.command_error(
                "Cannot write OpenAMS config '%s': %s" % (path, e))

    cmd_CREATE_GROUP_help = "Create a new (empty) filament group at runtime"

    def cmd_CREATE_GROUP(self, gcmd):
        name = gcmd.get("GROUP")
        try:
            topo = T.with_group(self.topo, name)
        except T.TopologyError as e:
            raise gcmd.error(str(e))
        self._persist_and_apply(topo)
        gcmd.respond_info("Created group '%s' (saved to %s)."
                          % (name, self.openams_config_path))

    cmd_DELETE_GROUP_help = "Delete a filament group at runtime"

    def cmd_DELETE_GROUP(self, gcmd):
        name = gcmd.get("GROUP")
        if name not in self.topo.groups:
            raise gcmd.error("filament_group '%s' does not exist." % name)
        self._assert_editable([self.topo.lane_of_group(name)], groups=(name,))
        self._persist_and_apply(T.without_group(self.topo, name))
        gcmd.respond_info("Deleted group '%s' (saved to %s)."
                          % (name, self.openams_config_path))

    cmd_ASSIGN_BAY_help = "Assign an OAMS bay to a filament group at runtime"

    def cmd_ASSIGN_BAY(self, gcmd):
        group = gcmd.get("GROUP")
        oams_name = gcmd.get("OAMS")
        bay = gcmd.get_int("BAY", minval=0, maxval=3)
        if oams_name not in self.oams:
            raise gcmd.error("Unknown OAMS '%s'." % oams_name)
        affected = {self.topo.lane_of_oams(oams_name)}
        # Both groups whose membership changes are protected while loaded: the
        # destination AND the donor (the group the bay is being moved out of).
        touched = {group}
        donor = next((g for g, bays in self.topo.groups.items()
                      if (oams_name, bay) in bays), None)
        if donor is not None:
            touched.add(donor)
        for g in touched:
            if g in self.topo.groups:
                gl = self.topo.lane_of_group(g)
                if gl is not None:
                    affected.add(gl)
        self._assert_editable(affected, groups=touched)
        try:
            topo = T.with_bay(self.topo, group, oams_name, bay)
        except T.TopologyError as e:
            raise gcmd.error(str(e))
        self._persist_and_apply(topo)
        gcmd.respond_info("Assigned %s-%d to '%s' (saved to %s)."
                          % (oams_name, bay, group, self.openams_config_path))

    cmd_UNASSIGN_BAY_help = "Remove an OAMS bay from a filament group at runtime"

    def cmd_UNASSIGN_BAY(self, gcmd):
        group = gcmd.get("GROUP")
        oams_name = gcmd.get("OAMS")
        bay = gcmd.get_int("BAY", minval=0, maxval=3)
        if group not in self.topo.groups:
            raise gcmd.error("filament_group '%s' does not exist." % group)
        self._assert_editable([self.topo.lane_of_group(group)], groups=(group,))
        try:
            topo = T.without_bay(self.topo, group, oams_name, bay)
        except T.TopologyError as e:
            raise gcmd.error(str(e))
        self._persist_and_apply(topo)
        gcmd.respond_info("Removed %s-%d from '%s' (saved to %s)."
                          % (oams_name, bay, group, self.openams_config_path))


def load_config(config):
    return OAMSManager(config)
