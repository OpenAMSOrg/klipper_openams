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

from . import oams_state as S
from .oams_runtime import Runtime


class OAMSManager:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.reload_before = config.getfloat("reload_before_toolhead_distance", 0.0)

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

        self.register_commands()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    # --------------------------------------------------------------- topology

    def _build_topology(self):
        # FPS lanes, keyed by short name.
        self.fpss = {}
        for _, fps in self.printer.lookup_objects(module="fps"):
            self.fpss[fps.fps_name] = fps
        if not self.fpss:
            raise self.config.error(
                "No [fps] section found; OpenAMS requires at least one FPS.")
        sole_fps = next(iter(self.fpss)) if len(self.fpss) == 1 else None

        # OAMS units + lane assignment.
        self.oams = {}
        self.oam_by_idx = {}
        self.lane_oams = {name: [] for name in self.fpss}
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
            fps_name = oam.fps_name or sole_fps
            if fps_name is None:
                raise self.config.error(
                    "%s must set 'fps:' when multiple [fps] are defined" % name)
            if fps_name not in self.fpss:
                raise self.config.error(
                    "%s references unknown fps '%s'" % (name, fps_name))
            oam.fps_name = fps_name
            self.lane_oams[fps_name].append(oam)
            self.oam_by_idx[oam.oams_idx] = oam

        # Filament groups -> lane + per-lane (oams_idx, bay) lists. A group's bays
        # must all live on one FPS lane (invariant G1).
        self.filament_groups = {}
        self.group_lane = {}
        self.groups_by_lane = {name: {} for name in self.fpss}
        seen = set()
        for name, group in self.printer.lookup_objects(module="filament_group"):
            gname = name.split()[-1]
            if gname in seen:
                raise self.config.error(
                    "Duplicate filament_group '%s'; names must be unique." % gname)
            seen.add(gname)
            self.filament_groups[gname] = group
            lane = None
            bays = []
            for (oam, bay) in group.bays:
                oam_fps = oam.fps_name
                if lane is None:
                    lane = oam_fps
                elif lane != oam_fps:
                    raise self.config.error(
                        "filament_group '%s' spans multiple FPS lanes (%s and %s);"
                        " a group's bays must share one FPS" % (gname, lane, oam_fps))
                bays.append((oam.oams_idx, bay))
            if lane is not None:
                self.group_lane[gname] = lane
                self.groups_by_lane[lane][gname] = tuple(bays)
            logging.info("OAMS: group %s on lane %s -> %s", gname, lane, bays)

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
                path_len[oam.oams_idx] = oam.filament_path_length
            epos = fps.extruder.last_position if fps.extruder is not None else 0.0
            lanes[fps_name] = S.LaneWorld(
                extruder_pos=epos, printing=printing, loaded=loaded, ready=ready,
                group_bays=self.groups_by_lane.get(fps_name, {}),
                path_len=path_len, reload_before=self.reload_before)
        return S.World(lanes=lanes)

    # ----------------------------------------------------------------- ready

    def handle_ready(self):
        self.idle_timeout = self.printer.lookup_object("idle_timeout")
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
        return eventtime + S.MONITOR_INTERVAL

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
        for gname, group in self.filament_groups.items():
            status["filament_groups"][gname] = {
                "lane": self.group_lane.get(gname),
                "bays": ["oams%d-%d" % (oam.oams_idx, bay)
                         for (oam, bay) in group.bays]}
        request.send({"status": {"openams": status}})

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
        ):
            gcode.register_command(name, handler, desc=desc)

    def _resolve_fps(self, gcmd, prefer_op=None):
        """Resolve which FPS lane a command targets. Optional FPS= parameter;
        defaults to the sole lane, or (when ambiguous) the single lane currently
        in prefer_op. Returns the fps name, or None after responding to gcmd."""
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
        gcmd.respond_info("Multiple FPS present; specify FPS=<name>")
        return None

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
        fps = self._resolve_fps(gcmd, prefer_op=S.OP_LOADED)
        if fps is None:
            return
        lane = self.runtime.get_state().lanes[fps]
        if lane.unit is None:
            gcmd.respond_info("No spool is currently loaded on lane %s" % fps)
            return
        oam = self.oam_by_idx.get(lane.unit[0])
        if oam is not None:
            oam.set_oams_follower(enable, direction)

    cmd_LOAD_FILAMENT_CANCEL_help = "Cancel the in-flight load on a lane"

    def cmd_LOAD_FILAMENT_CANCEL(self, gcmd):
        fps = self._resolve_fps(gcmd, prefer_op=S.OP_LOADING)
        if fps is None:
            return
        lane = self.runtime.get_state().lanes[fps]
        if lane.op == S.OP_LOADING:
            self.runtime.dispatch(S.Cancel(fps))
            gcmd.respond_info("Load cancel requested on lane %s" % fps)
        else:
            gcmd.respond_info("No load filament operation is in progress")

    cmd_UNLOAD_FILAMENT_help = "Unload the spool loaded on a lane"

    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps = self._resolve_fps(gcmd, prefer_op=S.OP_LOADED)
        if fps is None:
            return
        lane = self.runtime.get_state().lanes[fps]
        if lane.op != S.OP_LOADED:
            # SAFE_UNLOAD_FILAMENT enables the follower (rewind) before calling
            # us; if nothing is loaded, stop it so it cannot keep rewinding.
            for oam in self.lane_oams[fps]:
                oam.set_oams_follower(0, S.FOLLOWER_REVERSE)
            gcmd.respond_info("No spool is loaded on lane %s (follower stopped)"
                              % fps)
            return
        result = self.runtime.request(fps, S.Unload(fps)).wait()
        gcmd.respond_info(result.message)

    cmd_LOAD_FILAMENT_help = "Load a spool from a group (lane inferred from group)"

    def cmd_LOAD_FILAMENT(self, gcmd):
        group = gcmd.get("GROUP")
        fps = self.group_lane.get(group)
        if fps is None:
            raise gcmd.error("Group %s does not exist" % group)
        result = self.runtime.request(fps, S.Load(fps, group)).wait()
        gcmd.respond_info(result.message)


def load_config(config):
    return OAMSManager(config)
