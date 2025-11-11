# OpenAMS Manager
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


import logging
import time
from functools import partial
from collections import deque

PAUSE_DISTANCE = 60
ENCODER_SAMPLES = 2
MIN_ENCODER_DIFF = 1
FILAMENT_PATH_LENGTH_FACTOR = 1.14  # Replace magic number with a named constant
MONITOR_ENCODER_LOADING_SPEED_AFTER = 2.0 # in seconds
MONITOR_ENCODER_UNLOADING_SPEED_AFTER = 2.0 # in seconds

class OAMSState:
    def __init__(self, name, since, current_spool):
        self.name = name
        self.since = since
        self.current_spool = current_spool
        self.following = False
        self.direction = 0

class OAMSManager:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.filament_groups = {}
        self.oams = {}
        self._initialize_oams()
        self._initialize_filament_groups()
        self.current_state = OAMSState(None, None, None)
        self.reactor = self.printer.get_reactor()
        
        self.encoder_samples = deque(maxlen=ENCODER_SAMPLES)
        
        # runout variables
        self.current_group = None
        self.current_spool = None
        self.runout_position = None
        self.runout_after_position = None
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None
        self.monitor_timers = []
        self.ready = False

        self.fps = self.printer.lookup_object("fps")
        
        self.reload_before_toolhead_distance = config.getfloat("reload_before_toolhead_distance", 0.0)

        self.webhooks = self.printer.lookup_object('webhooks')
        self.webhooks.register_endpoint("openams/status", self._webhook_status)
        
        self.register_commands()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def _webhook_status(self, request):
        status = {"ready" : self.ready, "current_group" : self.current_group}
        status["units"] = len(self.oams)
        status["fps_value"] = self.fps.get_value()
        status["filament_groups"] = {}
        for group_name, group in self.filament_groups.items():
            status["filament_groups"][group_name] = {
                "bays": len(group.bays),
                "spools": ["oams" + str(oam.oams_idx) + "-" + str(bay_index) for (oam, bay_index) in group.bays]
            }
        request.send({"status" : {"openams": status}})

    def get_status(self, eventtime):
        return {"current_group": self.current_group}
    
    def determine_state(self):
        self.current_group, current_oam, current_spool_idx = self.determine_current_loaded_group()
        if current_oam is not None and current_spool_idx is not None:
            self.current_spool = (current_oam, current_spool_idx)
            self.current_state = OAMSState("LOADED", self.reactor.monotonic() , self.current_spool)
        else:
            self.current_spool = None
            self.current_state = OAMSState("UNLOADED", self.reactor.monotonic() , None)
        
    def handle_ready(self):
        self.determine_state()
        self.start_monitors()
        self.ready = True

    def _log_status(self, is_printing):
        logging.info(
            f"""
    OAMS: print status = {is_printing}, current_group = {self.current_group}, current_spool_oam = {None if self.current_spool is None else self.current_spool[0].name}, current_spool_idx = {None if self.current_spool is None else self.current_spool[1]}, ran_out = {self.current_spool is not None and not bool(self.current_spool[0].hub_hes_value[self.current_spool[1]])}, runout_position = {self.runout_position}, traveled_after_runout = {self.runout_after_position}
            """
        )
    
    def _pause_before_coasting(self, eventtime, initial_position, pause_distance):
        extruder = self.printer.lookup_object("extruder")
        current_position = extruder.last_position
        traveled_distance = current_position - initial_position
        if traveled_distance >= pause_distance:
            logging.info("OAMS: Pause complete, coasting the follower.")
            self.current_spool[0].set_oams_follower(0, 1)
            self._register_load_next_spool_timer(eventtime, pause_distance)
            return self.printer.get_reactor().NEVER
        return eventtime + 1.0
    
    def _register_pause_timer(self, eventtime, pause_distance):
        logging.info(f"OAMS: Filament runout detected, pausing for {pause_distance} mm before coasting the follower.")
        extruder = self.printer.lookup_object("extruder")
        initial_position = extruder.last_position
        self.monitor_pause_timer = self.printer.get_reactor().register_timer(
            lambda et: self._pause_before_coasting(et, initial_position, pause_distance), eventtime)
    
    def _load_next_spool(self, eventtime, pause_distance):
        extruder = self.printer.lookup_object("extruder")
        try:
            if self.runout_position is None:
                self.runout_position = extruder.last_position
                logging.info(f"OAMS: Runout position set to {self.runout_position}")
            else:
                self.runout_after_position = extruder.last_position - self.runout_position
                logging.info(f"OAMS: Traveled after runout: {self.runout_after_position}")
                if self.runout_after_position + pause_distance + self.reload_before_toolhead_distance > self.current_spool[0].filament_path_length / FILAMENT_PATH_LENGTH_FACTOR:
                    logging.info("OAMS: Loading next spool in the filament group.")
                    for (oam, bay_index) in self.filament_groups[self.current_group].bays:
                        if oam.is_bay_ready(bay_index):
                            success, message = oam.load_spool(bay_index)
                            if success:
                                logging.info(f"OAMS: Successfully loaded spool in bay {bay_index} of OAM {oam.name}")
                                self.current_spool = (oam, bay_index)
                                self.runout_position = None
                                self.runout_after_position = None
                                self._register_monitor_spool_timer()
                                return self.printer.get_reactor().NEVER
                            else:
                                logging.error(f"OAMS: Failed to load spool: {message}")
                                raise Exception(message)
                    self._pause_print()
                    return self.printer.get_reactor().NEVER
        except Exception as e:
            logging.error(f"OAMS: Error while loading next spool: {str(e)}")
            self._pause_print()
            return self.printer.get_reactor().NEVER
        return eventtime + 1.0
    
    def _register_load_next_spool_timer(self, eventtime, pause_distance):
        logging.info("OAMS: Registering timer to load next spool.")
        self.monitor_load_next_spool_timer = self.printer.get_reactor().register_timer(
            lambda et: self._load_next_spool(et, pause_distance), eventtime)
    
    def _pause_print(self):
        logging.info("OAMS: No spool available, pausing the print.")
        gcode = self.printer.lookup_object("gcode")
        message = f"Print has been paused due to filament runout on group {self.current_group}"
        gcode.run_script(f"M118 {message}")
        gcode.run_script(f"M114 {message}")
        gcode.run_script("PAUSE")
        self.current_group = None
        self.current_spool = None
        self.runout_position = None
        self.runout_after_position = None
        self._register_monitor_spool_timer()
    
    def _monitor_spool(self, eventtime):
        idle_timeout = self.printer.lookup_object("idle_timeout")
        is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
    
        if is_printing and \
            self.current_state.name == "LOADED" and \
            self.current_group is not None and \
            self.current_spool is not None and \
            not bool(self.current_spool[0].hub_hes_value[self.current_spool[1]]):
            self._register_pause_timer(eventtime, PAUSE_DISTANCE)
            return self.printer.get_reactor().NEVER
    
        return eventtime + 1.0
    
    def _register_monitor_spool_timer(self):
        reactor = self.printer.get_reactor()
        self.monitor_spool_timer = reactor.register_timer(self._monitor_spool, reactor.NOW)

    def _initialize_oams(self):
        for (name, oam) in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def _initialize_filament_groups(self):
        for (name, group) in self.printer.lookup_objects(module="filament_group"):
            name = name.split()[-1]
            logging.info(f"OAMS: Adding group {name}")
            self.filament_groups[name] = group
    
    def determine_current_loaded_group(self):
        for group_name, group in self.filament_groups.items():
            for (oam, bay_index) in group.bays:
                if oam.is_bay_loaded(bay_index):
                    return group_name, oam, bay_index
        return None, None, None
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "OAMSM_UNLOAD_FILAMENT",
            self.cmd_UNLOAD_FILAMENT,
            desc=self.cmd_UNLOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "OAMSM_LOAD_FILAMENT",
            self.cmd_LOAD_FILAMENT,
            desc=self.cmd_LOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "OAMSM_FOLLOWER",
            self.cmd_FOLLOWER,
            desc=self.cmd_FOLLOWER_help,
        )
        
        gcode.register_command(
            "OAMSM_CURRENT_LOADED_GROUP",
            self.cmd_CURRENT_LOADED_GROUP,
            desc=self.cmd_CURRENT_LOADED_GROUP_help,
        )
        
        gcode.register_command(
            "OAMSM_CLEAR_ERRORS",
            self.cmd_CLEAR_ERRORS,
            desc=self.cmd_CLEAR_ERRORS_help,
        )
    
    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"
    def cmd_CLEAR_ERRORS(self, gcmd):
        if len(self.monitor_timers) > 0:
            self.stop_monitors()
        self.encoder_samples.clear()
        for _, oam in self.oams.items():
            oam.clear_errors()
        self.determine_state()
        self.start_monitors()
        
        return
    
    cmd_CURRENT_LOADED_GROUP_help = "Get the current loaded group"
    def cmd_CURRENT_LOADED_GROUP(self, gcmd):
        group_name, _, _ = self.determine_current_loaded_group()
        if group_name is not None:
            gcmd.respond_info(group_name)
        else:
            gcmd.respond_info("No group is currently loaded")
        return
    
    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        if enable is None:
            gcmd.respond_info("Missing ENABLE parameter")
            return
        direction = gcmd.get_int('DIRECTION')
        if direction is None:
            gcmd.respond_info("Missing DIRECTION parameter")
            return
        loaded = False
        for _, oam in self.oams.items():
            if oam.current_spool is not None:
                oam.set_oams_follower(enable, direction)
                self.current_state.following = enable
                self.current_state.direction = direction
                self.current_state.encoder = oam.encoder_clicks
                self.current_state.current_spool = (oam, oam.current_spool)
                loaded = True
        if not loaded:
            gcmd.respond_info("No spool is currently loaded")
        return
    
    def is_printer_loaded(self):
        for _, oam in self.oams.items():
            if oam.current_spool is not None:
                return True
        return False
    
    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        for _, oam in self.oams.items():
            if oam.current_spool is not None:
                self.current_state.name = "UNLOADING"
                self.current_state.encoder = oam.encoder_clicks
                self.current_state.since = self.reactor.monotonic()
                self.current_state.current_spool = (oam, oam.current_spool)
                
                
                success, message = oam.unload_spool()
                
                if success:
                    self.current_state.name = "UNLOADED"
                    self.current_state.current_spool = None
                    self.current_state.following = False
                    self.current_state.direction = 0
                    self.current_state.since = self.reactor.monotonic()
                    
                    self.current_group = None
                    self.current_spool = None
                    return
                else:
                    gcmd.respond_info(message)
                    return
        gcmd.respond_info("No spool is loaded in any of the OAMS")
        self.current_group = None
        return
        
    cmd_LOAD_FILAMENT_help = "Load a spool from an specific group"
    def cmd_LOAD_FILAMENT(self, gcmd):
        if self.is_printer_loaded():
            gcmd.respond_info("Printer is already loaded with a spool")
            return
        group_name = gcmd.get('GROUP')
        if group_name not in self.filament_groups:
            gcmd.respond_info(f"Group {group_name} does not exist")
            return
        for (oam, bay_index) in self.filament_groups[group_name].bays:
            if oam.is_bay_ready(bay_index):
                self.current_state.name = "LOADING"
                self.current_state.encoder = oam.encoder_clicks
                self.current_state.since = self.reactor.monotonic()
                self.current_state.current_spool = (oam, bay_index)
                
                success, message = oam.load_spool(bay_index)
                
                if success:
                    self.current_group = group_name
                    self.current_spool = (oam, bay_index)
                    
                    self.current_state.name = "LOADED"
                    self.current_state.since = self.reactor.monotonic()
                    self.current_state.current_spool = self.current_spool
                    self.current_state.following = False
                    self.current_state.direction = 1
                    
                    gcmd.respond_info(message)
                    self.current_group = group_name
                    return
                else:
                    gcmd.respond_info(message)
                    return
        gcmd.respond_info(f"No spool available for group {group_name}")
        return
        
    def _pause_printer_message(self, message):
        logging.info(f"OAMS: {message}")
        gcode = self.printer.lookup_object("gcode")
        message = f"Print has been paused: {message}"
        gcode.run_script(f"M118 {message}")
        gcode.run_script(f"M114 {message}")
        gcode.run_script("PAUSE")
        
    def _monitor_unload_speed(self, eventtime):
        #logging.info("OAMS: Monitoring unloading speed state: %s" % self.current_state.name)
        if self.current_state.name == "UNLOADING" and self.reactor.monotonic() - self.current_state.since > MONITOR_ENCODER_UNLOADING_SPEED_AFTER:
            self.encoder_samples.append(self.current_state.current_spool[0].encoder_clicks)
            if len(self.encoder_samples) < ENCODER_SAMPLES:
                return eventtime + 1.0
            encoder_diff = abs(self.encoder_samples[-1] - self.encoder_samples[0])
            logging.info("OAMS[%d] Unload Monitor: Encoder diff %d" %(self.current_state.current_spool[0].oams_idx, encoder_diff))
            if encoder_diff < MIN_ENCODER_DIFF:              
                self.current_state.current_spool[0].set_led_error(self.current_state.current_spool[1], 1)
                self._pause_printer_message("Printer paused because the unloading speed of the moving filament was too low")
                logging.info("after unload speed too low")
                self.stop_monitors()
                return self.printer.get_reactor().NEVER
        return eventtime + 1.0
    
    def _monitor_load_speed(self, eventtime):
        #logging.info("OAMS: Monitoring loading speed state: %s" % self.current_state.name)
        if self.current_state.name == "LOADING" and self.reactor.monotonic() - self.current_state.since > MONITOR_ENCODER_LOADING_SPEED_AFTER:
            self.encoder_samples.append(self.current_state.current_spool[0].encoder_clicks)
            if len(self.encoder_samples) < ENCODER_SAMPLES:
                return eventtime + 1.0
            encoder_diff = abs(self.encoder_samples[-1] - self.encoder_samples[0])
            logging.info("OAMS[%d] Load Monitor: Encoder diff %d" %(self.current_state.current_spool[0].oams_idx, encoder_diff))
            if encoder_diff < MIN_ENCODER_DIFF:
                self.current_state.current_spool[0].set_led_error(self.current_state.current_spool[1], 1)               
                self._pause_printer_message("Printer paused because the loading speed of the moving filament was too low")
                self.stop_monitors()
                return self.printer.get_reactor().NEVER
        return eventtime + 1.0
    
    def start_monitors(self):
        self.monitor_timers = []
        reactor = self.printer.get_reactor()
        self.monitor_timers.append(reactor.register_timer(self._monitor_unload_speed, reactor.NOW))
        self.monitor_timers.append(reactor.register_timer(self._monitor_load_speed, reactor.NOW))
        self.monitor_timers.append(reactor.register_timer(self._monitor_spool, reactor.NOW))
        logging.info("OAMS: Monitors started")
    
    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []

def load_config(config):
    return OAMSManager(config)
