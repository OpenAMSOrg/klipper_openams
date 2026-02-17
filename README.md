image
OpenAMS for Klipper
Version 0.0.3 — Lane-based AFC integration with event-driven monitoring

A Klipper integration for OpenAMS that enables multi-material printing with automatic filament management, runout detection, and intelligent retry logic.

Table of Contents
Overview
Repository Structure
Architecture
What's New
Features
Prerequisites
Installation
Install/Update AFC
Lane Architecture Primer
Install the AFC Add-On
Stage AFC Configuration Templates
AFC Hardware Configuration Checklist
Install OpenAMS
Switching to This Fork
Custom Installation Paths
Apply AFC Integration Files
Configuration
OpenAMS Manager Settings
OAMS Hardware Settings
FPS Configuration
Retry Behavior
Clog Detection Settings
Advanced Detection Tunables
Optional Features
Mainsail AFC Panel
Initial Calibration
Infinite Spooling
Troubleshooting
CAN Bus Debugging
Credits
Overview
OpenAMS provides automated filament handling for Klipper-based 3D printers. This fork integrates tightly with Armored Turtle's AFC (Automatic Filament Changer) add-on using a lane-based architecture. The combination delivers end-to-end multi-material automation—from AFC's physical filament routing to OpenAMS' retry logic, runout handling, and print-state awareness.

Full Integration at a Glance
AFC exposes lanes, runout sensors, and hub LEDs.
OpenAMS maps those AFC lanes into the AMS manager, applies retry and clog detection logic, and keeps Moonraker/Klipper informed of state changes.
Optional services like Spoolman enrich the integration with live spool metadata.
Example Settings
The snippets below show how the integration pieces fit together. Adjust lane names and MCU UUIDs to match your hardware.

OpenAMS Manager
AFC Lane Mapping
OAMS Retry Settings
These configuration blocks are sourced from the repository templates—AFC_Oams.cfg contains the [oams_manager] and [oams ...] sections while AFC_AMS_1.cfg defines the [AFC_lane ...] entries—and reference the synced AFC extras installed in later steps.

Key Capabilities
Lane-based filament management through AFC integration
Automatic filament loading and unloading with pressure sensing
Intelligent retry logic for stuck filament detection
Clog detection with configurable sensitivity
Event-driven sensor monitoring for optimal performance
Runout detection and automatic lane switching
Spoolman integration for filament tracking
LED status indicators
Repository Structure
klipper_openams/
├── README.md                       # This documentation
├── LICENSE                         # MIT License
├── install-openams.sh              # Installation / uninstall script
├── AFC_OpenAMS.py                  # AFC integration patch (copied to AFC extras)
├── openams_integration.py          # Shared helpers & event system (copied to AFC extras)
├── AFC_AMS_1.cfg                   # Lane configuration template
├── AFC_Oams.cfg                    # OAMS hardware & manager config template
├── mainsail.zip                    # Optional Mainsail AFC panel
│
├── src/                            # Core Klipper modules (linked to klippy/extras)
│   ├── oams_manager.py             # Central manager / coordinator
│   ├── oams.py                     # OAMS hardware controller (CAN bus)
│   ├── fps.py                      # Filament Pressure Sensor driver (ADC)
│   ├── hdc1080.py                  # HDC1080 temperature & humidity sensor (I2C)
│   └── openams_moonraker.py        # Moonraker database client for state persistence
│
├── scripts/                        # Utility scripts (linked to klipper/scripts)
│   └── canbus_logger.py            # CAN bus log viewer for firmware debugging
│
└── file_templates/                 # Templates consumed by the installer
    ├── HDC1080.cfg                 # HDC1080 sensor type registration
    ├── moonraker_update.txt        # Moonraker update manager entry
    └── openams.service             # Systemd service file
Architecture
OpenAMS sits between Klipper/AFC and the physical hardware. The diagram below shows how the major modules relate to each other.

┌────────────────────────────────────────────────────────────┐
│                   Klipper / Moonraker                      │
└────────────────┬──────────────────────────┬────────────────┘
                 │                          │
       ┌─────────────────────┐    ┌──────────────────────┐
       │   oams_manager.py   │    │ openams_moonraker.py │
       │  (State machine,    │    │ (Status persistence  │
       │   retry logic,      │◄──►│  via Moonraker DB)   │
       │   clog / runout     │    └──────────────────────┘
       │   detection)        │
       └───┬────────┬────────┘
           │        │
   ┌───────┘        └────────┐
   ▼                         ▼
┌──────────┐          ┌──────────┐
│  oams.py │          │  fps.py  │
│ (CAN bus │          │ (ADC     │
│  motor,  │          │  pressure│
│  encoder,│          │  sensor) │
│  HES)    │          └──────────┘
└──────────┘
       │
       ▼
┌──────────────┐
│  hdc1080.py  │
│ (I2C temp /  │
│  humidity)   │
└──────────────┘

┌────────────────────────────────────────────────────────────┐
│         AFC Integration Layer (installed into AFC)         │
│                                                            │
│  AFC_OpenAMS.py          openams_integration.py            │
│  (Monkey-patches AFC     (AMSEventBus, AMSHardwareService, │
│   lanes to call OpenAMS   LaneRegistry, RunoutCoordinator, │
│   load/unload/runout)     OpenAMSManagerFacade)            │
└────────────────────────────────────────────────────────────┘
Module summary:

Module	Location	Role
oams_manager.py	src/ → klippy/extras/	Central coordinator — monitors encoder, FPS pressure, and F1S sensors; drives retry logic, clog detection, stuck spool detection, and runout handling
oams.py	src/ → klippy/extras/	Hardware controller — talks to the OAMS mainboard over CAN bus to control BLDC motors, read encoder clicks, and query Hall-Effect Sensors
fps.py	src/ → klippy/extras/	Filament Pressure Sensor — ADC-based driver that reads buffer pressure (0.0–1.0) and dispatches callbacks on value changes
hdc1080.py	src/ → klippy/extras/	HDC1080 I2C sensor driver — reports enclosure temperature and humidity to Klipper/Moonraker
openams_moonraker.py	src/ → klippy/extras/	Lightweight HTTP client that publishes manager status to the Moonraker database for web UI display and state recovery on restart
AFC_OpenAMS.py	repo root → AFC extras/	Patches AFC lane load/unload/runout paths to route through OpenAMS, adds virtual filament sensors and TD-1 capture
openams_integration.py	repo root → AFC extras/	Shared event bus, unified sensor polling service, lane registry, runout coordinator, and manager facade used by both AFC and OpenAMS
canbus_logger.py	scripts/ → klipper/scripts/	Real-time CAN bus log viewer — decodes OAMS firmware log messages with color-coded severity levels
What's New
v0.0.3 — Current Release
Moonraker State Persistence:

New openams_moonraker.py client publishes manager status to the Moonraker database
Fingerprint-based deduplication avoids redundant writes
Automatic retry on transient network failures
Status survives Klipper restarts via database read-back
Performance Optimizations:

Unified sensor polling through AMSHardwareService reduces duplicate MCU communication by ~50%
Adaptive polling intervals (2.0 s active / 4.0 s idle) cut CPU overhead by 15–25% when the printer is idle
Object caching for frequently accessed Klipper objects (idle_timeout, gcode, toolhead, AFC)
State change tracking with intelligent interval switching
Enhanced Detection Tunables:

All pressure thresholds and timing windows are now configurable in [oams_manager]
Per-FPS reload_before_toolhead_distance override for mixed-length setups
Configurable engagement pressure threshold for post-load verification
Increased grace periods and suppression windows to reduce false positives
Kalico (Danger Klipper) Support:

FPS driver supports use_kalico: True for setups running Kalico instead of stock Klipper
Lane-Based Architecture (Baseline)
The system uses a lane-based architecture for AFC integration:

AFC Lanes: Each OpenAMS slot is configured as an AFC lane with independent settings
Event-Driven Sensors: Sensor monitoring uses event-based callbacks instead of constant polling
AFC Runout Integration: Runout handling integrates directly with AFC's lane system
Hub Mapping: Each lane maps to its own hub for visual indication and broken filament detection
Detection Tuning: [oams_manager] options for pressure thresholds and dwell windows
Migration Notes (from pre-lane versions):

Filament group configuration must be converted to AFC lanes
Macro calls changed from GROUP=T0 to LANE=lane0 format
Runout configuration uses AFC's SET_RUNOUT command instead of OpenAMS filament groups
See the Install/Update AFC section for detailed configuration examples
Features
Lane-Based Architecture: Integration with AFC lanes for flexible spool configuration and mapping
Event-Driven Sensors: Efficient event-based monitoring instead of constant polling for better performance
Automatic Retry Logic: Configurable retry attempts for both load and unload operations with a configurable delay
Stuck Spool Detection: Pressure-based detection with configurable thresholds and automatic retry before pausing
Clog Detection: Three sensitivity levels (low, medium, high) to detect filament clogs during printing
Runout Handling: Automatic filament runout detection integrated with AFC lane system, including cross-extruder runout support
Infinite Spooling: Seamless lane switching for continuous printing using AFC runout configuration
Moonraker State Persistence: Manager status published to the Moonraker database for web UI display and restart recovery
LED Status Indicators: Visual feedback through lane LEDs
HDC1080 Sensor Support: Temperature and humidity monitoring within the AMS unit
Kalico Compatibility: FPS driver supports both stock Klipper and Kalico (Danger Klipper)
Prerequisites
Before installing OpenAMS, ensure you have:

Klipper installed and running
Moonraker configured
MCU's id's uuid's or serial id for FPS and AMS board
Optional but recommended:
Spoolman for filament tracking
Mainsail or Fluidd web interface
Installation
Follow the steps below in order to ensure a working AFC + OpenAMS setup. Each stage builds on the previous one.

Clone This Repository First
The AFC installer will copy OpenAMS-specific files during setup, so start by cloning this repository:

cd ~
git clone https://github.com/lindnjoe/klipper_openams.git
cd klipper_openams
Leave the repository in place for the later steps—you'll stage configuration templates from it and then run the OpenAMS installer.

1. Install/Update AFC
This step lays down the Armored Turtle AFC framework that OpenAMS builds on. Complete each subsection before moving to the OpenAMS installer.

Lane Architecture Primer
OpenAMS integrates with AFC through lanes instead of the legacy filament group system. Each lane represents:

One physical spool slot on your OpenAMS unit
A mapping to a specific hub for filament routing
Optional LED indicators
Association with a tool number (T0, T1, T2, T3, etc.)
Benefits of lane-based configuration:

More flexible spool-to-tool mappings
Better integration with AFC's toolchange system
Easier to configure runout behavior per lane
Supports multiple hubs and complex routing
Example: A single OpenAMS with four slots becomes four AFC lanes (lane0–lane3), each independently configurable with its own hub and runout behavior.

Install the AFC Add-On
Clone the Armored Turtle repository (OpenAMS should already be cloned from the previous step):

cd ~
git clone https://github.com/ArmoredTurtle/AFC-Klipper-Add-On.git
Install the AFC add-on from the multi_extruder branch:

cd ~/AFC-Klipper-Add-On
./install-afc.sh -b multi_extruder
Important setup notes:

Read the AFC documentation: Most of the Armored Turtle setup guide applies directly to OpenAMS. Review it at https://www.armoredturtle.xyz/docs/afc-klipper-add-on/index.html.

Box Turtle vs OpenAMS: Some documentation is specific to Box Turtle hardware and will not apply to OpenAMS. When in doubt, leave defaults in place—the OpenAMS-specific config files override Box Turtle defaults.

Installation prompts: During the AFC installation, you'll be presented with an interactive menu:

Press T to cycle through installation types until you see OpenAMS
Select the OpenAMS unit type when prompted
Enter an AMS name or use the default (please make sure it starts with AMS) (default: AMS_1)
Configure your preferred options (tip forming, cutters, etc.)
Complete the installation
AFC installation prompts
Best practice: Install AFC while your OpenAMS unit is empty to avoid interruptions during the system file updates.

Stage AFC Configuration Templates
After the AFC installer completes, stage the OpenAMS-specific configuration files from this repository so they are in place before you run the OpenAMS installer:

cd ~/klipper_openams
cp AFC_AMS_1.cfg ~/printer_data/config/AFC/
cp AFC_Oams.cfg ~/printer_data/config/AFC/
Replace ~/printer_data with your actual printer data path if different.

Configuration file overview:

File	Purpose	Must edit?
AFC_AMS_1.cfg	Defines AFC lanes mapped to OpenAMS slots and hubs	Yes – configure lanes and T-number mappings
AFC_Oams.cfg	OpenAMS hardware configuration (MCU, sensors, FPS)	Yes – set CAN UUIDs and calibration values
Include the chosen files in your Klipper configuration stack (typically printer.cfg or split include files):

[include AFC/AFC_Oams.cfg]
The [oams_manager] and [oams ...] sections shown earlier come from AFC_Oams.cfg, while the lane definitions originate from AFC_AMS_1.cfg. Customize them before running the OpenAMS installer so the service restarts with your hardware details.

AFC Hardware Configuration Checklist
With the templates staged, edit the hardware-specific values before moving on:

AFC_Oams.cfg:
Set your CAN bus UUIDs or serial identifiers for FPS and OAMS MCU boards
Adjust retry settings and sensor thresholds as needed
Configure FPS (Filament Pressure Sensor) pin and settings
AFC_AMS_1.cfg (lane configuration):
Map each OpenAMS slot to a tool number (T0–T3)
Verify each lane specifies its unit (e.g., AMS_1:1) and hub
Set load_to_hub: False for OpenAMS lanes (required for AMS units; matches the AFC_AMS_1.cfg template)
Set LED indices if using LED indicators
Review the default lane template from this repo and tailor hub mappings and bowden lengths to your hardware
AFC_Hardware.cfg:
Open the hardware template installed by AFC: nano ~/printer_data/config/AFC/AFC_Hardware.cfg
In [AFC_extruder extruder], set pin_tool_start: to AMS_extruder when the Filament Pressure Sensor handles ramming
If you rely on a toolhead filament sensor instead, set pin_tool_start: to your actual sensor pin (e.g., ^PC0)
Verify your toolhead distances and speeds for reliable load/unload (see the AFC documentation toolhead settings page: https://www.armoredturtle.xyz/docs/afc-klipper-add-on/index.html):
[AFC_extruder extruder]
tool_stn: 92                    # Distance in mm from the toolhead sensor to the nozzle tip
tool_stn_unload: 110            # Distance to move in mm while unloading the toolhead
tool_sensor_after_extruder: 0   # Extra distance after sensors clear to fully exit gears
tool_unload_speed: 25           # Unload speed in mm/s
tool_load_speed: 15             # Load speed in mm/s
deadband: 1
Only after the AFC installer, template staging, and configuration edits are complete should you proceed to installing OpenAMS. Screenshot 2025-11-09 095729

2. Install OpenAMS
If this is your first time installing OpenAMS, use the provided installation script from the repository you cloned earlier:

cd ~/klipper_openams
./install-openams.sh
The installation script will:

Link OpenAMS Python modules (src/*.py) to Klipper's klippy/extras/ directory
Link utility scripts (scripts/*.py) to Klipper's scripts/ directory
Register the HDC1080 temperature sensor type in Klipper
Add an [update_manager openams] entry to moonraker.conf for git-based updates
Restart Klipper and Moonraker services
Switching to This Fork
If you already have OpenAMS installed and want to switch to this fork:

cd ~/klipper_openams
git remote add lindnjoe https://github.com/lindnjoe/klipper_openams 2>/dev/null \
  || git remote set-url lindnjoe https://github.com/lindnjoe/klipper_openams
git fetch lindnjoe master
git checkout -B lindnjoe-master lindnjoe/master
./install-openams.sh
The git checkout -B command creates a local lindnjoe-master branch that tracks this repository, allowing easy updates with git pull.

Custom Installation Paths
If your directory structure differs from the standard layout, configure the installation with additional parameters:

./install-openams.sh [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>]
Parameters:

-k : Path to Klipper installation (default: ~/klipper)
-s : Klipper service name (default: klipper)
-c : Configuration directory path (default: ~/printer_data/config)
Example:

./install-openams.sh -k /home/pi/klipper -c /home/pi/printer_data/config
Uninstalling
To remove the symlinks created by the installer:

cd ~/klipper_openams
./install-openams.sh -u
This removes the module and script symlinks from Klipper. You will still need to manually remove the [update_manager openams] section from moonraker.conf and any OpenAMS configuration sections from your Klipper config files.

3. Apply AFC Integration Files
OpenAMS relies on AFC integration helpers that are not bundled with the upstream AFC add-on yet. After completing the installations above, copy the updated integration files from this repository into your AFC add-on checkout:

cd ~/klipper_openams
cp AFC_OpenAMS.py ~/AFC-Klipper-Add-On/extras/
cp openams_integration.py ~/AFC-Klipper-Add-On/extras/
# Skip this overwrite if you already customized the staged file in Step 1
cp AFC_AMS_1.cfg ~/printer_data/config/AFC/
If you have edits in ~/printer_data/config/AFC/AFC_AMS_1.cfg, either merge them back in after this copy or omit the line above.

Re-run sudo service klipper restart (or your custom service name) to pick up the changes. This manual step will be removed once the AFC add-on ships these files.

Configuration
OpenAMS Manager Settings
The [oams_manager] section controls global OpenAMS behavior:

[oams_manager]
# Optional: Distance before toolhead to trigger reload during runout (default: 0.0)
# Set this to start loading the next filament before the current one fully runs out
# Useful for reducing print pauses during runout
reload_before_toolhead_distance: 0.0

# Optional: Clog detection sensitivity: low, medium, high (default: medium)
# Controls how aggressive the clog detection system is
clog_sensitivity: medium

# Optional: Enable/disable detection systems (default: True)
enable_clog_detection: True
enable_stuck_spool_detection: True

# Optional: F1S runout debounce duration (seconds)
# Defaults to AFC debounce_delay if available
debounce_delay: 0.0
Configuration Tips:

reload_before_toolhead_distance: Set this to a positive value to load replacement spool sooner. Helpful with longer ptfe lengths and faster printing speeds. May require manual tuning to get just right for your printer.
clog_sensitivity: Start with medium. Increase to high if clogs go undetected. Decrease to low if false positives occur.
enable_clog_detection / enable_stuck_spool_detection: Disable only when diagnosing issues or if you need to run without automated pauses.
debounce_delay: Increase if you see false runouts from noisy F1S sensors.
If you need a per-FPS override, set reload_before_toolhead_distance in the [fps ...] section for that sensor.

OAMS Hardware Settings
For each OAMS unit, configure retry behavior in your OAMS hardware configuration file (typically AFC_Oams.cfg):

[mcu oams_mcu1]
canbus_uuid: <your_unique_OAMS_MCU1_UUID>
# or: serial: /dev/serial/by-id/...

[oams oams1]
mcu: oams_mcu1

# Retry Configuration (all optional - defaults shown in comments)
load_retry_max: 3              # Maximum number of load retry attempts (default: 3)
unload_retry_max: 2            # Maximum number of unload retry attempts (default: 2)
retry_delay: 3.0               # Delay in seconds between retries (default: 3.0)
# Extra retract overlap before unload (default: -10.0)
extra_retract: -10.0
# auto_unload_on_failed_load: True  # Defaults to True - only set if you need False

# Optional load behavior
dock_load: True                # True = load from dock when supported
post_load_purge: 75            # mm to purge after a successful load

# Pressure sensor thresholds (required)
fps_upper_threshold: 0.7
fps_lower_threshold: 0.3
fps_is_reversed: true

# HES thresholds (required)
f1s_hes_on: 0.1, 0.1, 0.1, 0.1
f1s_hes_is_above: false
hub_hes_on: 0.85, 0.85, 0.85, 0.85
hub_hes_is_above: true

# Physical configuration (required)
ptfe_length: 0
oams_idx: 1

# Optional tuning
fps_target: 0.5
current_target: 0.30
current_kp: 3.0
current_ki: 0.0
current_kd: 0.0
Important Configuration Notes:

MCU UUIDs: You must update the [mcu ...] canbus_uuid (or serial) values with your actual hardware UUIDs. Find them with:

~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
or
ls /dev/serial/by-id/*  
Retry Settings: The defaults work well for most setups, but you may need to adjust:

Increase load_retry_max if filament occasionally fails to load on first attempt
Increase retry_delay if your hardware needs more recovery time
FPS Configuration
Each Filament Pressure Sensor is defined in AFC_Oams.cfg. The FPS reads an analog voltage via ADC and maps it to a 0.0–1.0 pressure range that the manager uses for clog, stuck spool, and engagement detection.

[fps fps1]
pin: fps:PA2                  # ADC pin on the FPS MCU
reversed: false               # Flip the 0→1 scale (set true if unloaded reads ~1.0)
oams: oams1                   # Comma-separated OAMS units this FPS serves
extruder: extruder            # Associated Klipper extruder

# Optional: per-FPS runout reload margin (mm). Overrides the global
# reload_before_toolhead_distance from [oams_manager] for this sensor.
#reload_before_toolhead_distance: 0.0

# Optional: set to true when running Kalico (Danger Klipper) instead of
# stock Klipper. Changes the ADC setup call to the Kalico variant.
#use_kalico: False
Multiple FPS / OAMS units: duplicate the [mcu ...], [oams ...], and [fps ...] sections with unique names and pins:

[mcu fps2]
canbus_uuid: <your_unique_FPS2_UUID>

[oams oams2]
mcu: oams_mcu2
# ... same settings as oams1, adjusted for your second unit ...

[fps fps2]
pin: fps2:PA2
reversed: false
oams: oams2
extruder: extruder
Retry Behavior
The OpenAMS system includes automatic retry logic for both load and unload operations to handle temporary failures gracefully:

Load Retries:

Default: 3 attempts
Monitors encoder movement during loading
Automatically unloads and retries if filament gets stuck
Only pauses the printer if all retry attempts fail
Unload Retries:

Default: 2 attempts
Monitors encoder movement during unloading
Aborts stuck operations and retries automatically
Only pauses the printer if all retry attempts fail
Clog Detection Settings
The clog_sensitivity setting in [oams_manager] controls how aggressive the clog detection is:

Sensitivity	Observation Window	Tolerance	Best For
low	48mm extrusion	More tolerant	Printers with flow variations, flexible materials
medium (default)	24mm extrusion	Balanced	General use, most materials
high	12mm extrusion	More sensitive	Quick clog detection, important prints
Tuning Tips:

Start with medium sensitivity
If you get false clog detections (pauses when no clog exists), lower to low
If clogs aren't detected quickly enough, increase to high
Consider your material: flexible filaments may need low sensitivity
Advanced Detection Tunables
Fine-tune pressure-based detection in [oams_manager] when calibrating new hardware or dialing in tricky materials:

[oams_manager]
# Delay before stuck spool checks begin after a load starts (default: 8.0s)
stuck_spool_load_grace: 8.0

# Max stuck spool retry attempts before pause (default: 2)
stuck_spool_max_attempts: 2

# Pressure thresholds for stuck spool detection (defaults: 0.08 / 0.12)
stuck_spool_pressure_threshold: 0.08
stuck_spool_pressure_clear_threshold: 0.12  # Must be greater than threshold

# Target FPS pressure for clog detection (default: 0.50)
clog_pressure_target: 0.50

# Seconds to monitor pressure after a load before declaring success (default: 15.0)
post_load_pressure_dwell: 15.0

# FPS level above which a load is treated as stuck (default: 0.75)
load_fps_stuck_threshold: 0.75

# Pressure drop threshold to confirm engagement during post-load verification (default: 0.6)
engagement_pressure_threshold: 0.6
Tuning guidance:

Increase stuck_spool_load_grace if sensitive sensors flag issues during the first seconds of a load.
Raise stuck_spool_max_attempts if you want the manager to retry more before pausing.
Raise stuck_spool_pressure_threshold or load_fps_stuck_threshold if reliable hardware still reports premature “stuck” errors.
Lower clog_pressure_target or shorten post_load_pressure_dwell if high-flow materials routinely trigger clog detection.
Optional Features
Mainsail AFC Panel
Enable the optional Mainsail AFC panel for easy lane management and status monitoring.

Installation:

Backup your existing Mainsail installation:
cd ~/mainsail
tar -czf ~/mainsail-backup-$(date +%Y%m%d).tar.gz .
Extract the included panel files:
cd ~/klipper_openams
unzip -o mainsail.zip -d ~/mainsail/
Clear your browser cache and reload Mainsail
Features:

Visual lane status display
Quick lane selection
Runout configuration interface
Spool management integration
Initial Calibration
After completing the OpenAMS and AFC installation, calibrate each OpenAMS unit to ensure accurate filament detection and optimal performance.

Before calibrating, review the AFC [AFC] section and macro order of operation details in the Armored Turtle documentation to configure load/unload flows, cutting, and waste management options (poop, wipe, kick, park, and tip-form). See the AFC configuration guide: https://www.armoredturtle.xyz/docs/afc-klipper-add-on/configuration/AFC.cfg.html#afc-section.

# - Load               |   - Unload
#   - Load Filament    |    - Cut
#   - Poop             |    - Park
#   - Wipe             |    or
#   - Kick             |    - Park
#   - Wipe             |    - Tip Form
#   - Print            |

# TOOL Cutting Settings
tool_cut: True                  
#    Boolean, when set to true a toolhead cutter will be utilized.
tool_cut_cmd: AFC_CUT
#    Default: AFC_CUT           
#    Macro name to call when cutting filament. Using the default AFC_CUT macro
#    will call the macro defined in `Cut.cfg`. You can replace this with a 
#    custom macro name if you have a different cutting method or tool.
tool_cut_threshold: 10000
#    A warning will print out 1,000 cuts before the threshold is hit. Once this
#    threshold is hit, a message is displayed as an error notifying that the cut
#    threshold for the current blade has exceeded this threshold.

# Park Settings
park: True                      
#    Boolean, when set to true, the the park functionality will be enabled.
park_cmd: AFC_PARK              
#    Default: AFC_PARK
#    Macro name to call when parking the toolhead. Using the default AFC_PARK
#    macro will call the macro defined in `Park.cfg`. You can replace this with
#    a custom macro name if you have a different parking method or tool.

# Poop Settings
poop: True                      
#    Boolean, when set to true, the system will use the `poop` method for 
#    purging filament after a color change.
poop_cmd: AFC_POOP              
#    Default: AFC_POOP
#    Macro name to call when pooping filament. Using the default AFC_POOP macro
#    will call the macro defined in `Poop.cfg`. You can replace this with a 
#    custom macro name if you have a different pooping method or tool.
#    Please note that the only valid parameter for the AFC_POOP macro is
#    `purge_length`, which defines the length of filament to purge.

# Kick Settings
kick: True                      
#    Boolean, when set to true, the system will use enable the `kick` 
#    functionality to clear purged filament from the bed.
kick_cmd: AFC_KICK              
#    Default: AFC_Kick
#    Macro name to call when wiping filament. Using the default AFC_KICK macro
#    will call the macro defined in `Brush.cfg`. You can replace this with a 
#    custom macro name if you have a different wiping method or tool.

# Wipe Settings
wipe: True                      
#    Boolean, when set to true, the system will use a wiper to help clean the 
#    toolhead.
wipe_cmd: AFC_BRUSH
#    Default: AFC_BRUSH
#    Macro name to call when wiping filament. Using the default AFC_BRUSH macro
#    will call the macro defined in `Brush.cfg`. You can replace this with a 
#    custom macro name if you have a different wiping method or tool.

# Form Tip Settings
form_tip: False                 
#    Boolean, when set to true, the system will use a form tip macro to help 
#    shape the filament tip for better loading / unloading.
form_tip_cmd: AFC               
#    Default: AFC
#    Macro name to call when using tip-forming. Using the default AFC macro will
#    call the built-in macro. You can replace this with a custom macro name if 
#    you have a different tip-forming method or tool. Configuration for the AFC 
#    macro is defined in the `AFC.cfg` file.```


**Calibration Process:**

1. Ensure your AMS unit has at least one spool loaded

2. Open the OpenAMS calibration menu via the Klipper console:
UNIT_PTFE_CALIBRATION UNIT=AMS_1


3. Use the prompts to run:
- **PTFE length calibration** for each loaded lane
- **HUB HES calibration** for each loaded lane (or all lanes)

You can also run the commands directly:
AFC_OAMS_CALIBRATE_PTFE UNIT=AMS_1 SPOOL=<spool_index>

AFC_OAMS_CALIBRATE_HUB_HES UNIT=AMS_1 SPOOL=<spool_index>

AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT=AMS_1


4. Once calibration completes, restart Klipper to load the new settings:
FIRMWARE_RESTART


**Repeat this process for each OpenAMS unit** in your system. Proper calibration ensures:
- Reliable lane detection
- Accurate filament position tracking
- Prevention of loading errors during multi-material prints

**Troubleshooting Calibration:**
- Ensure filament is loaded in the lane being calibrated
- Verify encoder (hub) is clean and functioning
- Check that tube lengths are within reasonable bounds (50-2000mm typical)

## Infinite Spooling

Infinite spooling allows automatic switching between lanes when a spool runs out, enabling continuous printing without manual intervention. OpenAMS now integrates with AFC's lane-based runout system for seamless operation.

**Key Features:**
- Integrated with AFC lane runout configuration
- OpenAMS detects runout and notifies AFC to load the next lane
- Supports infinite spooling between lanes on the same extruder/FPS
- Automatic lane switching based on AFC runout settings
- Can be configured via AFC console commands or Mainsail AFC panel

**How It Works:**
1. OpenAMS monitors the FPS (Filament Pressure Sensor) and hub sensors
2. When a spool runs empty, OpenAMS detects the runout event
3. OpenAMS notifies AFC about the runout
4. AFC handles the lane switching using its configured runout mappings
5. The new lane is automatically loaded and printing continues

**Cross-extruder runouts:** If the runout target lives on a different extruder/FPS, OpenAMS flags a cross-extruder runout and AFC performs a full toolchange sequence (matching Box Turtle behavior). Same-FPS runouts continue to use OpenAMS' coasting and reload handling.

**Configuration:**

Runout lanes are configured through AFC using the `SET_RUNOUT` command:

**Method 1: Klipper Console (AFC Command)**
SET_RUNOUT LANE=lane# RUNOUT=lane#


Example: Set lane0 to use lane1 as runout backup:
SET_RUNOUT LANE=lane0 RUNOUT=lane1


**Method 2: AFC Panel (Mainsail)**
1. Navigate to the AFC panel in Mainsail
2. Select the tool/lane
3. Configure the runout lane in the AFC interface

**Multi-Lane Chains:**

You can create chains of runouts for extended printing by configuring multiple mappings:
SET_RUNOUT LANE=lane0 RUNOUT=lane1 SET_RUNOUT LANE=lane1 RUNOUT=lane2 SET_RUNOUT LANE=lane2 RUNOUT=lane3


**Material Matching:**

For best results:
- Assign lanes with the same material type and color to the same tool
- Use Spoolman to track filament properties across lanes
- Update spool weights regularly for accurate runout detection
- Configure appropriate runout chains through AFC

**Note:** This system replaces the previous filament group configuration. All runout handling now goes through AFC's lane system for better integration and flexibility.

## Troubleshooting

### Stuck Spool Detection

If you experience issues with stuck spool detection during load or unload operations:

**Check retry configuration:**
```ini
[oams oams1]
load_retry_max: 3         # Try 3 times before giving up
unload_retry_max: 2       # Try 2 times before giving up
retry_delay: 3.0          # Delay between retry attempts
Verify retry behavior:

Monitor Klipper logs: tail -f ~/printer_data/logs/klippy.log
Look for messages like "letting retry logic handle it" and "retry X/Y"
Retries should happen automatically before pausing
Common solutions:

Increase retry counts: Set load_retry_max: 5 if filament occasionally needs extra attempts
Check encoder: Clean the OAMS encoder wheel and verify it rotates freely
Verify filament path: Ensure PTFE tubes are not kinked or obstructed
CAN Bus Issues
Symptoms:

OpenAMS units not detected
Intermittent connection losses
Calibration failures
Solutions:

Verify CAN termination resistors (120Ω at each end of the bus)
Check CAN bus speed matches across all devices (typically 500000 or 1000000)
Query CAN devices:
~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
Check Klipper logs for CAN timeout errors
Verify wiring: Ensure CAN_H and CAN_L are not swapped
CAN Bus Debugging
The repository includes a real-time CAN bus log viewer (scripts/canbus_logger.py) that decodes OAMS firmware log messages. This is useful for diagnosing hardware-level issues that don't surface in klippy.log.

# After installation the script is linked into ~/klipper/scripts/
python ~/klipper/scripts/canbus_logger.py can0
The logger filters CAN frames from the OAMS logging address (0x780), decodes the AMS index and log severity, and displays color-coded output:

Color	Level
Red	FATAL
Yellow	ERROR
Green	WARNING
Blue	INFO
Gray	DEBUG
Requirements: the python-can and termcolor packages must be installed in the environment running the script (not required for normal Klipper operation).

Clog Detection False Positives
If the printer pauses due to false clog detection:

Lower sensitivity:

[oams_manager]
clog_sensitivity: low
Check for actual flow issues:

Partial nozzle clogs
Extruder tension too tight/loose
Filament diameter variations
Verify encoder function:

Clean encoder wheel
Check encoder wiring
LED Issues
LEDs not changing color:

Verify LED index configuration in AFC_AMS1.cfg:

[AFC_lane lane0]
led_index: AFC_indicator:1
Check LED strip configuration

Test LEDs directly:

SET_LED LED=AFC_indicator INDEX=1 RED=1.0 GREEN=0 BLUE=0
Filament Loading Failures
Filament won't load to toolhead:

Check AFC_Hardware.cfg pin configuration:

[AFC_extruder extruder]
pin_tool_start: AMS_extruder  # or your sensor pin
Verify bowden length calibration:

Run OpenAMS PTFE calibration for each unit:
AFC_OAMS_CALIBRATE_PTFE UNIT=AMS_1 SPOOL=<spool_index>
Check that measured lengths are reasonable
Adjust afc_bowden_length in hub configuration if needed
Test individual components:

Manually extrude filament to verify extruder works
Check that toolhead sensor triggers correctly
Verify PTFE tube path is clear
Increase retry attempts temporarily:

[oams oams1]
load_retry_max: 5
Configuration File Errors
Klipper won't start after configuration changes:

Check klippy.log for specific error messages:

tail -50 ~/printer_data/logs/klippy.log
Common issues:

Missing or incorrect CAN UUIDs
Duplicate section names
Invalid pin names
Syntax errors in configuration files
Test configuration syntax:

~/klippy-env/bin/python ~/klipper/klippy/klippy.py ~/printer_data/config/printer.cfg -d ~/printer_data/klipper.dict -l /tmp/test.log
Getting Help
If you're still experiencing issues:

Check Klipper logs: ~/printer_data/logs/klippy.log

Enable debug logging for OpenAMS components

Gather information:

Klipper version
AFC version
OpenAMS fork commit hash: git -C ~/klipper_openams rev-parse HEAD
Full error messages from logs
Configuration files
Ask for help:

GitHub Issues: https://github.com/lindnjoe/klipper_openams/issues
Credits
This project was created by knight.rad_iant and Armored Turtle Team on Discord.

Based on the original OpenAMS project with enhancements for AFC integration, retry logic, and clog detection.
