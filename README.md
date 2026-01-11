<img width="1024" height="1024" alt="image" src="https://github.com/user-attachments/assets/7b515408-d0b3-437f-b0a4-8c7128d2e922" />



# OpenAMS for Klipper

A Klipper integration for OpenAMS that enables multi-material printing with automatic filament management, runout detection, and intelligent retry logic.

## Table of Contents

- [Overview](#overview)
- [What's New](#whats-new)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Install/Update AFC](#1-installupdate-afc)
    - [Lane Architecture Primer](#lane-architecture-primer)
    - [Install the AFC Add-On](#install-the-afc-add-on)
    - [Stage AFC Configuration Templates](#stage-afc-configuration-templates)
    - [AFC Hardware Configuration Checklist](#afc-hardware-configuration-checklist)
  - [Install OpenAMS](#2-install-openams)
    - [Switching to This Fork](#switching-to-this-fork)
    - [Custom Installation Paths](#custom-installation-paths)
  - [Apply AFC Integration Files](#3-apply-afc-integration-files)
- [Configuration](#configuration)
  - [OpenAMS Manager Settings](#openams-manager-settings)
  - [OAMS Hardware Settings](#oams-hardware-settings)
  - [Retry Behavior](#retry-behavior)
  - [Clog Detection Settings](#clog-detection-settings)
  - [Advanced Detection Tunables](#advanced-detection-tunables)
- [Optional Features](#optional-features)
  - [Mainsail AFC Panel](#mainsail-afc-panel)
- [Initial Calibration](#initial-calibration)
- [Infinite Spooling](#infinite-spooling)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)

## Overview

OpenAMS provides automated filament handling for Klipper-based 3D printers. This fork integrates tightly with Armored Turtle's AFC (Automatic Filament Changer) add-on using a **lane-based architecture**. The combination delivers end-to-end multi-material automation—from AFC's physical filament routing to OpenAMS' retry logic, runout handling, and print-state awareness.

### Full Integration at a Glance

- **AFC** exposes lanes, runout sensors, and hub LEDs.
- **OpenAMS** maps those AFC lanes into the AMS manager, applies retry and clog detection logic, and keeps Moonraker/Klipper informed of state changes.
- **Optional services** like Spoolman enrich the integration with live spool metadata.

### Example Settings

The snippets below show how the integration pieces fit together. Adjust lane names and MCU UUIDs to match your hardware.

<details>
<summary><strong>OpenAMS Manager</strong></summary>

```ini
[oams_manager]
# Optional: start loading replacement filament early
reload_before_toolhead_distance: 0.0

# Optional: lane-wide clog sensitivity (low/medium/high)
clog_sensitivity: medium
```

</details>

<details>
<summary><strong>AFC Lane Mapping</strong></summary>

```ini
[AFC_lane lane0]
unit: AMS_1:1
hub: Hub_1
map: T0

[AFC_lane lane1]
unit: AMS_1:2
hub: Hub_2
map: T1
```

</details>

<details>
<summary><strong>OAMS Retry Settings</strong></summary>

```ini
[oams oams1]
mcu: oams_mcu1
load_retry_max: 3
unload_retry_max: 2
retry_backoff_base: 1.0
retry_backoff_max: 5.0
```

</details>

These configuration blocks are sourced from the repository templates—`AFC_Oams.cfg` contains the `[oams_manager]` and `[oams ...]` sections while `AFC_AMS_1.cfg` defines the `[AFC_lane ...]` entries—and reference the synced AFC extras installed in later steps.

### Key Capabilities
- Lane-based filament management through AFC integration
- Automatic filament loading and unloading with pressure sensing
- Intelligent retry logic for stuck filament detection
- Clog detection with configurable sensitivity
- Event-driven sensor monitoring for optimal performance
- Runout detection and automatic lane switching
- Spoolman integration for filament tracking
- LED status indicators

## What's New

**Recent Major Updates:**

### Lane-Based Architecture (Current Version)
The system has transitioned from filament groups to a lane-based architecture for better AFC integration:

- **AFC Lanes**: Each OpenAMS slot is now configured as an AFC lane with independent settings
- **Event-Driven Sensors**: Sensor monitoring switched from polling to event-based for better performance
- **AFC Runout Integration**: Runout handling now integrates directly with AFC's lane system
- **Hub Mapping**: Each lane is mapped to its own hub for visual indication and broke filament detection scenarios
- **Detection Tuning**: New `[oams_manager]` options let you adjust pressure thresholds and dwell windows for stuck spool and clog detection

**Migration Notes:**
- If upgrading from an older version, your filament group configuration will need to be converted to AFC lanes
- Macro calls have changed from `GROUP=T0` to `LANE=lane0` format
- Runout configuration now uses AFC's runout mapping command (`SET_RUNOUT`) instead of OpenAMS filament groups
- See the [Install/Update AFC](#1-installupdate-afc) section for detailed configuration examples

## Features

- **Lane-Based Architecture**: Integration with AFC lanes for flexible spool configuration and mapping
- **Event-Driven Sensors**: Efficient event-based monitoring instead of constant polling for better performance
- **Automatic Retry Logic**: Configurable retry attempts for both load and unload operations with exponential backoff
- **Clog Detection**: Three sensitivity levels (low, medium, high) to detect filament clogs during printing
- **Runout Handling**: Automatic filament runout detection integrated with AFC lane system
- **Infinite Spooling**: Seamless lane switching for continuous printing using AFC runout configuration
- **LED Status Indicators**: Visual feedback through lane LEDs
- **HDC1080 Sensor Support**: Temperature and humidity monitoring within the AMS unit

## Prerequisites

Before installing OpenAMS, ensure you have:

1. **Klipper** installed and running
2. **Moonraker** configured
3. **MCU's id's** uuid's or serial id for FPS and AMS board
4. **Optional but recommended:**
   - Spoolman for filament tracking
   - Mainsail or Fluidd web interface

## Installation

Follow the steps below in order to ensure a working AFC + OpenAMS setup. Each stage builds on the previous one.

### Clone This Repository First

The AFC installer will copy OpenAMS-specific files during setup, so start by cloning this repository:

```bash
cd ~
git clone https://github.com/lindnjoe/klipper_openams.git
cd klipper_openams
```

Leave the repository in place for the later steps—you'll stage configuration templates from it and then run the OpenAMS installer.

### 1. Install/Update AFC

This step lays down the Armored Turtle AFC framework that OpenAMS builds on. Complete each subsection before moving to the OpenAMS installer.

#### Lane Architecture Primer

OpenAMS integrates with AFC through **lanes** instead of the legacy filament group system. Each lane represents:

- One physical spool slot on your OpenAMS unit
- A mapping to a specific hub for filament routing
- Optional LED indicators
- Association with a tool number (T0, T1, T2, T3, etc.)

**Benefits of lane-based configuration:**

- More flexible spool-to-tool mappings
- Better integration with AFC's toolchange system
- Easier to configure runout behavior per lane
- Supports multiple hubs and complex routing

**Example:** A single OpenAMS with four slots becomes four AFC lanes (`lane0`–`lane3`), each independently configurable with its own hub and runout behavior.

#### Install the AFC Add-On

Clone the Armored Turtle repository (OpenAMS should already be cloned from the previous step):

```bash
cd ~
git clone https://github.com/ArmoredTurtle/AFC-Klipper-Add-On.git
```

Install the AFC add-on from the `multi_extruder` branch:

```bash
cd ~/AFC-Klipper-Add-On
./install-afc.sh -b multi_extruder
```

**Important setup notes:**

1. **Read the AFC documentation:** Most of the Armored Turtle setup guide applies directly to OpenAMS. Review it at <https://www.armoredturtle.xyz/docs/afc-klipper-add-on/index.html>.
2. **Box Turtle vs OpenAMS:** Some documentation is specific to Box Turtle hardware and will not apply to OpenAMS. When in doubt, leave defaults in place—the OpenAMS-specific config files override Box Turtle defaults.
3. **Installation prompts:** During the AFC installation, you'll be presented with an interactive menu:
   - Press **T** to cycle through installation types until you see **OpenAMS**
   - Select the **OpenAMS** unit type when prompted
   - Enter an AMS name or use the default (please make sure it starts with AMS) (default: `AMS_1`) 
   - Configure your preferred options (tip forming, cutters, etc.)
   - Complete the installation
   
   <img width="1086" height="803" alt="AFC installation prompts" src="https://github.com/user-attachments/assets/7b62feea-d566-4be5-9d44-5b79644fc841" />

4. **Best practice:** Install AFC while your OpenAMS unit is **empty** to avoid interruptions during the system file updates.

#### Stage AFC Configuration Templates

After the AFC installer completes, stage the OpenAMS-specific configuration files from this repository so they are in place before you run the OpenAMS installer:

```bash
cd ~/klipper_openams
cp AFC_AMS_1.cfg ~/printer_data/config/AFC/
cp AFC_Oams.cfg ~/printer_data/config/AFC/
```

Replace `~/printer_data` with your actual printer data path if different.

**Configuration file overview:**

| File | Purpose | Must edit? |
|------|---------|------------|
| `AFC_AMS_1.cfg` | Defines AFC lanes mapped to OpenAMS slots and hubs | Yes – configure lanes and T-number mappings |
| `AFC_Oams.cfg` | OpenAMS hardware configuration (MCU, sensors, FPS) | Yes – set CAN UUIDs and calibration values |

Include the chosen files in your Klipper configuration stack (typically `printer.cfg` or split include files):

```ini
[include AFC/AFC_Oams.cfg]
```

The `[oams_manager]` and `[oams ...]` sections shown earlier come from `AFC_Oams.cfg`, while the lane definitions originate from `AFC_AMS_1.cfg`. Customize them before running the OpenAMS installer so the service restarts with your hardware details.

#### AFC Hardware Configuration Checklist

With the templates staged, edit the hardware-specific values before moving on:

1. **AFC_Oams.cfg:**
   - Set your CAN bus UUIDs or serial identifiers for FPS and OAMS MCU boards
   - Adjust retry settings and sensor thresholds as needed
   - Configure FPS (Filament Pressure Sensor) pin and settings
2. **AFC_AMS_1.cfg (lane configuration):**
   - Map each OpenAMS slot to a tool number (T0–T3)
   - Verify each lane specifies its `unit` (e.g., `AMS_1:1`) and hub
   - Set `load_to_hub: False` for OpenAMS lanes (required for AMS units; matches the AFC_AMS_1.cfg template)
   - Set LED indices if using LED indicators
   - Review the default lane template from this repo and tailor hub mappings and bowden lengths to your hardware
3. **AFC_Hardware.cfg:**
   - Open the hardware template installed by AFC: `nano ~/printer_data/config/AFC/AFC_Hardware.cfg`
   - In `[AFC_extruder extruder]`, set `pin_tool_start:` to `AMS_extruder` when the Filament Pressure Sensor handles ramming
   - If you rely on a toolhead filament sensor instead, set `pin_tool_start:` to your actual sensor pin (e.g., `^PC0`)
   - Verify your toolhead distances and speeds for reliable load/unload (see the AFC documentation toolhead settings page: <https://www.armoredturtle.xyz/docs/afc-klipper-add-on/index.html>):
     ```ini
     [AFC_extruder extruder]
     tool_stn: 92                    # Distance in mm from the toolhead sensor to the nozzle tip
     tool_stn_unload: 110            # Distance to move in mm while unloading the toolhead
     tool_sensor_after_extruder: 0   # Extra distance after sensors clear to fully exit gears
     tool_unload_speed: 25           # Unload speed in mm/s
     tool_load_speed: 15             # Load speed in mm/s
     deadband: 1
     ```

Only after the AFC installer, template staging, and configuration edits are complete should you proceed to installing OpenAMS.
<img width="512" height="512" alt="image" src="https://github.com/user-attachments/assets/093877d5-ff56-4837-9f38-0e878ae5180b" />

### 2. Install OpenAMS

If this is your first time installing OpenAMS, use the provided installation script from the repository you cloned earlier:

```bash
cd ~/klipper_openams
./install-openams.sh
```

The installation script will:
1. Link OpenAMS Python modules to Klipper's extras directory
2. Add HDC1080 temperature sensor support
3. Configure Moonraker update manager
4. Restart Klipper services

#### Switching to This Fork

If you already have OpenAMS installed and want to switch to this fork:

```bash
cd ~/klipper_openams
git remote add lindnjoe https://github.com/lindnjoe/klipper_openams 2>/dev/null \
  || git remote set-url lindnjoe https://github.com/lindnjoe/klipper_openams
git fetch lindnjoe master
git checkout -B lindnjoe-master lindnjoe/master
./install-openams.sh
```

The `git checkout -B` command creates a local `lindnjoe-master` branch that tracks this repository, allowing easy updates with `git pull`.

#### Custom Installation Paths

If your directory structure differs from the standard layout, configure the installation with additional parameters:

```bash
./install-openams.sh [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>]
```

**Parameters:**
- `-k` : Path to Klipper installation (default: `~/klipper`)
- `-s` : Klipper service name (default: `klipper`)
- `-c` : Configuration directory path (default: `~/printer_data/config`)

**Example:**
```bash
./install-openams.sh -k /home/pi/klipper -c /home/pi/printer_data/config
```

### 3. Apply AFC Integration Files

OpenAMS relies on AFC integration helpers that are not bundled with the upstream AFC add-on yet. After completing the installations above, copy the updated integration files from this repository into your AFC add-on checkout:

```bash
cd ~/klipper_openams
cp AFC_OpenAMS.py ~/AFC-Klipper-Add-On/extras/
cp openams_integration.py ~/AFC-Klipper-Add-On/extras/
# Skip this overwrite if you already customized the staged file in Step 1
cp AFC_AMS_1.cfg ~/printer_data/config/AFC/
```

If you have edits in `~/printer_data/config/AFC/AFC_AMS_1.cfg`, either merge them back in after this copy or omit the line above.

Re-run `sudo service klipper restart` (or your custom service name) to pick up the changes. This manual step will be removed once the AFC add-on ships these files.

## Configuration

### OpenAMS Manager Settings

The `[oams_manager]` section controls global OpenAMS behavior:

```ini
[oams_manager]
# Optional: Distance before toolhead to trigger reload during runout (default: 0.0)
# Set this to start loading the next filament before the current one fully runs out
# Useful for reducing print pauses during runout
reload_before_toolhead_distance: 0.0

# Optional: Clog detection sensitivity: low, medium, high (default: medium)
# Controls how aggressive the clog detection system is
clog_sensitivity: medium

# Optional: F1S runout debounce duration (seconds)
# Defaults to AFC debounce_delay if available
debounce_delay: 0.0
```

**Configuration Tips:**
- **reload_before_toolhead_distance**: Set this to a positive value to load replacement spool sooner. Helpful with longer ptfe lengths and faster printing speeds. May require manual tuning to get just right for your printer.
- **clog_sensitivity**: Start with `medium`. Increase to `high` if clogs go undetected. Decrease to `low` if false positives occur.
- **debounce_delay**: Increase if you see false runouts from noisy F1S sensors.

If you need a per-FPS override, set `reload_before_toolhead_distance` in the `[fps ...]` section for that sensor.

### OAMS Hardware Settings

For each OAMS unit, configure retry behavior in your OAMS hardware configuration file (typically `AFC_Oams.cfg`):

```ini
[mcu oams_mcu1]
canbus_uuid: <your_unique_OAMS_MCU1_UUID>
# or: serial: /dev/serial/by-id/...

[oams oams1]
mcu: oams_mcu1

# Retry Configuration (all optional - defaults shown in comments)
load_retry_max: 3              # Maximum number of load retry attempts (default: 3)
unload_retry_max: 2            # Maximum number of unload retry attempts (default: 2)
retry_backoff_base: 1.0        # Base delay in seconds between retries (default: 1.0)
retry_backoff_max: 5.0         # Maximum delay between retries (default: 5.0)
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
```

**Important Configuration Notes:**

1. **MCU UUIDs**: You must update the `[mcu ...]` `canbus_uuid` (or `serial`) values with your actual hardware UUIDs. Find them with:
   ```bash
   ~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
   or
   ls /dev/serial/by-id/*  
   ```

2. **Retry Settings**: The defaults work well for most setups, but you may need to adjust:
   - Increase `load_retry_max` if filament occasionally fails to load on first attempt
   - Increase `retry_backoff_base` if your hardware needs more recovery time
  

### Retry Behavior

The OpenAMS system includes automatic retry logic for both load and unload operations to handle temporary failures gracefully:

**Load Retries:**
- Default: 3 attempts 
- Monitors encoder movement during loading
- Automatically unloads and retries if filament gets stuck
- Only pauses the printer if all retry attempts fail

**Unload Retries:**
- Default: 2 attempts 
- Monitors encoder movement during unloading
- Aborts stuck operations and retries automatically
- Only pauses the printer if all retry attempts fail


### Clog Detection Settings

The `clog_sensitivity` setting in `[oams_manager]` controls how aggressive the clog detection is:

| Sensitivity | Observation Window | Tolerance | Best For |
|-------------|-------------------|-----------|----------|
| **low** | 48mm extrusion | More tolerant | Printers with flow variations, flexible materials |
| **medium** (default) | 24mm extrusion | Balanced | General use, most materials |
| **high** | 12mm extrusion | More sensitive | Quick clog detection, important prints |

**Tuning Tips:**
- Start with `medium` sensitivity
- If you get false clog detections (pauses when no clog exists), lower to `low`
- If clogs aren't detected quickly enough, increase to `high`
- Consider your material: flexible filaments may need `low` sensitivity

### Advanced Detection Tunables

Fine-tune pressure-based detection in `[oams_manager]` when calibrating new hardware or dialing in tricky materials:

```ini
[oams_manager]
# Delay before stuck spool checks begin after a load starts (default: 8.0s)
stuck_spool_load_grace: 8.0

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

# Default preretract distance (mm) applied before unloads (default: -10.0)
preretract: -10.0
```

**Tuning guidance:**
- Increase `stuck_spool_load_grace` if sensitive sensors flag issues during the first seconds of a load.
- Raise `stuck_spool_pressure_threshold` or `load_fps_stuck_threshold` if reliable hardware still reports premature “stuck” errors.
- Lower `clog_pressure_target` or shorten `post_load_pressure_dwell` if high-flow materials routinely trigger clog detection.

## Optional Features


### Mainsail AFC Panel

Enable the optional Mainsail AFC panel for easy lane management and status monitoring.

**Installation:**

1. Backup your existing Mainsail installation:

```bash
cd ~/mainsail
tar -czf ~/mainsail-backup-$(date +%Y%m%d).tar.gz .
```

2. Extract the included panel files:

```bash
cd ~/klipper_openams
unzip -o mainsail.zip -d ~/mainsail/
```

3. Clear your browser cache and reload Mainsail

**Features:**
- Visual lane status display
- Quick lane selection
- Runout configuration interface
- Spool management integration

## Initial Calibration

After completing the OpenAMS and AFC installation, calibrate each OpenAMS unit to ensure accurate filament detection and optimal performance.

Before calibrating, review the AFC `[AFC]` section and macro order of operation details in the Armored Turtle documentation to configure load/unload flows, cutting, and waste management options (poop, wipe, kick, park, and tip-form). See the AFC configuration guide: <https://www.armoredturtle.xyz/docs/afc-klipper-add-on/configuration/AFC.cfg.html#afc-section>.

```# Macro order of operation
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
   ```
   UNIT_PTFE_CALIBRATION UNIT=AMS_1
   ```

3. Use the prompts to run:
   - **PTFE length calibration** for each loaded lane
   - **HUB HES calibration** for each loaded lane (or all lanes)

   You can also run the commands directly:
   ```
   AFC_OAMS_CALIBRATE_PTFE UNIT=AMS_1 SPOOL=<spool_index>
   
   AFC_OAMS_CALIBRATE_HUB_HES UNIT=AMS_1 SPOOL=<spool_index>
   
   AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT=AMS_1
   ```

4. Once calibration completes, restart Klipper to load the new settings:
   ```
   FIRMWARE_RESTART
   ```

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
```
SET_RUNOUT LANE=lane# RUNOUT=lane#
```

Example: Set lane0 to use lane1 as runout backup:
```
SET_RUNOUT LANE=lane0 RUNOUT=lane1
```

**Method 2: AFC Panel (Mainsail)**
1. Navigate to the AFC panel in Mainsail
2. Select the tool/lane
3. Configure the runout lane in the AFC interface

**Multi-Lane Chains:**

You can create chains of runouts for extended printing by configuring multiple mappings:
```
SET_RUNOUT LANE=lane0 RUNOUT=lane1
SET_RUNOUT LANE=lane1 RUNOUT=lane2
SET_RUNOUT LANE=lane2 RUNOUT=lane3
```

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
retry_backoff_base: 1.0   # Wait 1s, 2s, 3s between attempts
```

**Verify retry behavior:**
- Monitor Klipper logs: `tail -f ~/printer_data/logs/klippy.log`
- Look for messages like "letting retry logic handle it" and "retry X/Y"
- Retries should happen automatically before pausing

**Common solutions:**
- **Increase retry counts**: Set `load_retry_max: 5` if filament occasionally needs extra attempts
- **Check encoder**: Clean the OAMS encoder wheel and verify it rotates freely
- **Verify filament path**: Ensure PTFE tubes are not kinked or obstructed

### CAN Bus Issues

**Symptoms:**
- OpenAMS units not detected
- Intermittent connection losses
- Calibration failures

**Solutions:**

1. **Verify CAN termination resistors** (120Ω at each end of the bus)
2. **Check CAN bus speed** matches across all devices (typically 500000 or 1000000)
3. **Query CAN devices:**
   ```bash
   ~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
   ```
4. **Check Klipper logs** for CAN timeout errors
5. **Verify wiring:** Ensure CAN_H and CAN_L are not swapped

### Clog Detection False Positives

If the printer pauses due to false clog detection:

1. **Lower sensitivity:**
   ```ini
   [oams_manager]
   clog_sensitivity: low
   ```

2. **Check for actual flow issues:**
   - Partial nozzle clogs
   - Extruder tension too tight/loose
   - Filament diameter variations

3. **Verify encoder function:**
   - Clean encoder wheel
   - Check encoder wiring
 
### LED Issues

**LEDs not changing color:**

1. **Verify LED index configuration** in `AFC_AMS1.cfg`:
   ```ini
   [AFC_lane lane0]
   led_index: AFC_indicator:1
   ```

2. **Check LED strip configuration** 
3. **Test LEDs directly:**
   ```
   SET_LED LED=AFC_indicator INDEX=1 RED=1.0 GREEN=0 BLUE=0
   ```

### Filament Loading Failures

**Filament won't load to toolhead:**

1. **Check AFC_Hardware.cfg pin configuration:**
   ```ini
   [AFC_extruder extruder]
   pin_tool_start: AMS_extruder  # or your sensor pin
   ```

2. **Verify bowden length calibration:**
   - Run OpenAMS PTFE calibration for each unit:
     ```
     AFC_OAMS_CALIBRATE_PTFE UNIT=AMS_1 SPOOL=<spool_index>
     ```
   - Check that measured lengths are reasonable
   - Adjust `afc_bowden_length` in hub configuration if needed

3. **Test individual components:**
   - Manually extrude filament to verify extruder works
   - Check that toolhead sensor triggers correctly
   - Verify PTFE tube path is clear

4. **Increase retry attempts temporarily:**
   ```ini
   [oams oams1]
   load_retry_max: 5
   ```

### Configuration File Errors

**Klipper won't start after configuration changes:**

1. **Check klippy.log** for specific error messages:
   ```bash
   tail -50 ~/printer_data/logs/klippy.log
   ```

2. **Common issues:**
   - Missing or incorrect CAN UUIDs
   - Duplicate section names
   - Invalid pin names
   - Syntax errors in configuration files

3. **Test configuration syntax:**
   ```bash
   ~/klippy-env/bin/python ~/klipper/klippy/klippy.py ~/printer_data/config/printer.cfg -d ~/printer_data/klipper.dict -l /tmp/test.log
   ```

### Getting Help

If you're still experiencing issues:

1. **Check Klipper logs:** `~/printer_data/logs/klippy.log`
2. **Enable debug logging** for OpenAMS components
3. **Gather information:**
   - Klipper version
   - AFC version
   - OpenAMS fork commit hash: `git -C ~/klipper_openams rev-parse HEAD`
   - Full error messages from logs
   - Configuration files

4. **Ask for help:**
   - GitHub Issues: https://github.com/lindnjoe/klipper_openams/issues


## Credits

This project was created by **knight.rad_iant** and **Armored Turtle Team** on Discord.

Based on the original OpenAMS project with enhancements for AFC integration, retry logic, and clog detection.
