#!/bin/bash
# Force script to exit if an error occurs
set -e

KLIPPER_PATH="${HOME}/klipper"
KLIPPER_SERVICE_NAME=klipper
SYSTEMDDIR="/etc/systemd/system"
MOONRAKER_CONFIG_DIR="${HOME}/printer_data/config"

# Fall back to old directory for configuration as default
if [ ! -d "${MOONRAKER_CONFIG_DIR}" ]; then
    echo "\"$MOONRAKER_CONFIG_DIR\" does not exist. Falling back to \"${HOME}/klipper_config\" as default."
    MOONRAKER_CONFIG_DIR="${HOME}/klipper_config"
fi

usage(){ echo "Usage: $0 [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>] [-u]" 1>&2; exit 1; }
# Parse command line arguments
while getopts "k:s:c:uh" arg; do
    case $arg in
        k) KLIPPER_PATH=$OPTARG;;
        s) KLIPPER_SERVICE_NAME=$OPTARG;;
        c) MOONRAKER_CONFIG_DIR=$OPTARG;;
        u) UNINSTALL=1;;
        h) usage;;
    esac
done

# Find SRCDIR from the pathname of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRCDIR="$SCRIPT_DIR/src"
SCRIPTSDIR="$SCRIPT_DIR/scripts"

# Verify Klipper has been installed
check_klipper()
{
    if [ "$(sudo systemctl list-units --full -all -t service --no-legend | grep -F "$KLIPPER_SERVICE_NAME.service")" ]; then
        echo "Klipper service found with name \"$KLIPPER_SERVICE_NAME\"."
    else
        echo "[ERROR] Klipper service with name \"$KLIPPER_SERVICE_NAME\" not found, please install Klipper first or specify name with -s."
        exit -1
    fi
}

check_folders()
{
    if [ ! -d "$KLIPPER_PATH/klippy/extras/" ]; then
        echo "[ERROR] Klipper installation not found in directory \"$KLIPPER_PATH\". Exiting."
        exit -1
    fi
    echo "Klipper installation found at $KLIPPER_PATH"

    if [ ! -f "${MOONRAKER_CONFIG_DIR}/moonraker.conf" ]; then
        echo "[ERROR] Moonraker configuration not found in directory \"$MOONRAKER_CONFIG_DIR\". Exiting."
        exit -1
    fi
    echo "Moonraker configuration found at $MOONRAKER_CONFIG_DIR"
}

# Link OpenAMS extension modules to Klipper
link_extension()
{
    echo -n "Linking OpenAMS extension to Klipper... "
    for file in "${SRCDIR}"/*.py; do
        ln -sf "${file}" "${KLIPPER_PATH}/klippy/extras/"
    done
    echo "[OK]"
}

# Link OpenAMS helper scripts to Klipper
link_scripts()
{
    echo -n "Linking OpenAMS scripts to Klipper... "
    for file in "${SCRIPTSDIR}"/*.py; do
        ln -sf "${file}" "${KLIPPER_PATH}/scripts/$(basename "$file")"
    done
    echo "[OK]"
}

# Restart Moonraker service
restart_moonraker()
{
    echo -n "Restarting Moonraker... "
    sudo systemctl restart moonraker
    echo "[OK]"
}

# Add OpenAMS update manager section to moonraker.conf
add_updater()
{
    echo -e -n "Adding update manager to moonraker.conf... "

    if grep -q '\[update_manager openams\]' "${MOONRAKER_CONFIG_DIR}/moonraker.conf"; then
        echo "[update_manager openams] already exists in moonraker.conf [SKIPPED]"
        return
    fi

    {
        echo ""
        cat "${SCRIPT_DIR}/file_templates/moonraker_update.txt"
        echo ""
    } >> "${MOONRAKER_CONFIG_DIR}/moonraker.conf"

    echo "[OK]"
    restart_moonraker
}

# Install OpenAMS config files into the Moonraker configuration directory
install_config()
{
    echo -n "Installing OpenAMS config files... "

    if [ ! -f "${MOONRAKER_CONFIG_DIR}/oams.cfg" ]; then
        cp "${SCRIPT_DIR}/oams_sample.cfg" "${MOONRAKER_CONFIG_DIR}/oams.cfg"
        echo -n "oams.cfg installed... "
    else
        echo -n "oams.cfg already exists [SKIPPED]... "
    fi

    if [ ! -f "${MOONRAKER_CONFIG_DIR}/oams_macros.cfg" ]; then
        cp "${SCRIPT_DIR}/oams_macros.cfg" "${MOONRAKER_CONFIG_DIR}/oams_macros.cfg"
        echo -n "oams_macros.cfg installed... "
    else
        echo -n "oams_macros.cfg already exists [SKIPPED]... "
    fi

    echo "[OK]"
}

# Add OpenAMS config include to printer.cfg
add_printer_includes()
{
    echo -n "Adding OpenAMS include to printer.cfg... "
    local printer_cfg="${MOONRAKER_CONFIG_DIR}/printer.cfg"
    local tmp_cfg

    if [ ! -f "$printer_cfg" ]; then
        echo "[SKIPPED] printer.cfg not found."
        return
    fi

    if grep -q '^\[include[[:space:]]*oams\.cfg\]$' "$printer_cfg"; then
        echo "oams.cfg already included [SKIPPED]"
        return
    fi

    tmp_cfg="$(mktemp)"

    if grep -q '^\[printer\]$' "$printer_cfg"; then
        awk '
        !inserted && /^\[printer\]$/ {
            print "[include oams.cfg]"
            print ""
            inserted=1
        }
        { print }
        ' "$printer_cfg" > "$tmp_cfg"
    else
        {
            echo "[include oams.cfg]"
            echo ""
            cat "$printer_cfg"
        } > "$tmp_cfg"
    fi

    mv "$tmp_cfg" "$printer_cfg"
    echo "[OK]"
}

# Remove OpenAMS config include from printer.cfg without deleting nearby config
remove_printer_includes()
{
    echo -n "Removing OpenAMS includes from printer.cfg... "
    local printer_cfg="${MOONRAKER_CONFIG_DIR}/printer.cfg"
    local tmp_cfg

    if [ ! -f "$printer_cfg" ]; then
        echo "[SKIPPED] printer.cfg not found."
        return
    fi

    tmp_cfg="$(mktemp)"

    awk '
    BEGIN { skip_blank=0 }
    /^\[include[[:space:]]*oams\.cfg\]$/ {
        skip_blank=1
        next
    }
    skip_blank && /^$/ {
        skip_blank=0
        next
    }
    {
        skip_blank=0
        print
    }
    ' "$printer_cfg" > "$tmp_cfg"

    mv "$tmp_cfg" "$printer_cfg"
    echo "[OK]"
}

# Remove OpenAMS update manager section from moonraker.conf
remove_updater()
{
    echo -n "Removing OpenAMS update manager from moonraker.conf... "
    local moonraker_cfg="${MOONRAKER_CONFIG_DIR}/moonraker.conf"
    local tmp_cfg

    if [ ! -f "$moonraker_cfg" ]; then
        echo "[SKIPPED] moonraker.conf not found."
        return
    fi

    tmp_cfg="$(mktemp)"

    awk '
    BEGIN { skip=0 }
    /^\[update_manager openams\]$/ { skip=1; next }
    /^\[/ { if (skip) skip=0 }
    !skip { print }
    ' "$moonraker_cfg" > "$tmp_cfg"

    mv "$tmp_cfg" "$moonraker_cfg"

    echo "[OK]"
    restart_moonraker
}


restart_klipper()
{
    echo -n "Restarting Klipper... "
    sudo systemctl restart $KLIPPER_SERVICE_NAME
    echo "[OK]"
}

start_klipper()
{
    echo -n "Starting Klipper... "
    sudo systemctl start $KLIPPER_SERVICE_NAME
    echo "[OK]"
}

stop_klipper()
{
    echo -n "Stopping Klipper... "
    sudo systemctl stop $KLIPPER_SERVICE_NAME
    echo "[OK]"
}

uninstall()
{
    if [ -f "${KLIPPER_PATH}/klippy/extras/oams.py" ]; then
        echo -n "Uninstalling OpenAMS... "
        for file in "${SRCDIR}"/*.py; do
            rm -f "${KLIPPER_PATH}/klippy/extras/$(basename "$file")"
        done
        for file in "${SCRIPTSDIR}"/*.py; do
            rm -f "${KLIPPER_PATH}/scripts/$(basename "$file")"
        done
        echo "[OK]"
        read -p "Remove oams.cfg and oams_macros.cfg from ${MOONRAKER_CONFIG_DIR}? [y/N] " confirm
        if [[ "$confirm" =~ ^[Yy]$ ]]; then
            rm -f "${MOONRAKER_CONFIG_DIR}/oams.cfg"
            rm -f "${MOONRAKER_CONFIG_DIR}/oams_macros.cfg"
            echo "Config files removed."
        else
            echo "Config files kept."
        fi
    else
        echo "oams.py not found in \"${KLIPPER_PATH}/klippy/extras/\". Is it installed?"
        echo "[FAILED]"
        return 1
    fi
}

# Helper functions
verify_ready()
{
    if [ "$EUID" -eq 0 ]; then
        echo "[ERROR] This script must not run as root. Exiting."
        exit -1
    fi
}

# Run steps
verify_ready
check_klipper
check_folders
stop_klipper
if [[ -z "${UNINSTALL:-}" ]]; then
    link_extension
    link_scripts
    add_updater
    install_config
    add_printer_includes
else
    uninstall
    remove_printer_includes
    remove_updater
fi
start_klipper