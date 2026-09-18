# Sourced by install-openams.sh. Preparation happens before stopping Klipper;
# the working tree is advanced only after Klipper has stopped.
GCO_ROUTINES_URL="https://github.com/OpenAMSOrg/gco-routines.git"

gco_error()
{
    printf '[ERROR] gco-routines: %s\n' "$*" >&2
    return 1
}

check_gco_destination()
{
    local destination="${KLIPPER_PATH}/klippy/extras/gco_routines"
    local source="${GCO_ROUTINES_PATH}/klippy_extra/gco_routines"
    if [[ -e "$destination" || -L "$destination" ]]; then
        if [[ ! -L "$destination" ]] || [[ "$(readlink -m "$destination")" != "$source" ]]; then
            gco_error "$destination already exists and is not this checkout's symlink. Back up and migrate the existing installation explicitly; nothing was replaced."
            return 1
        fi
    fi
}

check_gco_checkout()
{
    local top origin branch
    top="$(git -C "$GCO_ROUTINES_PATH" rev-parse --show-toplevel 2>/dev/null)" || {
        gco_error "$GCO_ROUTINES_PATH exists but is not a Git checkout. Use -r with a new directory or migrate it explicitly."
        return 1
    }
    [[ "$top" == "$GCO_ROUTINES_PATH" ]] || {
        gco_error "$GCO_ROUTINES_PATH is not a repository root."
        return 1
    }
    origin="$(git -C "$GCO_ROUTINES_PATH" config --get remote.origin.url)" || return 1
    case "${origin,,}" in
        https://github.com/openamsorg/gco-routines.git|https://github.com/openamsorg/gco-routines|git@github.com:openamsorg/gco-routines.git) ;;
        *) gco_error "Refusing to update a checkout with a different origin: $origin"; return 1;;
    esac
    branch="$(git -C "$GCO_ROUTINES_PATH" symbolic-ref --quiet --short HEAD)" || {
        gco_error "Checkout has a detached HEAD; select main explicitly before installing."
        return 1
    }
    [[ "$branch" == main ]] || {
        gco_error "Checkout is on $branch, not main; it was left unchanged."
        return 1
    }
    [[ -z "$(git -C "$GCO_ROUTINES_PATH" status --porcelain)" ]] || {
        gco_error "Checkout has local changes; commit or relocate them before installing."
        return 1
    }
}

prepare_gco_routines()
{
    command -v git >/dev/null || { gco_error "git is required."; return 1; }
    command -v python3 >/dev/null || { gco_error "python3 is required."; return 1; }
    GCO_ROUTINES_PATH="$(python3 -c 'import os, sys; print(os.path.realpath(os.path.expanduser(sys.argv[1])))' "$GCO_ROUTINES_PATH")"
    check_gco_destination || return 1
    if [[ ! -e "$GCO_ROUTINES_PATH" ]]; then
        git clone --branch main --single-branch "$GCO_ROUTINES_URL" "$GCO_ROUTINES_PATH" || return 1
    fi
    check_gco_checkout || return 1
    git -C "$GCO_ROUTINES_PATH" fetch --no-tags origin refs/heads/main:refs/remotes/origin/main || return 1
    GCO_ROUTINES_COMMIT="$(git -C "$GCO_ROUTINES_PATH" rev-parse refs/remotes/origin/main)"
    git -C "$GCO_ROUTINES_PATH" merge-base --is-ancestor HEAD "$GCO_ROUTINES_COMMIT" || {
        gco_error "Local history cannot fast-forward to origin/main; no files were replaced."
        return 1
    }
    local required
    for required in tools/install_gco_routines.py klippy_extra/gco_routines/__init__.py; do
        git -C "$GCO_ROUTINES_PATH" cat-file -e "$GCO_ROUTINES_COMMIT:$required" || {
            gco_error "The fetched repository is missing $required."
            return 1
        }
    done
    printf 'gco-routines prepared at %s (%s).\n' "$GCO_ROUTINES_PATH" "$GCO_ROUTINES_COMMIT"
}

install_gco_routines()
{
    # Recheck local ownership/state before changing any existing source files.
    check_gco_destination || return 1
    check_gco_checkout || return 1
    git -C "$GCO_ROUTINES_PATH" merge-base --is-ancestor HEAD "$GCO_ROUTINES_COMMIT" || {
        gco_error "Local history changed after preparation; refusing to install."
        return 1
    }
    git -C "$GCO_ROUTINES_PATH" merge --ff-only "$GCO_ROUTINES_COMMIT" || return 1
    python3 "$GCO_ROUTINES_PATH/tools/install_gco_routines.py" --klipper "$KLIPPER_PATH" || return 1
    printf 'gco-routines installed. Existing macros and render modes were not changed.\n'
    printf 'Activation is opt-in; review supported Klipper versions and configuration in %s/README.md.\n' "$GCO_ROUTINES_PATH"
}
