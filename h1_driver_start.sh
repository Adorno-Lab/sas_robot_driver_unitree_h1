#!/bin/bash

# Copyright (c) 2026-2026 Adorno-Lab software developments
#
#    This file is part of sas_robot_driver_unitree_h1.
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This software is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with this software.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Daniel S. J. Derwent, email: daniel.derwent@manchester.ac.uk
#
# ################################################################

# Stop on failed commands (-e) or references to unset variables (-u).
# Optional environment variables use ${VARIABLE:-default} to avoid
# triggering the unset-variable check.
set -eu


# ------------------------------------------------------------------
# Temporary X11 authentication files and cleanup
# ------------------------------------------------------------------
# These files are only needed for authenticated X11, such as the
# SSH-forwarded display used when connecting to the H1.
#
# Initialise the paths so cleanup also works when no files are created.
AUTH_FILE=""
AUTH_ENTRIES=""

cleanup() {
    [ -z "$AUTH_FILE" ] || rm -f "$AUTH_FILE"
    [ -z "$AUTH_ENTRIES" ] || rm -f "$AUTH_ENTRIES"
}

# Keep the files available while docker run is active, then remove them
# when this launcher exits. Interrupts also trigger the EXIT cleanup.
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM


# ------------------------------------------------------------------
# Assemble optional GUI arguments
# ------------------------------------------------------------------
# POSIX shell positional parameters hold the optional Docker arguments.
# "$@" later expands each argument separately, preserving quoted paths.
#
# This avoids Bash arrays, so the script can be run using:
#   sh h1_driver_start.sh
set --


# ------------------------------------------------------------------
# Select the display configuration
# ------------------------------------------------------------------
# Three routes are supported:
#   1. Headless: no GUI configuration.
#   2. Local WSLg: share the local X11 socket and DISPLAY.
#   3. Authenticated X11: export the current display's credentials.
#
# GUI=0 explicitly selects headless mode:
#   GUI=0 sh h1_driver_start.sh
#
# Otherwise, the launcher attempts GUI configuration automatically.
if [ "${GUI:-auto}" = "0" ] || [ -z "${DISPLAY:-}" ]; then

    # No display is requested or available. The container can still be
    # used for compilation, terminal tools and non-GUI applications.
    echo "Starting without GUI."

elif [ -n "${WSL_DISTRO_NAME:-}" ] \
    && [ -d /mnt/wslg ] \
    && [ "$DISPLAY" = ":0" ]; then

    # --------------------------------------------------------------
    # Local WSLg display on the development PC
    # --------------------------------------------------------------
    # This branch targets a local WSLg setup with DISPLAY=:0.
    # The existing OpenCV viewer uses X11, so only DISPLAY and the X11
    # socket are shared. Wayland and audio forwarding are not configured.
    #
    # Do not run the SSH Xauthority setup here: this WSLg route does not
    # depend on the user's ~/.Xauthority file.
    #
    # Requiring DISPLAY=:0 prevents an SSH-forwarded display such as
    # localhost:10.0 from being mistaken for the local WSLg display.
    echo "Using local WSLg display."

    set -- "$@" \
        -e "DISPLAY=$DISPLAY" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:ro

elif command -v xauth >/dev/null 2>&1; then

    # --------------------------------------------------------------
    # Authenticated X11, including SSH forwarding on the H1
    # --------------------------------------------------------------
    # DISPLAY identifies the display, but the container also needs its
    # authentication credentials. Export only the entries matching the
    # current display into private temporary files.
    #
    # xauth uses the host's XAUTHORITY environment variable when set,
    # otherwise it uses ~/.Xauthority.
    AUTH_FILE=$(mktemp /tmp/h1-camera-xauth.XXXXXX)
    AUTH_ENTRIES=$(mktemp /tmp/h1-camera-entries.XXXXXX)
    chmod 600 "$AUTH_FILE" "$AUTH_ENTRIES"

    if xauth nlist "$DISPLAY" > "$AUTH_ENTRIES" \
        && [ -s "$AUTH_ENTRIES" ]; then

        # Change the address-family field to FamilyWild (ffff), allowing
        # the exported credentials to match from inside the container.
        # The authentication cookie itself is retained unchanged.
        sed 's/^..../ffff/' "$AUTH_ENTRIES" \
            | xauth -f "$AUTH_FILE" nmerge -

        # Mount the generated credentials read-only and tell applications
        # inside the container where to find them.
        set -- "$@" \
            -e "DISPLAY=$DISPLAY" \
            -e XAUTHORITY=/tmp/container.xauth \
            --mount "type=bind,src=$AUTH_FILE,dst=/tmp/container.xauth,readonly"

        # Local X11 displays use this socket directory. Mount it only if
        # present. SSH-forwarded localhost displays instead use the SSH
        # proxy over TCP, reachable through the host network namespace.
        if [ -d /tmp/.X11-unix ]; then
            set -- "$@" \
                -v /tmp/.X11-unix:/tmp/.X11-unix:ro
        fi

        echo "Using authenticated X11 display: $DISPLAY"
    else
        # Missing credentials must not prevent development-container
        # startup. GUI applications will not have a configured display.
        echo "No X11 credentials found; starting without GUI."
    fi

else
    # No credential-export tool is available. Start without GUI rather
    # than blocking compilation or other terminal-based development.
    echo "xauth unavailable; starting without GUI."
fi


# ------------------------------------------------------------------
# Launch the development/robot container
# ------------------------------------------------------------------
# -it:
#   Provide an interactive terminal.
#
# --name:
#   Sets the name of the container
#
# --rm:
#   Remove the container when it exits.
#
# --privileged:
#   Grants broad hardware-access permissions.
#
# --network host:
#   Share the host network namespace, preserving the robot networking
#   setup and access to the host's SSH-forwarded localhost display (if there is one).
#
# --ipc host:
#   Preserve the existing shared host IPC configuration.
#
# "$@":
#   Insert the GUI arguments assembled above, or no arguments in
#   headless mode.
#
# Workspace mount:
#   Mount the current host directory into the container. Run this
#   launcher from the repository root. Source and build changes made
#   in this directory are shared with the host.
#
# Final argument:
#   Select the existing sas_robot_driver_unitree_h1 Docker image.
docker run \
    -it \
    --name sas_robot_driver_unitree_h1 \
    --rm \
    --privileged \
    --network host \
    --ipc host \
    "$@" \
    -v "$(pwd):/sas_robot_driver_unitree_h1" \
    sas_robot_driver_unitree_h1