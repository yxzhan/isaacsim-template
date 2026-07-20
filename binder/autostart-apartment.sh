#!/usr/bin/env bash
#
# Auto-launch the apartment Isaac Sim demo as soon as the graphical session
# (and $DISPLAY) is up.
#
# Wired via XDG autostart (~/.config/autostart/IsaacSim-Apartment-autostart.desktop).
# In this image the X server is NOT running at container start: the VNC server is
# spawned on demand by jupyter-remote-desktop-proxy the first time the user opens
# the "Desktop" tab, and its xstartup is `exec dbus-launch xfce4-session` -- so
# this script fires exactly then, which is when a display first exists.
#
# examples/apartment.py runs with "headless": False, so it hard-requires a
# display; there is nothing useful to do before the desktop session exists.
#
# Opens a terminal on the desktop so the sim has a real tty and its log output
# stays visible. Written defensively so it survives across terminal emulators.
set -u

: "${DISPLAY:=:1}"
export DISPLAY

REPO_DIR="${REPO_DIR:-$HOME/isaacsim-template}"
WRAPPER="${REPO_DIR}/binder/isaacsim_python_wrapper.sh"
TARGET="examples/apartment.py"

# The session manager already started us after X came up, but give the display
# server / window manager a moment to actually accept clients before we spawn
# Isaac Sim. No hard dependency on xdpyinfo -- fall back to the X socket.
for _ in $(seq 1 60); do
    if command -v xdpyinfo >/dev/null 2>&1; then
        xdpyinfo >/dev/null 2>&1 && break
    else
        n="${DISPLAY#*:}"; n="${n%%.*}"
        [ -S "/tmp/.X11-unix/X${n}" ] && break
    fi
    sleep 0.5
done

# ROS 2 must be sourced for the Isaac Sim ROS 2 bridge nodes in apartment.py.
# The desktop session inherits the entrypoint's environment, but source again
# defensively -- it is idempotent and cheap.
#
# `set +u` around it is required, not cosmetic: ROS's setup.bash dereferences
# AMENT_TRACE_SETUP_FILES et al. while they are unset, which under `set -u`
# aborts this script before the sim is ever launched.
_ros_setup="${ROS_PATH:-/opt/ros/jazzy}/setup.bash"
if [ -f "${_ros_setup}" ]; then
    set +u
    # shellcheck disable=SC1090
    . "${_ros_setup}"
    set -u
fi

# apartment.py resolves every asset from BASE_DIR (its own __file__), so the
# working directory only decides how the target path is spelled.
RUN_CMD="cd '${REPO_DIR}' && '${WRAPPER}' '${TARGET}'"

# Pick whatever terminal emulator this image ships. Real terminals are tried
# before the x-terminal-emulator alternative: in this image that alternative
# resolves to Debian's gnome-terminal.wrapper, an arg-translation shim that does
# NOT understand `-- <argv>` and simply hangs when given it. The wrapper only
# speaks the legacy `-e <string>` form, same as the older emulators.
for t in gnome-terminal xfce4-terminal mate-terminal lxterminal konsole xterm x-terminal-emulator; do
    if command -v "$t" >/dev/null 2>&1; then
        TERM_EMU="$t"
        break
    fi
done

case "${TERM_EMU:-}" in
    "")
        # No terminal emulator found: run without a tty. The sim still comes up
        # on the desktop; only the console log has nowhere to go.
        exec bash -c "${RUN_CMD}"
        ;;
    gnome-terminal|xfce4-terminal|mate-terminal)
        exec "$TERM_EMU" -- bash -c "${RUN_CMD}; exec bash"
        ;;
    *)
        exec "$TERM_EMU" -e "bash -c \"${RUN_CMD}; exec bash\""
        ;;
esac
