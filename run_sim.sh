#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="$HOME/uas/PX4-Autopilot"
QGC_APPIMAGE="$HOME/uas/QGroundControl-x86_64.AppImage"

# Optional: kill old instances (quietly)
pkill -f "QGroundControl-x86_64.AppImage" >/dev/null 2>&1 || true

# Start QGroundControl silently in the background
chmod +x "$QGC_APPIMAGE"
nohup "$QGC_APPIMAGE" >/dev/null 2>&1 & disown || true

# Run PX4 + Gazebo in THIS terminal
cd "$PX4_DIR"
PX4_GZ_WORLD=aruco make px4_sitl gz_x500_flow_cam_down