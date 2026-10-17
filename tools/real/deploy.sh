#!/usr/bin/env bash
# Bash version of deploy.cmd (Git Bash / WSL / Claude Code). Run from anywhere:
#   bash tools/real/deploy.sh
# git pull, then send the 4 real-car packages + Orin scripts over one ssh and
# build them with tools/real/orin_build.sh. wro2026_sim is never sent.
set -eo pipefail
ORIN="${ORIN:-user2@192.168.0.15}"
cd "$(dirname "$0")/../.."

echo "[1/2] git pull"
git pull --ff-only

echo "[2/2] send to $ORIN and build"
tar --exclude=__pycache__ -cf - tools/real/orin_build.sh tools/real/orin_check.sh \
    src/wro2026_base_r550 src/wro2026_lidar_n10p src/wro2026_description src/wro2026_bringup \
    | ssh "$ORIN" "rm -rf ~/racecar_ws/.deploy && mkdir -p ~/racecar_ws/.deploy && tar -xf - -C ~/racecar_ws/.deploy && bash ~/racecar_ws/.deploy/tools/real/orin_build.sh"

echo "OK. On the Orin: ros2 launch wro2026_bringup sensors.launch.py"
