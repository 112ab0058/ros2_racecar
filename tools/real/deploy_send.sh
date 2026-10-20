#!/usr/bin/env bash
# Called by deploy.sh / deploy.cmd after git pull: send the 4 real-car packages
# and tools/real to the Orin over one ssh and build them there.
set -eo pipefail
ORIN="${ORIN:-user2@192.168.0.15}"
cd "$(dirname "$0")/../.."

echo "[2/2] send to $ORIN and build"
tar --exclude=__pycache__ -cf - tools/real \
    src/wro2026_base_r550 src/wro2026_lidar_n10p src/wro2026_description src/wro2026_bringup src/wro2026_sim_r550 \
    | ssh "$ORIN" "rm -rf ~/racecar_ws/.deploy && mkdir -p ~/racecar_ws/.deploy && tar -xf - -C ~/racecar_ws/.deploy && bash ~/racecar_ws/.deploy/tools/real/orin_build.sh"

echo "OK. On the Orin: ros2 launch wro2026_bringup sensors.launch.py"
