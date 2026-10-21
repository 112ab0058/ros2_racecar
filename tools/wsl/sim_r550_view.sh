#!/usr/bin/env bash
# WSL (WSLg screen): LOOK at the R550 Gazebo simulation. SIMULATION ONLY, separate
# from the Orin and the real car. Run in a WSL terminal:
#
#   bash /mnt/c/Users/chaon/ros2_racecar/tools/wsl/sim_r550_view.sh         # Gazebo window
#   bash /mnt/c/Users/chaon/ros2_racecar/tools/wsl/sim_r550_view.sh drive   # + scripted course
#   bash /mnt/c/Users/chaon/ros2_racecar/tools/wsl/sim_r550_view.sh lap     # + 2 laps (pure pursuit)
#
# 1. copies wro2026_base_r550 / _description / _bringup / _sim_r550 from the Windows repo
#    into ~/r550_sim_ws (NOT ~/ros2_ws, NOT the old ~/ros2_racecar copy) and builds them
# 2. sim.launch.py gui:=true foxglove:=false, ROS_DOMAIN_ID=43 (Orin sim uses 42, car 0)
# 3. "drive": after 25 s drive_test_node runs the same course as orin_sim_check.sh
#    (result: ~/r550_sim_ws/drive_out/drive_test.txt)
#    "lap": after 25 s lap_node drives 2 laps on the lane centre line with the EKF pose
#    (result: ~/r550_sim_ws/lap_out/lap.txt)
#
# Drive by hand instead, in a second WSL terminal:
#   export ROS_DOMAIN_ID=43; source ~/r550_sim_ws/install/setup.bash
#   ros2 run wro2026_base_r550 teleop_node
# Stop: Ctrl+C here (closes Gazebo and every node).
set -eo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$HOME/r550_sim_ws"
PKGS=(wro2026_base_r550 wro2026_description wro2026_bringup wro2026_sim_r550)
export ROS_DOMAIN_ID=43

source /opt/ros/jazzy/setup.bash
mkdir -p "$WS/src"
for p in "${PKGS[@]}"; do
    rm -rf "${WS:?}/src/$p"
    cp -r "$REPO/src/$p" "$WS/src/$p"
done
echo "[build] $WS"
(cd "$WS" && colcon build --packages-select "${PKGS[@]}" 2>&1 | tail -n 3)
source "$WS/install/setup.bash"

case "${1:-}" in
    drive) NODE=drive_test_node; RES="$WS/drive_out"; TXT=drive_test.txt; EXTRA=() ;;
    lap)   NODE=lap_node; RES="$WS/lap_out"; TXT=lap.txt; EXTRA=(-p laps:=2) ;;
    *)     NODE="" ;;
esac
if [ -n "$NODE" ]; then
    mkdir -p "$RES"
    (sleep 25 && timeout 150 ros2 run wro2026_sim_r550 "$NODE" --ros-args \
        -p use_sim_time:=true -p out:="$RES" "${EXTRA[@]}" < /dev/null \
        && cat "$RES/$TXT") &
fi
echo "[launch] Gazebo window opens in ~10 s; Ctrl+C to stop"
ros2 launch wro2026_sim_r550 sim.launch.py gui:=true foxglove:=false
