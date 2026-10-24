#!/usr/bin/env bash
# WSL (WSLg screen): LOOK at the R550 Gazebo simulation. SIMULATION ONLY, separate
# from the Orin and the real car. Run in a WSL terminal:
#
#   bash /mnt/c/Users/chaon/ros2_racecar/tools/wsl/sim_r550_view.sh         # Gazebo window
#   bash /mnt/c/Users/chaon/ros2_racecar/tools/wsl/sim_r550_view.sh drive   # + scripted course
#   bash /mnt/c/Users/chaon/ros2_racecar/tools/wsl/sim_r550_view.sh lap     # + 2 laps (pure pursuit)
#   bash /mnt/c/Users/chaon/ros2_racecar/tools/wsl/sim_r550_view.sh nav     # + Nav2, 2 laps
#
# 1. copies wro2026_base_r550 / _description / _bringup / _sim_r550 from the Windows repo
#    into ~/r550_sim_ws (NOT ~/ros2_ws, NOT the old ~/ros2_racecar copy) and builds them
# 2. sim.launch.py gui:=true foxglove:=false, ROS_DOMAIN_ID=43 (Orin sim uses 42, car 0)
# 3. "drive": after 25 s drive_test_node runs the same course as orin_sim_check.sh
#    (result: ~/r550_sim_ws/drive_out/drive_test.txt)
#    "lap": after 25 s lap_node drives 2 laps on the lane centre line with the EKF pose
#    (result: ~/r550_sim_ws/lap_out/lap.txt)
#    "nav": Nav2 (wro2026_bringup nav2.launch.py on maps/field_sim.yaml) starts after 20 s,
#    nav_lap_node sends 2 laps of goals after 40 s (result: ~/r550_sim_ws/nav_out/nav_lap.txt)
#
# Drive by hand instead, in a second WSL terminal:
#   export ROS_DOMAIN_ID=43; source ~/r550_sim_ws/install/setup.bash
#   ros2 run wro2026_base_r550 teleop_node
# drive / lap / nav close everything by themselves 5 s after printing the result.
# Stop earlier (or the plain Gazebo window): Ctrl+C here (closes Gazebo and every node).
set -eo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$HOME/r550_sim_ws"
PKGS=(wro2026_base_r550 wro2026_description wro2026_bringup wro2026_sim_r550)
WAIT=25
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
# Two Gazebo worlds on one PC share their topics and break each other (2026-10-21: an old
# "gz sim server/gui" pair outlived Ctrl+C). Close leftovers first, and everything on exit:
# background jobs (Nav2, test nodes) ignore Ctrl+C inside a script.
if pgrep -f "gz sim" > /dev/null; then
    echo "[cleanup] closing a Gazebo that is still running:"; pgrep -af "gz sim"
    pkill -TERM -f "gz sim"; sleep 2; pkill -KILL -f "gz sim" 2>/dev/null || true
fi
trap 'trap - EXIT; trap "" TERM; pkill -TERM -f "gz sim" 2>/dev/null; kill -TERM 0 2>/dev/null' EXIT

case "${1:-}" in
    drive) NODE=drive_test_node; RES="$WS/drive_out"; TXT=drive_test.txt; EXTRA=() ;;
    lap)   NODE=lap_node; RES="$WS/lap_out"; TXT=lap.txt; EXTRA=(-p laps:=2) ;;
    nav)   NODE=nav_lap_node; RES="$WS/nav_out"; TXT=nav_lap.txt; EXTRA=(-p laps:=2); WAIT=40
           MAP="$WS/install/wro2026_sim_r550/share/wro2026_sim_r550/maps/field_sim.yaml"
           (sleep 20 && ros2 launch wro2026_bringup nav2.launch.py use_sim_time:=true \
               map:="$MAP" > "$WS/nav2.log" 2>&1 < /dev/null) & ;;
    *)     NODE="" ;;
esac
echo "[launch] Gazebo window opens in ~10 s; Ctrl+C to stop"
ros2 launch wro2026_sim_r550 sim.launch.py gui:=true foxglove:=false &
LAUNCH=$!
# top-down camera once the window is up: 2.4 m above the field centre, looking straight
# down, north (+y) up (quaternion = yaw 90 deg * pitch 90 deg)
(sleep 15 && gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req 'pose: {position: {x: 0, y: 0, z: 2.4}, orientation: {x: -0.5, y: 0.5, z: 0.5, w: 0.5}}' \
    > /dev/null 2>&1) &
if [ -n "$NODE" ]; then
    mkdir -p "$RES"
    sleep "$WAIT"
    timeout 200 ros2 run wro2026_sim_r550 "$NODE" --ros-args \
        -p use_sim_time:=true -p out:="$RES" "${EXTRA[@]}" < /dev/null || true
    echo; cat "$RES/$TXT" 2>/dev/null || echo "no result file in $RES"
    echo "[done] closing Gazebo and every node in 5 s (result: $RES/$TXT)"
    sleep 5
    exit 0                       # the EXIT trap stops everything
fi
wait "$LAUNCH"
