#!/usr/bin/env bash
# Orin: KEYBOARD DRIVING + LIVE MAP + TRAINING DATA. MAKES THE MOTORS TURN.
#
#   bash ~/racecar_ws/tools/orin_map.sh                 # max 0.1 m/s, 300 s, turn radius 0.5 m
#   bash ~/racecar_ws/tools/orin_map.sh 0.1 120 0.35    # max_vx, seconds, turn radius
#
# Run it yourself in ssh (it reads your keys). Starts drive_map.launch.py in the
# background (base, lidar, EKF, slam_toolbox, Foxglove, rosbag), then the keyboard.
# Watch the map: Foxglove ws://192.168.0.15:8765, 3D panel, display frame map,
# topics /map and /scan.
#
# Limits: max_vx <= 0.15 m/s, <= 600 s (from the start of the keyboard), cmd_timeout
#         0.3 s, firmware timeout about 1.3 s, turn radius >= 0.35 m (firmware limit).
# Stop:   let go of the key, s / space, Ctrl+C (quits and saves the map), Motor switch OFF.
# Result: ~/r550_map/<time>/  bag/ (rosbag), map.pgm + map.yaml, posegraph.*, launch.log
set -o pipefail

MAX_VX="${1:-0.1}"
RUN_TIME="${2:-300}"
RADIUS="${3:-0.5}"
OUT="$HOME/r550_map/$(date +%Y%m%d_%H%M%S)"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

BUSY="wro2026_base_r550/(feedback_node|base_node)|wro2026_lidar_n10p|robot_state_publisher"
BUSY="$BUSY|foxglove_bridge|ekf_node|slam_toolbox"
if pgrep -f "$BUSY" > /dev/null; then
    echo "Other ROS programs are running. Stop them first (Ctrl+C in their terminals):"
    pgrep -af "$BUSY" | cut -c1-100
    exit 1
fi
if ! awk "BEGIN{exit !($MAX_VX > 0 && $MAX_VX <= 0.15)}"; then
    echo "max_vx $MAX_VX: must be above 0 and at most 0.15 m/s."; exit 1
fi
if ! awk "BEGIN{exit !($RUN_TIME >= 10 && $RUN_TIME <= 600)}"; then
    echo "seconds $RUN_TIME: must be 10 to 600."; exit 1
fi
if ! awk "BEGIN{exit !($RADIUS >= 0.35 && $RADIUS <= 2.0)}"; then
    echo "turn radius $RADIUS: must be 0.35 to 2.0 m."; exit 1
fi
RADIUS=$(awk "BEGIN{printf \"%.3f\", $RADIUS}")     # ROS parameter must be a float

echo "Drive + map: max_vx $MAX_VX m/s, max $RUN_TIME s, turn radius $RADIUS m, cmd_timeout 0.3 s"
echo "Stop: let go of the key / s / space / Ctrl+C / Motor switch OFF"
read -r -p "Area clear, hand can reach the Motor switch, charger unplugged? Type YES: " a
[ "$a" = YES ] || exit 0
mkdir -p "$OUT"

LAUNCH_PID=""
stop_all() {
    [ -n "$LAUNCH_PID" ] || return 0
    # Measured 2026-11-10: SIGTERM to the launch alone leaves its nodes orphaned, and a launch
    # or rosbag started in the background ignores SIGINT. So signal the whole group: SIGINT
    # stops the nodes (base_node sends its stop frames), SIGTERM makes rosbag close its file.
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 20); do [ "$(pgrep -g "$LAUNCH_PID" | wc -l)" -le 2 ] && break; sleep 0.25; done
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 40); do [ "$(pgrep -g "$LAUNCH_PID" | wc -l)" -eq 0 ] && break; sleep 0.25; done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
    LAUNCH_PID=""
    echo "stopped. RESULT_DIR=$OUT"
}
trap stop_all EXIT

setsid ros2 launch wro2026_bringup drive_map.launch.py enable_commands:=true \
    max_vx:="$MAX_VX" max_run_time:="$RUN_TIME" bag_dir:="$OUT/bag" \
    > "$OUT/launch.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "starting (about 10 s), keep the car still ..."
for _ in $(seq 80); do grep -q "COMMANDS ENABLED" "$OUT/launch.log" && break; sleep 0.25; done
if ! grep -q "COMMANDS ENABLED" "$OUT/launch.log"; then
    echo "base node did not start, see $OUT/launch.log"
    exit 1
fi
sleep 6

trap ':' INT   # Ctrl+C reaches the teleop node (it publishes 0 and quits), then the map is saved
ros2 run wro2026_base_r550 teleop_node --ros-args -p max_speed:="$MAX_VX" \
    -p speed:="$(awk "BEGIN{printf \"%.3f\", ($MAX_VX < 0.1) ? $MAX_VX : 0.1}")" \
    -p turn_radius:="$RADIUS"

echo "keyboard closed, the car is stopped. Saving the map (up to 40 s) ..."
timeout 25 ros2 run nav2_map_server map_saver_cli -f "$OUT/map" \
    --ros-args -p save_map_timeout:=10.0 2>&1 | tail -n 2
timeout 15 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$OUT/posegraph'}" 2>&1 | tail -n 1
