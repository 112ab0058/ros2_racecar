#!/usr/bin/env bash
# Orin: AUTONOMOUS DRIVING with a phone dead-man switch. MAKES THE MOTORS TURN.
#
#   CONFIRM=YES bash ~/racecar_ws/tools/orin_auto.sh [direction ccw] [speed 0.1] [seconds 180]
#
# Ray's corner technique (wro2026_imitation corner.py) drives, but ONLY while the phone
# page's 自動 button is held: bc_node publishes to bc/cmd_vel, and web_teleop_node passes
# those commands to /cmd_vel only while the button is held and they are fresh (< 0.3 s).
# Letting go, switching apps or losing WiFi stops within 0.35 s. The page's keys still
# drive by hand (they win over 自動). Claude Code starts it over ssh in the background
# ONLY AFTER Ray confirmed in chat (CONFIRM=YES replaces typing YES).
#
#   drive_map.launch.py  base (enable_commands, max_vx, max_run_time), lidar, EKF,
#                        slam_toolbox, rosbag
#   scan_memory_node     /scan + /odometry/filtered -> /scan_filled (black-wall blind spot)
#   bc_node              policy corner on /scan_filled -> bc/cmd_vel (+ bc/decision, bc/state)
#   web_teleop_node      phone page, https, 自動 tab
#
# Start the car in the MIDDLE of a straight, centred, pointing in the driving direction
# (the memory needs to have seen the inner wall before the first corner).
# Limits: speed <= 0.12 m/s, 10-600 s, cmd_timeout 0.3 s, firmware timeout ~1.3 s;
#         bc_node stops for good when anything comes within 2 cm of the car outline.
# Stop:   let go of 自動 / Motor switch OFF / touch ~/r550_map/latest/stop or
#         kill -TERM <script pid> (stops driving, saves the map, stops everything).
# Result: ~/r550_map/<time>/ (bag/, map.*, launch.log, memory.log, bc.log, web.log)
set -o pipefail

DIRECTION="${1:-ccw}"
SPEED="${2:-0.1}"
RUN_TIME="${3:-180}"
OUT="$HOME/r550_map/$(date +%Y%m%d_%H%M%S)_auto"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

BUSY="wro2026_base_r550/(feedback_node|base_node|web_teleop_node)|wro2026_lidar_n10p"
BUSY="$BUSY|robot_state_publisher|foxglove_bridge|ekf_node|slam_toolbox|bc_node"
if pgrep -f "$BUSY" > /dev/null; then
    echo "Other ROS programs are running. Stop them first:"
    pgrep -af "$BUSY" | cut -c1-100
    exit 1
fi
case "$DIRECTION" in ccw|cw) ;; *) echo "direction $DIRECTION: must be ccw or cw."; exit 1 ;; esac
if ! awk "BEGIN{exit !($SPEED > 0 && $SPEED <= 0.12)}"; then
    echo "speed $SPEED: must be above 0 and at most 0.12 m/s."; exit 1
fi
if ! awk "BEGIN{exit !($RUN_TIME >= 10 && $RUN_TIME <= 600)}"; then
    echo "seconds $RUN_TIME: must be 10 to 600."; exit 1
fi
if [ "${CONFIRM:-}" != YES ]; then
    echo "Refused: set CONFIRM=YES only after Ray has confirmed speed, time and how to stop."
    exit 1
fi
SPEED=$(awk "BEGIN{printf \"%.3f\", $SPEED}")      # ROS parameters must be floats
mkdir -p "$OUT"
ln -sfn "$OUT" "$HOME/r550_map/latest"
echo $$ > "$OUT/script.pid"
echo "Autonomous ($DIRECTION) with the phone 自動 button: speed $SPEED m/s, $RUN_TIME s"
echo "RESULT_DIR=$OUT"

LAUNCH_PID=""
WEB_PID=""
MEM_PID=""
BC_PID=""
count() { pgrep -g "$1" | wc -l; }
stop_group() {      # SIGTERM a process group, wait up to 5 s, then SIGKILL
    [ -n "$1" ] || return
    kill -TERM -- "-$1" 2>/dev/null
    for _ in $(seq 20); do [ "$(count "$1")" -eq 0 ] && break; sleep 0.25; done
    kill -KILL -- "-$1" 2>/dev/null
}
finish() {
    trap '' INT TERM HUP
    stop_group "$WEB_PID"
    [ -n "$WEB_PID" ] && echo "phone page stopped (it published zeros)"
    stop_group "$BC_PID"
    stop_group "$MEM_PID"
    if [ -n "$LAUNCH_PID" ] && [ "$(count "$LAUNCH_PID")" -gt 0 ]; then
        echo "saving the map (up to 40 s) ..."
        timeout 25 ros2 run nav2_map_server map_saver_cli -f "$OUT/map" \
            --ros-args -p save_map_timeout:=10.0 2>&1 | tail -n 2
        timeout 15 ros2 service call /slam_toolbox/serialize_map \
            slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT/posegraph'}" 2>&1 | tail -n 1
        # As orin_map.sh: SIGINT to the group stops the nodes (base_node sends its stop
        # frames); SIGTERM makes rosbag close its file.
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 20); do [ "$(count "$LAUNCH_PID")" -le 2 ] && break; sleep 0.25; done
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 40); do [ "$(count "$LAUNCH_PID")" -eq 0 ] && break; sleep 0.25; done
        kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
    fi
    grep -h "bc_node decisions" "$OUT/bc.log" 2>/dev/null
    echo "stopped. RESULT_DIR=$OUT"
}
trap finish EXIT
trap 'exit 0' INT TERM HUP

setsid ros2 launch wro2026_bringup drive_map.launch.py enable_commands:=true \
    max_vx:="$SPEED" max_run_time:="$RUN_TIME" bag_dir:="$OUT/bag" \
    > "$OUT/launch.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "starting (about 15 s), keep the car still ..."
for _ in $(seq 80); do grep -q "COMMANDS ENABLED" "$OUT/launch.log" && break; sleep 0.25; done
if ! grep -q "COMMANDS ENABLED" "$OUT/launch.log"; then
    echo "base node did not start, see $OUT/launch.log"
    exit 1
fi
sleep 6

setsid ros2 run wro2026_lidar_n10p scan_memory_node > "$OUT/memory.log" 2>&1 < /dev/null &
MEM_PID=$!
setsid ros2 run wro2026_imitation bc_node --ros-args -p policy:=corner \
    -p direction:="$DIRECTION" -p speed:="$SPEED" -p publish_cmd:=true \
    -p cmd_topic:=bc/cmd_vel -p stop_margin:=0.02 -r scan:=scan_filled \
    > "$OUT/bc.log" 2>&1 < /dev/null &
BC_PID=$!
for _ in $(seq 60); do grep -q "PUBLISHING" "$OUT/bc.log" && break; sleep 0.25; done
if ! grep -q "PUBLISHING" "$OUT/bc.log"; then
    echo "bc_node did not start, see $OUT/bc.log"
    exit 1
fi

setsid ros2 run wro2026_base_r550 web_teleop_node --ros-args \
    -p speed:="$SPEED" -p max_speed:="$SPEED" -p https:=true \
    -p auto_topic:=bc/cmd_vel > "$OUT/web.log" 2>&1 < /dev/null &
WEB_PID=$!
for _ in $(seq 80); do grep -q "phone page" "$OUT/web.log" && break; sleep 0.25; done
TOKEN=$(grep -o 'k=[0-9a-f]*' "$OUT/web.log" | head -n 1)
if [ -z "$TOKEN" ]; then
    echo "phone page did not start, see $OUT/web.log"
    exit 1
fi
for ip in $(hostname -I); do
    case "$ip" in *:*) ;; *) echo "PHONE_URL https://$ip:8080/?$TOKEN" ;; esac
done
echo "READY: open the 自動 tab and HOLD the button to drive. Time limit $RUN_TIME s."

END=$((SECONDS + RUN_TIME + 5))
while [ "$SECONDS" -lt "$END" ]; do
    if [ -e "$OUT/stop" ]; then echo "stop requested"; break; fi
    if ! kill -0 "$WEB_PID" 2>/dev/null; then echo "phone page ended"; break; fi
    if ! kill -0 "$BC_PID" 2>/dev/null; then echo "bc_node ended"; break; fi
    if [ "$(count "$LAUNCH_PID")" -eq 0 ]; then echo "launch ended"; break; fi
    sleep 1
done
exit 0
