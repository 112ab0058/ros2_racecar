#!/usr/bin/env bash
# Orin: PHONE DRIVING + LIVE MAP + TRAINING DATA. MAKES THE MOTORS TURN.
#
#   CONFIRM=YES bash ~/racecar_ws/tools/orin_field.sh [max_vx 0.1] [seconds 300] [turn_radius 0.5]
#
# orin_map.sh without a keyboard, for the field when there is only a phone. Claude Code
# starts it over ssh in the background ONLY AFTER Ray confirmed in chat (CONFIRM=YES
# replaces typing YES). Ray drives with the phone page (web_teleop_node): hold a button
# to drive, let go to stop. The script prints PHONE_URL lines; open the one on the
# phone's network (hotspot) in the phone's browser.
#
# Limits: as orin_map.sh: max_vx <= 0.15 m/s, 10-600 s, turn radius 0.35-2.0 m,
#         cmd_timeout 0.3 s, firmware timeout ~1.3 s; the page stops 0.35 s after the
#         phone stops sending.
# Stop:   let go of the button / Motor switch OFF / touch ~/r550_map/latest/stop or
#         kill -TERM <script pid> (stops driving, saves the map, stops everything).
# Result: ~/r550_map/<time>/ (bag/, map.*, posegraph.*, launch.log, web.log);
#         ~/r550_map/latest points to it.
set -o pipefail

MAX_VX="${1:-0.1}"
RUN_TIME="${2:-300}"
RADIUS="${3:-0.5}"
OUT="$HOME/r550_map/$(date +%Y%m%d_%H%M%S)"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

BUSY="wro2026_base_r550/(feedback_node|base_node|web_teleop_node)|wro2026_lidar_n10p"
BUSY="$BUSY|robot_state_publisher|foxglove_bridge|ekf_node|slam_toolbox|bc_node"
if pgrep -f "$BUSY" > /dev/null; then
    echo "Other ROS programs are running. Stop them first:"
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
if [ "${CONFIRM:-}" != YES ]; then
    echo "Refused: set CONFIRM=YES only after Ray has confirmed speed, time and how to stop."
    exit 1
fi
RADIUS=$(awk "BEGIN{printf \"%.3f\", $RADIUS}")     # ROS parameters must be floats
SPEED=$(awk "BEGIN{printf \"%.3f\", ($MAX_VX < 0.1) ? $MAX_VX : 0.1}")
mkdir -p "$OUT"
ln -sfn "$OUT" "$HOME/r550_map/latest"
echo $$ > "$OUT/script.pid"
echo "Phone drive + map: speed $SPEED (max $MAX_VX) m/s, $RUN_TIME s, turn radius $RADIUS m"
echo "RESULT_DIR=$OUT"

LAUNCH_PID=""
WEB_PID=""
count() { pgrep -g "$1" | wc -l; }
finish() {
    trap '' INT TERM HUP
    if [ -n "$WEB_PID" ]; then          # first: phone driving off (it publishes zeros)
        kill -TERM -- "-$WEB_PID" 2>/dev/null
        for _ in $(seq 20); do [ "$(count "$WEB_PID")" -eq 0 ] && break; sleep 0.25; done
        kill -KILL -- "-$WEB_PID" 2>/dev/null
        echo "phone driving stopped"
    fi
    if [ -n "$LAUNCH_PID" ] && [ "$(count "$LAUNCH_PID")" -gt 0 ]; then
        echo "saving the map (up to 40 s) ..."
        timeout 25 ros2 run nav2_map_server map_saver_cli -f "$OUT/map" \
            --ros-args -p save_map_timeout:=10.0 2>&1 | tail -n 2
        timeout 15 ros2 service call /slam_toolbox/serialize_map \
            slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT/posegraph'}" 2>&1 | tail -n 1
        # As orin_map.sh (measured 2026-11-10): SIGINT to the group stops the nodes,
        # base_node sends its stop frames; SIGTERM makes rosbag close its file.
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 20); do [ "$(count "$LAUNCH_PID")" -le 2 ] && break; sleep 0.25; done
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 40); do [ "$(count "$LAUNCH_PID")" -eq 0 ] && break; sleep 0.25; done
        kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
    fi
    echo "stopped. RESULT_DIR=$OUT"
}
trap finish EXIT
trap 'exit 0' INT TERM HUP

setsid ros2 launch wro2026_bringup drive_map.launch.py enable_commands:=true \
    max_vx:="$MAX_VX" max_run_time:="$RUN_TIME" bag_dir:="$OUT/bag" \
    > "$OUT/launch.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "starting (about 15 s), keep the car still ..."
for _ in $(seq 80); do grep -q "COMMANDS ENABLED" "$OUT/launch.log" && break; sleep 0.25; done
if ! grep -q "COMMANDS ENABLED" "$OUT/launch.log"; then
    echo "base node did not start, see $OUT/launch.log"
    exit 1
fi
sleep 6

setsid ros2 run wro2026_base_r550 web_teleop_node --ros-args \
    -p speed:="$SPEED" -p max_speed:="$(awk "BEGIN{printf \"%.3f\", $MAX_VX}")" \
    -p turn_radius:="$RADIUS" > "$OUT/web.log" 2>&1 < /dev/null &
WEB_PID=$!
for _ in $(seq 40); do grep -q "phone page" "$OUT/web.log" && break; sleep 0.25; done
TOKEN=$(grep -o 'k=[0-9a-f]*' "$OUT/web.log" | head -n 1)
if [ -z "$TOKEN" ]; then
    echo "phone page did not start, see $OUT/web.log"
    exit 1
fi
for ip in $(hostname -I); do
    case "$ip" in *:*) ;; *) echo "PHONE_URL http://$ip:8080/?$TOKEN" ;; esac
done
echo "READY: hold a button on the phone page to drive. Time limit $RUN_TIME s."

END=$((SECONDS + RUN_TIME + 5))
while [ "$SECONDS" -lt "$END" ]; do
    if [ -e "$OUT/stop" ]; then echo "stop requested"; break; fi
    if ! kill -0 "$WEB_PID" 2>/dev/null; then echo "phone page ended"; break; fi
    if [ "$(count "$LAUNCH_PID")" -eq 0 ]; then echo "launch ended"; break; fi
    sleep 1
done
exit 0
