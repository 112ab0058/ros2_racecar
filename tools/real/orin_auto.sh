#!/usr/bin/env bash
# Orin: AUTONOMOUS DRIVING with a phone dead-man switch. MAKES THE MOTORS TURN.
#
#   CONFIRM=YES bash ~/racecar_ws/tools/orin_auto.sh [direction auto] [speed 0.1] [seconds 180] \
#       [corner_ahead 0.15] [timing ahead] [laps 3] [policy corner]
#
# policy (2026-11-25): corner (default) = Ray's corner technique (corner.py); model = the
# imitation network ~/r550_map/model.npz (orin_train.sh), or the path of another model file.
# The network decides left / straight / right / full lock from /scan_filled by itself:
# direction, timing, corner_ahead and laps are not used (no lap stop: the time limit ends
# the run). The model file is copied into the result folder. All the safety rules below
# stay the same (自動 hold button, too-close latch, limits).
#
# laps: after this many laps (4 corners each) the car stops where it started (back on the
# start straight, the start point at most 5 cm ahead) and stays stopped; 0 = no lap limit
# (the time limit still ends the run). 3 laps at 0.10 m/s take about 150 s.
#
# timing: ahead (default, the field runs until 2026-11-20) = full lock when the inner wall
# ends corner_ahead m ahead; predict = the predicted turn (wro2026_imitation turn_predict.py,
# replayed and simulated 2026-11-23, not driven yet); corner_ahead is then not used.
#
# direction: auto (default) = at the first corner the side whose wall ends is the inside,
# every corner turns that way; ccw = always left, cw = always right. On the field pass
# ccw / cw: auto still picks the wrong side on the replay of 11-13 run 2 (LOGBOOK 11-20).
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
#   wall_node            C70 camera -> camera/wall_scan: black wall bottoms from ~0.3 m,
#                        where the lidar sees them only sometimes (+ 5 JPEG/s for the bag)
#   scan_memory_node     /scan + camera/wall_scan + /odometry/filtered -> /scan_filled
#                        (fills the black-wall blind spot from lidar and camera memory)
#   bc_node              policy corner on /scan_filled -> bc/cmd_vel (+ bc/decision, bc/state)
#   web_teleop_node      phone page, https, 自動 tab
#
# Start the car anywhere in a straight, the whole car inside it, pointing the way it
# will go (like WRO 2026 rules 9.6-9.8). From the second half of a straight the island's
# end is in both sensors' blind spots; the corner rule finds it from the beams that go
# past it to the far walls (CornerPolicy.crossing_end, 2026-11-24). 2-D sim 2026-11-24
# (predict, measured blind edge, camera max 0.5 m, fixed and auto direction): centred and
# straight, rear axle 0.03-0.56 m past the island's corner (0.56 = bumper at the
# straight's end): 160/160 laps; with +-6 cm across and +-10 deg: cw 144/144, ccw
# 118/144. The ccw misses are all poses where even the best forward-only path keeps
# < 3 cm (the car turns 0.403 m left at full lock in a 0.40 m corridor): in the second
# half of a straight do not put it against the island, nor pointing at the outer wall.
# Until 11-24 only the first ~0.3 m worked.
# Limits: speed <= 0.12 m/s, 10-600 s, cmd_timeout 0.3 s, firmware timeout ~1.3 s;
#         bc_node stops (latched) when the car WILL touch something: the outline moved
#         along the path the commands sent drive (the rest of a corner turn, else 5 cm,
#         the car stops within ~2-3 cm), measured and remembered points alike (Ray
#         11-26: lock only when the computed turn is sure to touch; on the field's
#         0.38 m corridor the island passed the turning car at a lidar-measured
#         1.6-1.9 cm and the old 2 cm margin latched three turns the 左滿舵 key then
#         drove through);
#         drive away with the page's keys, then 重置 in the 自動 tab unlatches it.
# Stop:   let go of 自動 / Motor switch OFF / touch ~/r550_map/latest/stop or
#         kill -TERM <script pid> (stops driving, saves the map, stops everything).
# Result: ~/r550_map/<time>/ (bag/, map.*, launch.log, camera.log, memory.log, bc.log, web.log,
#         net.log: the network addresses at the start and at every change)
# DRIVE_TOKEN=<hex> (env) keeps the phone page's ?k= the same every run (orin_launcher.py);
# unset = a new random one.
set -o pipefail

DIRECTION="${1:-auto}"
SPEED="${2:-0.1}"
RUN_TIME="${3:-180}"
# full lock when the inner wall ends this far ahead of the lidar (m); smaller = later.
# 0.15: with the lidar corrected by 3.6 cm (11-19), replaying 11-13 run 3 (ccw, 0.17 then,
# 11 corners without a stop) locks at the same places: mean +0.1 cm, -2..+3 cm per corner.
CORNER_AHEAD="${4:-0.15}"
TIMING="${5:-ahead}"
LAPS="${6:-3}"
POLICY="${7:-corner}"
OUT="$HOME/r550_map/$(date +%Y%m%d_%H%M%S)_auto"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"
# All our nodes run on this Orin and the phone talks https, not ROS: keep DDS on localhost.
# 2026-11-20 field run 5: every topic stopped in the same second for the rest of the run
# (nodes alive, bag and a fresh subscriber got nothing) while the phone hotspot was the
# network; a WiFi / IPv6 address change breaks DDS bound to it. Check topics from ssh
# with the same variable, or they look silent.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

BUSY="wro2026_base_r550/(feedback_node|base_node|web_teleop_node)|wro2026_lidar_n10p"
BUSY="$BUSY|robot_state_publisher|foxglove_bridge|ekf_node|slam_toolbox|bc_node"
BUSY="$BUSY|wro2026_camera_c70"
if pgrep -f "$BUSY" > /dev/null; then
    echo "Other ROS programs are running. Stop them first:"
    pgrep -af "$BUSY" | cut -c1-100
    exit 1
fi
case "$DIRECTION" in auto|ccw|cw) ;; *) echo "direction $DIRECTION: must be auto, ccw or cw."; exit 1 ;; esac
if ! awk "BEGIN{exit !($SPEED > 0 && $SPEED <= 0.12)}"; then
    echo "speed $SPEED: must be above 0 and at most 0.12 m/s."; exit 1
fi
if ! awk "BEGIN{exit !($RUN_TIME >= 10 && $RUN_TIME <= 600)}"; then
    echo "seconds $RUN_TIME: must be 10 to 600."; exit 1
fi
MODEL=""
case "$POLICY" in
    corner) ;;
    model) MODEL="$HOME/r550_map/model.npz" ;;
    *) MODEL="$POLICY" ;;
esac
if [ -n "$MODEL" ] && [ ! -f "$MODEL" ]; then
    echo "policy $POLICY: no model file $MODEL (train one with orin_train.sh)."; exit 1
fi
if ! awk "BEGIN{exit !($CORNER_AHEAD >= -0.10 && $CORNER_AHEAD <= 0.25)}"; then
    echo "corner_ahead $CORNER_AHEAD: must be -0.10 to 0.25 m."; exit 1
fi
case "$TIMING" in
    ahead|predict) ;;
    *) echo "timing $TIMING: must be ahead or predict."; exit 1 ;;
esac
case "$LAPS" in
    [0-9]|10) ;;
    *) echo "laps $LAPS: must be a whole number 0 to 10 (0 = no limit)."; exit 1 ;;
esac
if [ "${CONFIRM:-}" != YES ]; then
    echo "Refused: set CONFIRM=YES only after Ray has confirmed speed, time and how to stop."
    exit 1
fi
SPEED=$(awk "BEGIN{printf \"%.3f\", $SPEED}")      # ROS parameters must be floats
CORNER_AHEAD=$(awk "BEGIN{printf \"%.3f\", $CORNER_AHEAD}")
mkdir -p "$OUT"
ln -sfn "$OUT" "$HOME/r550_map/latest"
echo $$ > "$OUT/script.pid"
echo "Autonomous ($DIRECTION, timing $TIMING, corner_ahead $CORNER_AHEAD m, laps $LAPS)" \
     "with the phone 自動 button:" \
     "speed $SPEED m/s, $RUN_TIME s"
echo "RESULT_DIR=$OUT"
# FIELD_REVIEW A2 (2026-11-27): without lingering, 10 s after this user's last ssh session
# ends, logind (RemoveIPC=yes) deletes the user's /dev/shm files, Fast DDS shared memory
# included, and every topic stops mid-run. Warn only; the run still starts.
if [ "$(loginctl show-user "$(id -u)" -p Linger --value 2>/dev/null)" != yes ]; then
    echo "WARNING: logind lingering is off for $(id -un): topics stop 10 s after the last" \
         "ssh logout. Fix once: sudo loginctl enable-linger $(id -un)" | tee "$OUT/linger_warning.txt"
fi

LAUNCH_PID=""
WEB_PID=""
MEM_PID=""
CAM_PID=""
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
    stop_group "$CAM_PID"
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

setsid ros2 run wro2026_camera_c70 wall_node --ros-args -p image_rate:=5.0 \
    > "$OUT/camera.log" 2>&1 < /dev/null &
CAM_PID=$!
for _ in $(seq 40); do grep -q "READ ONLY" "$OUT/camera.log" && break; sleep 0.25; done
if ! grep -q "READ ONLY" "$OUT/camera.log" || ! kill -0 "$CAM_PID" 2>/dev/null; then
    echo "camera wall node did not start, see $OUT/camera.log"
    exit 1
fi
setsid ros2 run wro2026_lidar_n10p scan_memory_node > "$OUT/memory.log" 2>&1 < /dev/null &
MEM_PID=$!
POLICY_ARGS=(-p policy:=corner)
if [ -n "$MODEL" ]; then
    cp "$MODEL" "$OUT/model.npz"        # which network drove this run
    POLICY_ARGS=(-p policy:=model -p model:="$OUT/model.npz")
fi
setsid ros2 run wro2026_imitation bc_node --ros-args "${POLICY_ARGS[@]}" \
    -p direction:="$DIRECTION" -p corner_ahead:="$CORNER_AHEAD" -p timing:="$TIMING" \
    -p laps:="$LAPS" \
    -p speed:="$SPEED" -p publish_cmd:=true \
    -p cmd_topic:=bc/cmd_vel -p stop_margin:=0.0 -p memory_marks:=true \
    -p predict_stop:=true \
    -r scan:=scan_filled \
    > "$OUT/bc.log" 2>&1 < /dev/null &
BC_PID=$!
for _ in $(seq 60); do grep -q "PUBLISHING" "$OUT/bc.log" && break; sleep 0.25; done
if ! grep -q "PUBLISHING" "$OUT/bc.log"; then
    echo "bc_node did not start, see $OUT/bc.log"
    exit 1
fi

TOKEN_ARG=()
if [ -n "${DRIVE_TOKEN:-}" ]; then TOKEN_ARG=(-p "token:=$DRIVE_TOKEN"); fi
setsid ros2 run wro2026_base_r550 web_teleop_node --ros-args \
    -p speed:="$SPEED" -p max_speed:="$SPEED" -p https:=true "${TOKEN_ARG[@]}" \
    -p auto_topic:=bc/cmd_vel > "$OUT/web.log" 2>&1 < /dev/null &
WEB_PID=$!
for _ in $(seq 80); do grep -q "phone page" "$OUT/web.log" && break; sleep 0.25; done
TOKEN=$(grep -o 'k=[0-9A-Za-z]*' "$OUT/web.log" | head -n 1)
if [ -z "$TOKEN" ]; then
    echo "phone page did not start, see $OUT/web.log"
    exit 1
fi
for ip in $(hostname -I); do
    case "$ip" in *:*) ;; *) echo "PHONE_URL https://$ip:8080/?$TOKEN" ;; esac
done
echo "READY: open the 自動 tab and HOLD the button to drive. Time limit $RUN_TIME s."

END=$((SECONDS + RUN_TIME + 5))
# Evidence for the hotspot stall (11-20; LOCALHOST above is the fix, not yet tried on the
# hotspot): net.log gets a line with the time whenever the network addresses change.
NET_LAST=""
while [ "$SECONDS" -lt "$END" ]; do
    NET_NOW=$(ip -br addr 2>/dev/null | grep -v '^lo ')
    if [ "$NET_NOW" != "$NET_LAST" ]; then
        { date '+%F %T.%N'; echo "$NET_NOW"; } >> "$OUT/net.log"
        NET_LAST="$NET_NOW"
    fi
    if [ -e "$OUT/stop" ]; then echo "stop requested"; break; fi
    if ! kill -0 "$WEB_PID" 2>/dev/null; then echo "phone page ended"; break; fi
    if ! kill -0 "$BC_PID" 2>/dev/null; then echo "bc_node ended"; break; fi
    if ! kill -0 "$CAM_PID" 2>/dev/null; then echo "camera wall node ended"; break; fi
    if ! kill -0 "$MEM_PID" 2>/dev/null; then echo "scan memory ended"; break; fi
    if [ "$(count "$LAUNCH_PID")" -eq 0 ]; then echo "launch ended"; break; fi
    sleep 1
done
exit 0
