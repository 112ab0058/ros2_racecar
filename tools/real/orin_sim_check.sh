#!/usr/bin/env bash
# Orin: SIMULATION check of wro2026_sim_r550 (Gazebo). No hardware is touched:
# the real chassis / lidar nodes must NOT be running (the script refuses).
#
#   bash ~/racecar_ws/tools/orin_sim_check.sh          # ~3 min, no questions
#
# Uses ROS_DOMAIN_ID=42 so nothing here can reach a real base_node.
# 1. starts sim.launch.py (headless Gazebo, EKF, no Foxglove)
# 2. real-time factor, CPU, every topic the car has (/scan 480 beams in laser_link,
#    /odom, /imu/data, /odometry/filtered, /clock, /sim/ground_truth), TF
# 3. drive_test_node: scripted course -> odom / EKF vs true pose, stop, no spin
# Result: ~/r550_sim/<time>/report.txt (+ drive_test.txt/.csv, launch.log)
set -o pipefail
export ROS_DOMAIN_ID=42

OUT="$HOME/r550_sim/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
REPORT="$OUT/report.txt"
say() { echo "$*" | tee -a "$REPORT"; }
section() { say; say "===== $* ====="; }
run() { say "\$ $*"; timeout 20 "$@" < /dev/null 2>&1 | tee -a "$REPORT"; }

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

if pgrep -af "wro2026_base_r550/(base_node|feedback_node)|wro2026_lidar_n10p/scan_node"; then
    say "REAL car nodes are running: stop them first"; exit 1
fi
if pgrep -f "gz sim" > /dev/null; then
    say "a Gazebo is already running: stop it first"; pgrep -af "gz sim" | tee -a "$REPORT"; exit 1
fi

section "start sim.launch.py (log: launch.log), wait 30 s"
setsid ros2 launch wro2026_sim_r550 sim.launch.py foxglove:=false > "$OUT/launch.log" 2>&1 \
    < /dev/null &
LP=$!
stop_sim() {
    kill -INT -- "-$LP" 2>/dev/null
    for _ in $(seq 40); do kill -0 "$LP" 2>/dev/null || break; sleep 0.5; done
    kill -TERM -- "-$LP" 2>/dev/null
    pkill -TERM -f "gz sim" 2>/dev/null
}
trap stop_sim EXIT
sleep 30

section "errors in launch.log"
grep -iE "error|exception|failed|could not" "$OUT/launch.log" | head -n 20 | tee -a "$REPORT"

section "real-time factor (1.0 = as fast as real time)"
for _ in 1 2 3; do
    timeout 5 gz topic -e -t /world/r550_field/stats -n 1 2>/dev/null \
        | grep -m1 real_time_factor | tee -a "$REPORT"
    sleep 1
done

section "CPU (top, 1 sample)"
top -b -n 1 | head -n 20 | tee -a "$REPORT"

section "topics (expected like the car: /odom 20, /imu/data 20, /scan 10, /odometry/filtered 30 Hz)"
run ros2 topic list
for t in "/clock rosgraph_msgs/msg/Clock" "/odom nav_msgs/msg/Odometry" \
         "/imu/data sensor_msgs/msg/Imu" "/imu/data_raw sensor_msgs/msg/Imu" \
         "/odometry/filtered nav_msgs/msg/Odometry" "/sim/ground_truth nav_msgs/msg/Odometry"; do
    set -- $t
    if timeout 10 ros2 topic echo --once "$1" "$2" > /dev/null 2>&1 < /dev/null; then
        say "OK      $1"; else say "MISSING $1"; fi
done
for t in /odom /imu/data /scan /odometry/filtered; do
    say "\$ ros2 topic hz $t (6 s, wall clock)"
    timeout -s INT 6 ros2 topic hz "$t" < /dev/null 2>&1 | tail -n 3 | tee -a "$REPORT"
done

section "one /scan: frame, beams, valid"
timeout 10 ros2 topic echo --once /scan sensor_msgs/msg/LaserScan --full-length < /dev/null \
    > "$OUT/scan_once.yaml" 2>&1
python3 - "$OUT/scan_once.yaml" <<'PY' 2>&1 | tee -a "$REPORT"
import math
import sys

import yaml

msg = yaml.safe_load(open(sys.argv[1]).read().split('\n---')[0])
r = [float(v) for v in msg['ranges']]
ok = [v for v in r if math.isfinite(v)]
print('frame_id', msg['header']['frame_id'], ' beams', len(r), ' (expect laser_link, 480)')
print('valid', len(ok), ' min %.3f  max %.3f m' % (min(ok), max(ok)) if ok else 'no valid ranges')
PY

section "TF odom -> laser_link (EKF publishes odom -> base_link)"
timeout -s INT 4 ros2 run tf2_ros tf2_echo odom laser_link < /dev/null 2>&1 \
    | grep -m 2 -E "Translation|Exception" | tee -a "$REPORT"

section "drive test (scripted course in the simulation, up to 120 s)"
timeout 120 ros2 run wro2026_sim_r550 drive_test_node --ros-args -p use_sim_time:=true \
    -p out:="$OUT" < /dev/null > "$OUT/drive_test.log" 2>&1
cat "$OUT/drive_test.txt" 2>/dev/null | tee -a "$REPORT" || say "no drive_test.txt (see drive_test.log)"

stop_sim
trap - EXIT
sleep 2
section "left running after stop (expected nothing)"
pgrep -af "gz sim|wro2026_sim_r550|ekf_node|parameter_bridge|robot_state_publisher" \
    | tee -a "$REPORT" || say "nothing"
say "RESULT_DIR=$OUT"
