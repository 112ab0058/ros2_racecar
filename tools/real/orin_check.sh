#!/usr/bin/env bash
# Orin: READ-ONLY health check of the car's ROS system. Writes everything to
# ~/r550_check/<time>/ so it can be copied back and read later.
#
#   bash ~/racecar_ws/tools/orin_check.sh            # checks only (~1 min)
#   bash ~/racecar_ws/tools/orin_check.sh --bag 20   # + record 20 s rosbag
#   bash ~/racecar_ws/tools/orin_check.sh --ekf      # localization.launch.py (+ EKF):
#                                                    # filtered rate, who publishes the
#                                                    # odom TF, 60 s drift while still
#
# If sensors.launch.py is not running, this script starts it (without
# Foxglove) and stops it at the end. Nothing here writes to the STM32 or
# publishes /cmd_vel: the motors cannot turn. No sudo.
set -o pipefail

BAG_SECONDS=0
EKF=0
LAUNCH_FILE=sensors.launch.py
if [ "$1" = "--bag" ]; then BAG_SECONDS="${2:-20}"; fi
if [ "$1" = "--ekf" ]; then EKF=1; LAUNCH_FILE=localization.launch.py; fi

STM32=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
LIDAR=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00
OUT="$HOME/r550_check/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
REPORT="$OUT/report.txt"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

say() { echo "$*" | tee -a "$REPORT"; }
section() { say; say "===== $* ====="; }
run() { say "\$ $*"; timeout 15 "$@" < /dev/null 2>&1 | tee -a "$REPORT"; }

section "time, battery of the Orin side"
run date
run uptime

section "serial devices (STM32 = ...0002, lidar = ...0001)"
run ls -l /dev/serial/by-id/
for d in "$STM32" "$LIDAR"; do
    if [ -e "$d" ]; then say "OK   $d"; else say "MISSING $d"; fi
done

LAUNCH_PID=""
# All three must be running, or none. A leftover single node (e.g. an old
# scan.launch) gave wrong results on 2026-10-19: no /odom, 720-bin scan.
n_running=0
PATTERNS=("wro2026_base_r550/(feedback_node|base_node)" "wro2026_lidar_n10p/scan_node"
          "robot_state_publisher")
[ "$EKF" = 1 ] && PATTERNS+=("robot_localization/ekf_node")
for pat in "${PATTERNS[@]}"; do
    pgrep -f "$pat" > /dev/null && n_running=$((n_running + 1))
done
if [ "$EKF" = 1 ] && [ "$n_running" -eq 3 ]; then
    section "sensors are running WITHOUT the EKF: stop them, then run --ekf again"
    exit 1
fi
if [ "$n_running" -gt 0 ] && [ "$n_running" -lt 3 ]; then
    section "SOME ROS programs are already running: stop them first, then run again"
    run pgrep -af "feedback_node|base_node|scan_node|robot_state_publisher|foxglove_bridge"
    exit 1
elif [ "$n_running" -ge 3 ]; then
    section "sensors already running (not started by this script)"
    run pgrep -af "scan_node|feedback_node|base_node|robot_state_publisher|foxglove_bridge"
else
    section "starting $LAUNCH_FILE foxglove:=false (log: launch.log)"
    setsid ros2 launch wro2026_bringup "$LAUNCH_FILE" foxglove:=false \
        > "$OUT/launch.log" 2>&1 < /dev/null &
    LAUNCH_PID=$!
    sleep 8
fi

stop_launch() {
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.5
        done
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
        LAUNCH_PID=""
    fi
}
trap stop_launch EXIT

section "nodes and topics"
run ros2 node list
run ros2 topic list -t

section "rates (expected: /odom 20, /imu/data 20, /scan 10 Hz)"
for t in /odom /imu/data /scan; do
    say "\$ ros2 topic hz $t (6 s)"
    timeout -s INT 6 ros2 topic hz "$t" < /dev/null 2>&1 | tail -n 5 | tee -a "$REPORT"
done

section "battery and diagnostics"
run ros2 topic echo --once /battery --field voltage
run ros2 topic echo --once /diagnostics

section "TF base_link -> laser_link (expected 0.130 0.000 0.115)"
say "\$ ros2 run tf2_ros tf2_echo base_link laser_link (4 s)"
timeout -s INT 4 ros2 run tf2_ros tf2_echo base_link laser_link < /dev/null 2>&1 \
    | grep -m 3 -E "Translation|Rotation|Exception" | tee -a "$REPORT"
say "\$ ros2 run tf2_ros tf2_echo odom laser_link (4 s)"
timeout -s INT 4 ros2 run tf2_ros tf2_echo odom laser_link < /dev/null 2>&1 \
    | grep -m 3 -E "Translation|Rotation|Exception" | tee -a "$REPORT"

section "one /scan message: size and valid points"
timeout 10 ros2 topic echo --once /scan --full-length < /dev/null > "$OUT/scan_once.yaml" 2>&1
python3 - "$OUT/scan_once.yaml" <<'PY' 2>&1 | tee -a "$REPORT"
import math
import sys

import yaml

with open(sys.argv[1]) as f:
    msg = yaml.safe_load(f.read().split('\n---')[0])
r = [float(v) for v in msg['ranges']]
ok = [v for v in r if math.isfinite(v)]
print('frame_id', msg['header']['frame_id'], ' bins', len(r),
      ' increment %.4f rad' % msg['angle_increment'])
print('valid', len(ok), ' inf', len(r) - len(ok),
      ' min %.3f  max %.3f m' % (min(ok), max(ok)) if ok else '')
PY

if [ "$EKF" = 1 ]; then
    section "EKF (expected: /odometry/filtered 30 Hz; r550_feedback publish_odom_tf False)"
    say "\$ ros2 topic hz /odometry/filtered (6 s)"
    timeout -s INT 6 ros2 topic hz /odometry/filtered < /dev/null 2>&1 | tail -n 5 | tee -a "$REPORT"
    run ros2 param get /r550_feedback publish_odom_tf
    say "who publishes /tf (only ekf_filter_node and robot_state_publisher's static parts expected):"
    run ros2 topic info /tf --verbose
    run ros2 topic echo --once /diagnostics --field status
    section "EKF drift while still: pose now and after 60 s (do not touch the car)"
    pose() { timeout 5 ros2 topic echo --once /odometry/filtered --field pose.pose < /dev/null 2>&1; }
    pose | tee "$OUT/pose_t0.txt" | tee -a "$REPORT"
    sleep 60
    pose | tee "$OUT/pose_t60.txt" | tee -a "$REPORT"
fi

if [ "$BAG_SECONDS" -gt 0 ]; then
    section "rosbag ${BAG_SECONDS} s: /odom /scan /imu/data /tf /tf_static (push the car by hand now)"
    for i in 5 4 3 2 1; do say "starting in $i ..."; sleep 1; done
    say "RECORDING - push the car forward slowly"
    timeout -s INT "$BAG_SECONDS" ros2 bag record -o "$OUT/bag" \
        /odom /scan /imu/data /tf /tf_static < /dev/null > "$OUT/bag.log" 2>&1
    say "recording stopped"
    run ros2 bag info "$OUT/bag"
fi

stop_launch
section "done"
say "RESULT_DIR=$OUT"
