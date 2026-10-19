#!/usr/bin/env bash
# Orin: READ-ONLY calibration recordings (odom + IMU). The motors cannot turn:
# nothing here publishes /cmd_vel or writes to the STM32. No sudo.
#
#   bash ~/racecar_ws/tools/orin_calib.sh static [60]   # car still 60 s (no questions:
#                                                         Claude Code can run this one)
#   bash ~/racecar_ws/tools/orin_calib.sh straight      # you PUSH the car straight
#   bash ~/racecar_ws/tools/orin_calib.sh turn          # you TURN the car LEFT by hand
#
# straight / turn: keep the car still for the first 3 s (gyro bias), then push,
# then press Enter. Motor switch OFF so the wheels roll freely.
# Result: ~/r550_calib/<time>_<test>/ odom.csv, imu.csv, analysis_<test>.txt
# (analysis by analyze_calib.py, also runnable later on Windows).
set -o pipefail

TEST="$1"
case "$TEST" in static|straight|turn) ;; *)
    echo "usage: orin_calib.sh static [seconds] | straight | turn"; exit 1 ;; esac
SECONDS_STILL="${2:-60}"
OUT="$HOME/r550_calib/$(date +%Y%m%d_%H%M%S)_$TEST"
mkdir -p "$OUT"
TOOLS="$(dirname "$0")"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

if pgrep -f "wro2026_base_r550/base_node" > /dev/null; then
    echo "base_node (command side) is running: stop it first. This script is read only."
    exit 1
fi
n=0
for pat in "wro2026_base_r550/feedback_node" "wro2026_lidar_n10p/scan_node" "robot_state_publisher"; do
    pgrep -f "$pat" > /dev/null && n=$((n + 1))
done
PIDS=()
if [ "$n" -gt 0 ] && [ "$n" -lt 3 ]; then
    echo "Some ROS programs are running: stop them (or start all of sensors.launch.py)."
    exit 1
elif [ "$n" -eq 0 ]; then
    setsid ros2 launch wro2026_bringup sensors.launch.py foxglove:=false \
        > "$OUT/launch.log" 2>&1 < /dev/null &
    PIDS+=($!)
fi
setsid ros2 run wro2026_base_r550 sensor_log_node --ros-args -p dir:="$OUT" \
    > "$OUT/log.log" 2>&1 < /dev/null &
LOG_PID=$!
cleanup() {
    kill -INT -- "-$LOG_PID" 2>/dev/null
    sleep 1
    for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
    sleep 2
    for p in "$LOG_PID" "${PIDS[@]}"; do kill -TERM -- "-$p" 2>/dev/null; done
}
trap cleanup EXIT
echo "starting sensors (8 s) ..."
sleep 8

DIST=""
TURNS=""
if [ "$TEST" = static ]; then
    echo "RECORDING $SECONDS_STILL s - do not touch the car or the table"
    sleep "$SECONDS_STILL"
else
    echo "RECORDING. Keep the car STILL for 3 s ..."
    sleep 3
    if [ "$TEST" = straight ]; then
        echo "NOW push the car slowly STRAIGHT along the tape measure, stop, then press Enter."
    else
        echo "NOW turn the car LEFT (counter-clockwise seen from above) by hand,"
        echo "whole turns if you can, keep the rear wheels rolling, then press Enter."
    fi
    read -r _
    sleep 1
    if [ "$TEST" = straight ]; then
        read -r -p "Distance on the tape (m), e.g. 2.00: " DIST
    else
        read -r -p "How many full LEFT turns (e.g. 1, or Enter if unsure): " TURNS
    fi
fi
cleanup
trap - EXIT

ARGS=()
[ -n "$DIST" ] && ARGS+=(--distance "$DIST")
[ -n "$TURNS" ] && ARGS+=(--turns "$TURNS")
python3 "$TOOLS/analyze_calib.py" "$TEST" "$OUT" "${ARGS[@]}"
echo "RESULT_DIR=$OUT"
