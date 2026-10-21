#!/usr/bin/env bash
# Orin: START / STOP CYCLE TEST of localization.launch.py - read only (motors cannot turn).
#
#   bash ~/racecar_ws/tools/orin_cycle.sh [cycles]       # default 30 (~15 min)
#
# Each cycle: start localization.launch.py (no Foxglove) -> time until
# /odometry/filtered arrives -> check /odom, /imu/data_raw, /scan each send one
# message -> Ctrl+C (SIGINT) -> time until everything has exited -> anything
# left running? -> both serial devices still there?
# Finds: slow or flaky start-up, serial port not released, nodes that do not
# exit on Ctrl+C (the same path base_node uses to send its stop frames).
# Stops after 3 failed cycles in a row. Result: ~/r550_cycle/<time>/cycles.csv
# and summary.txt. The car must not move: Motor switch OFF.
set -o pipefail

CYCLES="${1:-30}"
OUT="$HOME/r550_cycle/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
PATTERN="wro2026_base_r550/|wro2026_lidar_n10p/|robot_state_publisher|ekf_node|ros2 launch wro2026_bringup"
STM32=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
LIDAR=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

if pgrep -f "$PATTERN" > /dev/null; then
    echo "ROS programs are already running; stop them first:"; pgrep -af "$PATTERN"; exit 1
fi

echo "cycle,startup_s,topics_ok,shutdown_s,left_running,serial_ok,ok" > "$OUT/cycles.csv"
fails=0
for i in $(seq "$CYCLES"); do
    t0=$(date +%s.%N)
    setsid ros2 launch wro2026_bringup localization.launch.py foxglove:=false \
        > "$OUT/launch_$i.log" 2>&1 < /dev/null &
    lp=$!
    startup=NA
    # the message TYPE makes echo wait for the topic (without it echo gives up at
    # once: "Could not determine the type", 2026-10-20 first run)
    if timeout 40 ros2 topic echo --once /odometry/filtered nav_msgs/msg/Odometry \
            --field header.stamp > /dev/null 2>&1 < /dev/null; then
        startup=$(awk "BEGIN{printf \"%.1f\", $(date +%s.%N) - $t0}")
    fi
    topics=0
    for tt in "/odom nav_msgs/msg/Odometry" "/imu/data_raw sensor_msgs/msg/Imu" \
              "/scan sensor_msgs/msg/LaserScan"; do
        set -- $tt
        timeout 8 ros2 topic echo --once "$1" "$2" --field header.stamp > /dev/null 2>&1 \
            < /dev/null && topics=$((topics + 1))
    done
    t1=$(date +%s.%N)
    kill -INT -- "-$lp" 2>/dev/null
    shutdown=NA
    for _ in $(seq 60); do
        if ! pgrep -f "$PATTERN" > /dev/null; then
            shutdown=$(awk "BEGIN{printf \"%.1f\", $(date +%s.%N) - $t1}"); break
        fi
        sleep 0.25
    done
    left=$(pgrep -f "$PATTERN" | wc -l)
    if [ "$left" -gt 0 ]; then
        pgrep -af "$PATTERN" > "$OUT/left_$i.txt"
        pkill -TERM -f "$PATTERN"; sleep 2; pkill -KILL -f "$PATTERN"
    fi
    kill -TERM -- "-$lp" 2>/dev/null
    serial=0
    [ -e "$STM32" ] && [ -e "$LIDAR" ] && serial=1
    ok=0
    [ "$startup" != NA ] && [ "$topics" = 3 ] && [ "$left" = 0 ] && [ "$serial" = 1 ] && ok=1
    echo "$i,$startup,$topics,$shutdown,$left,$serial,$ok" | tee -a "$OUT/cycles.csv"
    if [ "$ok" = 1 ]; then fails=0; else fails=$((fails + 1)); fi
    if [ "$fails" -ge 3 ]; then echo "3 failed cycles in a row: stopping"; break; fi
    sleep 3
done

python3 - "$OUT/cycles.csv" <<'PY' | tee "$OUT/summary.txt"
import csv
import statistics as st
import sys

rows = list(csv.DictReader(open(sys.argv[1])))
ok = [r for r in rows if r['ok'] == '1']
up = sorted(float(r['startup_s']) for r in rows if r['startup_s'] != 'NA')
down = sorted(float(r['shutdown_s']) for r in rows if r['shutdown_s'] != 'NA')
print('cycles %d, ok %d, failed %d' % (len(rows), len(ok), len(rows) - len(ok)))
if up:
    print('start-up to first /odometry/filtered: min %.1f  median %.1f  max %.1f s'
          % (up[0], st.median(up), up[-1]))
if down:
    print('Ctrl+C to all exited: min %.1f  median %.1f  max %.1f s'
          % (down[0], st.median(down), down[-1]))
print('cycles with programs left running: %d' % sum(1 for r in rows if r['left_running'] != '0'))
print('cycles with a serial device missing: %d' % sum(1 for r in rows if r['serial_ok'] != '1'))
print('cycles with < 3 topics: %d' % sum(1 for r in rows if r['topics_ok'] != '3'))
print('VERDICT', 'PASS' if len(ok) == len(rows) else 'WARN')
PY
echo "RESULT_DIR=$OUT"
