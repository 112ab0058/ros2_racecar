#!/usr/bin/env bash
# Orin: SOAK TEST - everything read only (motors cannot turn), car standing still.
#
#   bash ~/racecar_ws/tools/orin_soak.sh [minutes]        # default 120
#   nohup setsid bash ~/racecar_ws/tools/orin_soak.sh 120 > ~/soak.out 2>&1 &   # survives ssh
#
# Runs localization.launch.py (sensors + EKF, no Foxglove) and soak_log_node for
# <minutes>, and every 60 s records CPU time / memory of each node, temperatures,
# free memory and the battery. Stops early (and says why) when:
#   - the battery is below 11.0 V          - a node died
#   - the file ~/r550_soak/STOP exists     (touch it to stop cleanly)
# At the end: stops everything, checks nothing is left running, runs
# analyze_soak.py. Result: ~/r550_soak/<time>/report_soak.txt (+ csv files).
# The car must not move: Motor switch OFF, nobody touches the car or the table.
set -o pipefail

MINUTES="${1:-120}"
ROOT="$HOME/r550_soak"
OUT="$ROOT/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
rm -f "$ROOT/STOP"
TOOLS="$(dirname "$0")"
LOG="$OUT/soak.log"
say() { echo "$(date +%T) $*" | tee -a "$LOG"; }

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

if pgrep -f "wro2026_base_r550/|wro2026_lidar_n10p/|robot_state_publisher|ekf_node" > /dev/null; then
    say "ROS programs are already running; stop them first:"
    pgrep -af "wro2026_base_r550/|wro2026_lidar_n10p/|robot_state_publisher|ekf_node" | tee -a "$LOG"
    exit 1
fi

say "soak test $MINUTES min -> $OUT"
setsid ros2 launch wro2026_bringup localization.launch.py foxglove:=false \
    > "$OUT/launch.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
sleep 15
setsid ros2 run wro2026_base_r550 soak_log_node --ros-args -p dir:="$OUT" \
    > "$OUT/logger.log" 2>&1 < /dev/null &
LOGGER_PID=$!
sleep 5

stop_all() {
    kill -INT -- "-$LOGGER_PID" 2>/dev/null; sleep 2
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 30); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    kill -TERM -- "-$LAUNCH_PID" "-$LOGGER_PID" 2>/dev/null
    sleep 1
}
trap 'stop_all' EXIT

# name in the report -> pgrep pattern
declare -A NODES=(
    [r550_feedback]="wro2026_base_r550/feedback_node"
    [n10p_scan]="wro2026_lidar_n10p/scan_node"
    [ekf_filter_node]="robot_localization/ekf_node"
    [robot_state_publisher]="robot_state_publisher/robot_state_publisher"
    [soak_log_node]="wro2026_base_r550/soak_log_node"
)
echo "t,name,cpu_s,rss_kb" > "$OUT/resources.csv"
echo "t,zone,temp_c" > "$OUT/thermal.csv"
echo "t,mem_available_mb" > "$OUT/mem.csv"

REASON="finished"
END=$(( $(date +%s) + MINUTES * 60 ))
while [ "$(date +%s)" -lt "$END" ]; do
    now=$(date +%s)
    for name in "${!NODES[@]}"; do
        pid=$(pgrep -f "${NODES[$name]}" | head -1)
        if [ -z "$pid" ]; then
            REASON="node died: $name"; break 2
        fi
        read -r ctime rss < <(ps -o times=,rss= -p "$pid")   # CPU seconds, KB
        echo "$now,$name,$ctime,$rss" >> "$OUT/resources.csv"
    done
    for z in /sys/class/thermal/thermal_zone*; do
        # some zones (cv0/cv1/cv2-thermal) answer "No data available": skip them
        raw=$(cat "$z/temp" 2>/dev/null) || continue
        [ -n "$raw" ] || continue
        echo "$now,$(cat "$z/type"),$(awk -v r="$raw" 'BEGIN{printf "%.1f", r/1000}')" \
            >> "$OUT/thermal.csv"
    done
    echo "$now,$(awk '/MemAvailable/ {printf "%d", $2/1024}' /proc/meminfo)" >> "$OUT/mem.csv"
    v=$(tail -n 1 "$OUT/battery.csv" 2>/dev/null | cut -d, -f2)
    left=$(( (END - now) / 60 ))
    say "running, ${left} min left, battery ${v:-?} V, imu rows $(($(wc -l < "$OUT/imu_raw.csv") - 1))"
    if [ -n "$v" ] && awk "BEGIN{exit !($v < 11.0)}"; then REASON="battery below 11.0 V ($v)"; break; fi
    if [ -e "$ROOT/STOP" ]; then REASON="STOP file"; rm -f "$ROOT/STOP"; break; fi
    sleep 60
done

say "stopping: $REASON"
stop_all
trap - EXIT
left=$(pgrep -af "wro2026_base_r550/|wro2026_lidar_n10p/|robot_state_publisher|ekf_node")
if [ -n "$left" ]; then say "LEFT RUNNING after stop:"; echo "$left" | tee -a "$LOG"; else say "nothing left running"; fi
python3 "$TOOLS/analyze_soak.py" "$OUT" | tee -a "$LOG"
say "REASON=$REASON"
say "RESULT_DIR=$OUT"
