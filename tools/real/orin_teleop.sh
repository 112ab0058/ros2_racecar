#!/usr/bin/env bash
# Orin: KEYBOARD TELEOP of the R550. MAKES THE MOTORS TURN.
#
#   bash ~/racecar_ws/tools/orin_teleop.sh             # max 0.15 m/s, 300 s
#   bash ~/racecar_ws/tools/orin_teleop.sh 0.1 120     # max_vx 0.1, 120 s
#
# One terminal: starts base.launch (commands enabled, limits below) in the
# background, runs the keyboard node in front, and stops everything when you
# press Ctrl+C. Run it yourself in ssh (it reads your keys).
#
# FOR NOW: WHEELS IN THE AIR ONLY. The STM32 has no command timeout; driving
# on the floor waits for that decision (docs/STATUS.md).
#
# Limits: max_vx (default 0.15 m/s), reverse 0.15, cmd_timeout 0.3 s,
#         max_run_time (default 300 s after the first key), turn radius 0.5 m.
# Stop:   let go of the key (0 within ~0.25 s, wheels stop ~1 s later),
#         s / space, Ctrl+C (quits), or the Motor switch OFF.
# Result: ~/r550_teleop/<time>/drive.csv : t (arrival, s), topic, vx, wz for
#         /cmd_vel (keys), /cmd_vel_out (sent), /odom (wheels); base.log.
#         Release -> stop = first 0 on cmd_vel -> odom vx back to 0.
set -o pipefail

MAX_VX="${1:-0.15}"
RUN_TIME="${2:-300}"
OUT="$HOME/r550_teleop/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

if pgrep -f "wro2026_base_r550/(feedback_node|base_node)|wro2026_lidar_n10p|robot_state_publisher" > /dev/null; then
    echo "Other ROS programs are running. Stop them first (Ctrl+C in their terminals):"
    pgrep -af "feedback_node|base_node|scan_node|robot_state_publisher"
    exit 1
fi
if awk "BEGIN{exit !($MAX_VX > 0.15)}"; then
    echo "max_vx $MAX_VX is above 0.15 m/s: not allowed in this script."
    exit 1
fi

echo "Keyboard teleop: max_vx $MAX_VX m/s, max_run_time $RUN_TIME s, cmd_timeout 0.3 s"
echo "Stop: let go of the key / s / space / Ctrl+C / Motor switch OFF"
read -r -p "Wheels off the ground, hand at the Motor switch? Type YES: " a
[ "$a" = YES ] || exit 0

PIDS=()
cleanup() {
    for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
    sleep 2
    for p in "${PIDS[@]}"; do kill -TERM -- "-$p" 2>/dev/null; done
    echo "stopped. RESULT_DIR=$OUT"
}
trap cleanup EXIT

setsid ros2 launch wro2026_base_r550 base.launch.py enable_commands:=true \
    max_vx:="$MAX_VX" max_run_time:="$RUN_TIME" > "$OUT/base.log" 2>&1 < /dev/null &
PIDS+=($!)
# One recorder for all three topics, with arrival times (echo --csv gave up
# on 2026-10-19 because the topics did not exist yet, and Twist has no stamp).
setsid ros2 run wro2026_base_r550 record_node --ros-args -p out:="$OUT/drive.csv" \
    > "$OUT/record.log" 2>&1 < /dev/null &
PIDS+=($!)
echo "starting the base node (6 s) ..."
sleep 6
if ! grep -q "COMMANDS ENABLED" "$OUT/base.log"; then
    echo "base node did not start, see $OUT/base.log"
    exit 1
fi

trap ':' INT   # Ctrl+C reaches the teleop node (it publishes 0 and quits), then cleanup runs
ros2 run wro2026_base_r550 teleop_node --ros-args -p max_speed:="$MAX_VX" \
    -p speed:="$(awk "BEGIN{print ($MAX_VX < 0.1) ? $MAX_VX : 0.1}")"
