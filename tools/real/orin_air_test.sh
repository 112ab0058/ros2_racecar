#!/usr/bin/env bash
# Orin: WHEELS-IN-THE-AIR test of the R550 command side. MAKES THE MOTORS TURN.
#
#   bash ~/racecar_ws/tools/orin_air_test.sh
#
# Run it yourself in an ssh terminal (it asks questions; Claude Code cannot
# answer them or see the wheels). Before running:
#   - car on a box, ALL FOUR WHEELS OFF THE GROUND, nothing touching them
#   - only the STM32 USB cable needed; no other ROS program running
#   - your hand next to the red Motor switch (hardware stop)
# Limits: max_vx 0.1 m/s (real), commands 2-8 s per step, cmd_timeout 0.3 s.
# Stop at any time: Ctrl+C (stops the node, which sends stop frames), or
# Motor switch OFF. The STM32 itself has NO command timeout (balance.c).
# Results: ~/r550_airtest/<time>/report.txt + csv of /cmd_vel_out and /odom.
#
# 2026-10-19 lessons (Claude Code run): `ros2 topic pub` needs ~1.5 s to start,
# so pub() adds 1.5 s; recording starts 2 s before the commands; 0.05 m/s may
# not turn the wheels, so 0.08 m/s is used; max_run_time counts from the first
# /cmd_vel (CommandGate); step 10 (kill -9) is not run: firmware has no timeout.
set -o pipefail

MAX_VX=0.1
V=0.08          # test speed (m/s)
PUB_START=1.5   # s, ros2 topic pub start-up
OUT="$HOME/r550_airtest/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
REPORT="$OUT/report.txt"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

log() { echo "$*" | tee -a "$REPORT"; }
add() { awk "BEGIN{print $1 + $2}"; }

LAUNCH_PID=""
start_base() {
    setsid ros2 launch wro2026_base_r550 base.launch.py enable_commands:=true \
        max_vx:="$MAX_VX" "$@" > "$OUT/launch_step$STEP.log" 2>&1 < /dev/null &
    LAUNCH_PID=$!
    log "  (base node starting: max_vx $MAX_VX $*)"
    sleep 6
}
stop_base() {
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.25; done
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
        LAUNCH_PID=""
    fi
}
cleanup() { pkill -INT -f "ros2 topic pub" 2>/dev/null; pkill -INT -f "ros2 topic echo" 2>/dev/null; stop_base; }
trap cleanup EXIT
trap 'echo; log "Ctrl+C: stopping"; exit 130' INT

pub() {  # seconds-of-commands vx wz  (blocks)
    timeout -s INT "$(add "$1" "$PUB_START")" ros2 topic pub -r 10 /cmd_vel \
        geometry_msgs/msg/Twist "{linear: {x: $2}, angular: {z: $3}}" > /dev/null 2>&1 < /dev/null
}
record() {  # seconds-of-commands: background csv, starts 2 s early, ends 3 s after
    local d; d=$(add "$1" "$(add "$PUB_START" 5)")
    timeout -s INT "$d" ros2 topic echo --csv /cmd_vel_out > "$OUT/step${STEP}_cmd_out.csv" \
        2>/dev/null < /dev/null &
    timeout -s INT "$d" ros2 topic echo --csv /odom > "$OUT/step${STEP}_odom.csv" \
        2>/dev/null < /dev/null &
    sleep 2
}
drive() {  # seconds vx wz: record + publish, then wait for the wheels to stop
    record "$1"; pub "$1" "$2" "$3"; sleep 3
}
reason() {  # last command-gate decision from /diagnostics
    local r
    r=$(timeout 4 ros2 topic echo /diagnostics 2>/dev/null < /dev/null \
        | grep -A1 "key: reason" | grep -m1 "value" | sed "s/.*value: //")
    log "  gate reason now: ${r:-unknown}"
}
ask() { read -r -p "$1  [Enter = go, q = quit] " a; [ "$a" = q ] && exit 0; }
observe() { read -r -p "  >> $1 (y / n / note): " a; log "  step $STEP: $1 -> $a"; }

if pgrep -f "wro2026_base_r550/(feedback_node|base_node)|wro2026_lidar_n10p|robot_state_publisher|foxglove_bridge" > /dev/null; then
    echo "Other ROS programs are running. Stop them first (Ctrl+C in their terminals):"
    pgrep -af "feedback_node|base_node|scan_node|robot_state_publisher|foxglove_bridge"
    exit 1
fi

log "R550 air test $(date)"
log "limits: max_vx $MAX_VX m/s, test speed $V m/s, cmd_timeout 0.3 s; stop = Ctrl+C or Motor switch OFF"
read -r -p "Wheels off the ground, hand at the Motor switch? Type YES: " a
[ "$a" = YES ] || exit 0

STEP=1
log; log "== 1. Motor switch OFF: commands are sent but nothing may move =="
ask "Set the Motor switch to OFF."
start_base
drive 3 "$V" 0.0
observe "wheels did NOT move (Motor OFF); OLED shows ROS OFF"

STEP=2
log; log "== 2. forward $V m/s for 3 s, then the publisher stops =="
ask "Set the Motor switch to ON (wheels in the air)."
drive 3 "$V" 0.0
observe "REAR wheels turned FORWARD (both sides), then stopped about 1 s after"

STEP=3
log; log "== 3. reverse -$V m/s for 3 s =="
ask "Next: reverse."
drive 3 "-$V" 0.0
observe "rear wheels turned BACKWARD, then stopped"

STEP=4
log; log "== 4. ask for 1.0 m/s: must be limited to $MAX_VX =="
ask "Next: speed limit."
drive 3 1.0 0.0
observe "a bit faster than step 2, not full speed (csv: odom vx about 0.10)"

STEP=5
log; log "== 5. forward $V m/s with wz +0.16 (left turn, radius 0.5 m) =="
ask "Next: steering. Watch the FRONT wheels."
drive 3 "$V" 0.16
observe "front wheels steered to the LEFT (car's left)"

STEP=6
log; log "== 6. wz 0.5 with vx 0 (turn on the spot): nothing may move =="
ask "Next: turn on the spot (should be refused)."
drive 3 0.0 0.5
observe "nothing moved and the front wheels did not steer"

STEP=7
log; log "== 7. stop the node (Ctrl+C equivalent) while driving =="
ask "Next: the node is stopped while the wheels turn."
record 8
pub 8 "$V" 0.0 &
PUB=$!
sleep "$(add "$PUB_START" 3)"   # wheels turning for about 3 s
log "  stopping the node at $(date +%T.%N | cut -c1-12)"
stop_base
wait "$PUB" 2>/dev/null
observe "wheels stopped as soon as the node stopped (stop frames)"

STEP=8
log; log "== 8. max_run_time 5 s after the first command; commands continue for 8 s =="
ask "Next: run-time limit."
start_base max_run_time:=5.0
drive 8 "$V" 0.0
reason
observe "wheels stopped after about 5 s although commands continued"
stop_base

STEP=9
log; log "== 9. low battery limit set to 13 V (above the battery): must refuse =="
ask "Next: battery limit."
start_base min_battery_voltage:=13.0
record 3
pub 3 "$V" 0.0 &
PUB=$!
sleep "$(add "$PUB_START" 1)"
reason
wait "$PUB" 2>/dev/null
sleep 3
observe "wheels did NOT move (gate reason above should be battery_low)"
stop_base

# Step 10 (kill -9: does the STM32 stop by itself?) is not run: the command
# timeout in the firmware (balance.c lines 219-221) is commented out, so the
# STM32 keeps the last command. Only our node's cmd_timeout and the Motor
# switch can stop the car.

log; log "done. Motor switch OFF now."
log "RESULT_DIR=$OUT"
