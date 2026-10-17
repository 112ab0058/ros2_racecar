#!/usr/bin/env bash
# Orin: WHEELS-IN-THE-AIR test of the R550 command side. MAKES THE MOTORS TURN.
#
#   bash ~/racecar_ws/tools/orin_air_test.sh
#
# Run it yourself in an ssh terminal (it asks questions). Before running:
#   - car on a box, ALL FOUR WHEELS OFF THE GROUND, nothing touching them
#   - only the STM32 USB cable needed; sensors.launch.py must NOT be running
#   - your hand next to the red Motor switch (hardware stop)
# Limits: max_vx 0.1 m/s (real), each step 2-8 s, cmd_timeout 0.3 s.
# Stop at any time: Ctrl+C (stops the node, which sends stop frames), or
# Motor switch OFF.
# Results: ~/r550_airtest/<time>/report.txt + csv of /cmd_vel_out and /odom.
set -o pipefail

MAX_VX=0.1
OUT="$HOME/r550_airtest/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
REPORT="$OUT/report.txt"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

log() { echo "$*" | tee -a "$REPORT"; }

LAUNCH_PID=""
start_base() {
    setsid ros2 launch wro2026_base_r550 base.launch.py enable_commands:=true \
        max_vx:="$MAX_VX" "$@" > "$OUT/launch_$STEP.log" 2>&1 < /dev/null &
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
cleanup() { pkill -INT -f "ros2 topic pub" 2>/dev/null; stop_base; }
trap cleanup EXIT
trap 'echo; log "Ctrl+C: stopping"; exit 130' INT

pub() {  # seconds vx wz
    timeout -s INT "$1" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $2}, angular: {z: $3}}" > /dev/null 2>&1 < /dev/null
}
record() {  # seconds (background csv of what was sent and what the wheels did)
    timeout -s INT "$1" ros2 topic echo --csv /cmd_vel_out > "$OUT/step${STEP}_cmd_out.csv" \
        2>/dev/null < /dev/null &
    timeout -s INT "$1" ros2 topic echo --csv /odom > "$OUT/step${STEP}_odom.csv" \
        2>/dev/null < /dev/null &
}
ask() { read -r -p "$1  [Enter = go, q = quit] " a; [ "$a" = q ] && exit 0; }
observe() { read -r -p "  >> $1 (y / n / note): " a; log "  step $STEP: $1 -> $a"; }

if pgrep -f "wro2026_base_r550/(feedback_node|base_node)" > /dev/null; then
    echo "Another base/feedback node is running (it holds the serial port). Stop it first:"
    pgrep -af "feedback_node|base_node"
    exit 1
fi

log "R550 air test $(date)"
log "limits: max_vx $MAX_VX m/s, cmd_timeout 0.3 s; stop = Ctrl+C or Motor switch OFF"
read -r -p "Wheels off the ground, hand at the Motor switch? Type YES: " a
[ "$a" = YES ] || exit 0

STEP=1
log; log "== 1. Motor switch OFF: commands are sent but nothing may move =="
ask "Set the Motor switch to OFF."
start_base
record 4; pub 3 0.05 0.0; sleep 1
observe "wheels did NOT move (Motor OFF)"
log "  sent (cmd_vel_out csv): step1_cmd_out.csv, expect 0.05"

STEP=2
log; log "== 2. forward 0.05 m/s for 3 s, then the publisher stops =="
ask "Set the Motor switch to ON (wheels in the air)."
record 5; pub 3 0.05 0.0; sleep 2
observe "wheels turned FORWARD slowly, then stopped within about 0.3 s"

STEP=3
log; log "== 3. reverse -0.05 m/s for 2 s =="
ask "Next: reverse."
record 4; pub 2 -0.05 0.0; sleep 2
observe "wheels turned BACKWARD, then stopped"

STEP=4
log; log "== 4. ask for 1.0 m/s: must be limited to $MAX_VX =="
ask "Next: speed limit."
record 4; pub 2 1.0 0.0; sleep 2
observe "only slightly faster than step 2 (not full speed)"

STEP=5
log; log "== 5. forward 0.05 m/s with wz +0.1 (left turn, radius 0.5 m) =="
ask "Next: steering. Watch the FRONT wheels."
record 4; pub 2 0.05 0.1; sleep 2
observe "front wheels steered to the LEFT (car's left)"

STEP=6
log; log "== 6. wz 0.5 with vx 0 (turn on the spot): nothing may move =="
ask "Next: turn on the spot (should be refused)."
record 4; pub 2 0.0 0.5; sleep 2
observe "nothing moved and the front wheels did not steer"

STEP=7
log; log "== 7. Ctrl+C of the node while driving: node sends stop frames =="
ask "Next: the node is stopped while the wheels turn."
pub 6 0.05 0.0 &
PUB=$!
sleep 2
stop_base
wait "$PUB" 2>/dev/null
observe "wheels stopped as soon as the node stopped"

STEP=8
log; log "== 8. max_run_time 5 s: commands keep coming, the node stops after 5 s =="
ask "Next: run-time limit."
start_base max_run_time:=5.0
record 9; pub 8 0.05 0.0; sleep 1
observe "wheels stopped after about 5 s although commands continued"
stop_base

STEP=9
log; log "== 9. low battery limit set to 13 V (above the battery): must refuse =="
ask "Next: battery limit."
start_base min_battery_voltage:=13.0
record 4; pub 3 0.05 0.0; sleep 1
observe "wheels did NOT move"
stop_base

STEP=10
log; log "== 10. node KILLED (no stop frames): does the STM32 stop by itself? =="
log "  Unknown (no STM32 source yet). If the wheels keep turning, Motor switch OFF."
ask "Next: kill -9 while driving. Hand on the Motor switch."
start_base
pub 15 0.05 0.0 &
PUB=$!
sleep 3
pkill -9 -f "wro2026_base_r550/base_node"
log "  killed at $(date +%T)"
sleep 4
observe "wheels stopped BY THEMSELVES after the kill (n = kept turning)"
kill -INT "$PUB" 2>/dev/null
stop_base

log; log "done. Motor switch OFF now."
log "RESULT_DIR=$OUT"
