#!/usr/bin/env bash
# Orin: steering angles with the WHEELS IN THE AIR. MAKES THE MOTORS TURN (rear
# wheels slowly, 0.05 m/s) and moves the steering servo.
#
#   bash ~/racecar_ws/tools/orin_steer_air.sh
#
# Why (2026-11-25): on the floor the same full-lock command (radius 0.35 m) drove
# right 0.292 m but left 0.403 m. The firmware (balance.c Drive_Motor, usartx.c
# Vz_to_Akm_Angle) turns the radius into a right-front wheel angle, clamps it to
# -0.49..+0.32 rad, and maps it to the servo with a cubic fit: left full lock =
# PWM 1209 (291 us from 1500), right full lock = 2294 clipped to 2200 (700 us).
# This test photographs the front wheels at 13 commands so the wheel angle per
# PWM can be compared left vs right without tyre slip.
#
# Run it yourself in an ssh terminal (it waits for Enter; Claude Code cannot see
# the wheels). Before running:
#   - car on a box, ALL FOUR WHEELS OFF THE GROUND, nothing touching them
#   - a straight stick on each FRONT wheel: flat on the outer face, or taped on
#     top of the tyre along the tread (2026-11-26: the cables hide the right
#     wheel's outer face in turns; the front wheels are not driven, so they do not
#     spin). At most 8 cm past the tyre, nothing reaching back to the rear wheels
#     (they turn)
#   - only the STM32 USB cable needed; no other ROS program running
#   - your hand next to the red Motor switch (hardware stop)
# Limits: 0.05 m/s (max_vx 0.1), each step at most 60 s (then it stops by
# itself), whole test at most 900 s from the first command (base_node
# max_run_time; if the wheels stop answering, that limit was reached: run again), base_node
# cmd_timeout 0.3 s, firmware command timeout about 0.5 s.
# Stop at any time: q + Enter, Ctrl+C, or the Motor switch OFF.
# Results: ~/r550_steer/<time>/steps.csv (step, command, firmware PWM, start and
# end time, answers), odom.csv, report.txt. Photos go to Windows
# Downloads\r550_steer\<time>\ in shooting order.
set -o pipefail

MAX_VX=0.1
V=0.05
RUN_LIMIT=900
OUT="$HOME/r550_steer/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

if pgrep -f "wro2026_base_r550/(feedback_node|base_node|web_teleop_node)|wro2026_lidar_n10p|robot_state_publisher|foxglove_bridge" > /dev/null; then
    echo "有其他 ROS 程式在跑，請先停掉："
    pgrep -af "feedback_node|base_node|web_teleop_node|scan_node|robot_state_publisher|foxglove_bridge"
    exit 1
fi

LAUNCH_PID=""
stop_base() {
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.25; done
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
        LAUNCH_PID=""
    fi
}
trap stop_base EXIT

echo "R550 轉向架空測試 $(date)" | tee "$OUT/report.txt"
echo "限制：車速 $V m/s（上限 $MAX_VX）、每步最多 60 s、全程最多 $RUN_LIMIT s；停車：q + Enter、Ctrl+C 或 Motor 開關 OFF" \
    | tee -a "$OUT/report.txt"
read -r -p "四輪離地、前輪的棍子碰不到後輪、手在 Motor 開關旁？輸入 YES： " a
[ "$a" = YES ] || exit 0

setsid ros2 launch wro2026_base_r550 base.launch.py enable_commands:=true \
    max_vx:="$MAX_VX" max_run_time:="$RUN_LIMIT" > "$OUT/launch.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "（底盤程式啟動中，等 6 秒）"
sleep 6

cat > "$OUT/steps.py" << 'EOF'
"""Hold one steering command per step until Ray presses Enter (photo taken)."""
import csv
import math
import queue
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState

OUT, V = sys.argv[1], float(sys.argv[2])
STEP_MAX = 60.0         # s per step, then stop and wait
VX_SCALE = 1.035        # base_node divides vx and wz by it (r550_feedback.yaml)
L, W = 0.144, 0.162     # firmware Akm_axlespacing, Akm_wheelspacing
FULL = 0.35             # firmware MINI_AKM_MIN_TURN_RADIUS

# (label, radius in m: + left, - right, 0 straight)
STEPS = [('直走', 0.0),
         ('左轉 半徑 1.0 m', 1.0), ('左轉 半徑 0.7 m', 0.7), ('左轉 半徑 0.5 m', 0.5),
         ('左轉 半徑 0.42 m', 0.42), ('左滿舵（半徑 0.35 m）', FULL),
         ('直走（從左邊回正）', 0.0),
         ('右轉 半徑 1.0 m', -1.0), ('右轉 半徑 0.7 m', -0.7), ('右轉 半徑 0.5 m', -0.5),
         ('右轉 半徑 0.42 m', -0.42), ('右滿舵（半徑 0.35 m）', -FULL),
         ('直走（從右邊回正）', 0.0)]


def firmware(radius):
    """Return (vx frame, wz frame, right-front angle rad, servo PWM) the STM32 uses."""
    if radius == 0.0:
        return int(V / VX_SCALE * 1000), 0, 0.0, 1500
    vx_i = int(V / VX_SCALE * 1000)
    wz_i = int(V / radius / VX_SCALE * 1000)
    vx, wz = vx_i / 1000.0, wz_i / 1000.0
    if abs(vx / wz) <= FULL:
        wz = math.copysign(abs(vx) / FULL, wz)
    angle = math.atan(L / (vx / wz + 0.5 * W))
    angle = max(-0.49, min(0.32, angle))
    servo = -0.628 * angle ** 3 + 1.269 * angle ** 2 - 1.772 * angle + 1.573
    pwm = max(800, min(2200, int(1500 + (servo - 1.572) * 636.56)))
    return vx_i, wz_i, angle, pwm


class Steer(Node):
    def __init__(self):
        super().__init__('steer_air_test')
        self.vx = self.wz = 0.0
        self.voltage = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.odom = open(OUT + '/odom.csv', 'w', newline='')
        self.odom_csv = csv.writer(self.odom)
        self.odom_csv.writerow(['t', 'vx', 'wz'])
        self.create_timer(0.05, self._send)

    def _send(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.vx, self.wz
        self.pub.publish(msg)

    def _on_odom(self, msg):
        self.odom_csv.writerow(['%.3f' % time.time(), '%.4f' % msg.twist.twist.linear.x,
                                '%.4f' % msg.twist.twist.angular.z])

    def _on_battery(self, msg):
        self.voltage = msg.voltage

    def command(self, radius):
        self.vx = V
        self.wz = 0.0 if radius == 0.0 else V / radius

    def stop(self):
        self.vx = self.wz = 0.0


lines = queue.Queue()


def read_stdin():
    for line in sys.stdin:
        lines.put(line.strip())
    lines.put('q')


def drain():
    """Drop extra Enters typed ahead; return True if q (or end of input) was among them."""
    quit_seen = False
    while not lines.empty():
        quit_seen = quit_seen or lines.get_nowait() == 'q'
    return quit_seen


def ask(prompt, timeout=None):
    print(prompt, end=' ', flush=True)
    try:
        return lines.get(timeout=timeout)
    except queue.Empty:
        return None


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C = KeyboardInterrupt
    node = Steer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    threading.Thread(target=read_stdin, daemon=True).start()
    report = open(OUT + '/report.txt', 'a')
    rows = open(OUT + '/steps.csv', 'w', newline='')
    out = csv.writer(rows)
    out.writerow(['step', 'label', 'radius', 'vx_frame', 'wz_frame', 'angle_right_front_deg',
                  'pwm', 'start', 'end', 'buzz', 'stop_reached', 'note'])
    time.sleep(2.0)
    v = node.voltage
    print('電池 %s V' % ('%.2f' % v if v is not None else '讀不到'))
    report.write('battery %s V\n' % v)
    if v is not None and v < 10.8:
        print('電池低於 10.8 V，請先充電。')
        return
    try:
        i = 0
        while i < len(STEPS):
            label, radius = STEPS[i]
            vx_i, wz_i, angle, pwm = firmware(radius)
            if drain():
                break
            a = ask('\n第 %d/%d 步：%s。按 Enter 開始（q = 結束）' % (i + 1, len(STEPS), label))
            if a == 'q':
                break
            node.command(radius)
            start = time.time()
            print('  前輪轉向中。手機在車子正上方，四個輪子和兩根棍子都要入鏡，拍一張。')
            a = ask('  拍完按 Enter（%d s 內沒按會自動停）' % STEP_MAX, STEP_MAX)
            buzz = stop = note = ''
            if a is not None and a != 'q' and abs(radius) == FULL:
                left = STEP_MAX - (time.time() - start)
                buzz = ask('  舵機有沒有一直發出嗡嗡聲？(y / n / 備註)', max(left, 1.0)) or ''
                left = STEP_MAX - (time.time() - start)
                stop = ask('  轉向連桿或輪子有沒有碰到底、轉不過去？(y / n / 備註)',
                           max(left, 1.0)) or ''
            node.stop()
            end = time.time()
            if a is None:
                print('\n  超過 %d s，已停。這一步重做。' % STEP_MAX)
                continue
            out.writerow([i + 1, label, radius, vx_i, wz_i, '%.2f' % math.degrees(angle), pwm,
                          '%.3f' % start, '%.3f' % end, buzz, stop, note])
            rows.flush()
            report.write('step %d %s: pwm %d, buzz %s, stop %s\n' % (i + 1, label, pwm, buzz, stop))
            if a == 'q' or 'q' in (buzz, stop):
                break
            i += 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\nCtrl+C：停車')
    finally:
        node.stop()
        time.sleep(0.5)
        rows.close()
        report.close()
        executor.shutdown()
        spin.join(2.0)
        node.odom.close()
        node.destroy_node()
        rclpy.try_shutdown()
    print('\n結束。請把 Motor 開關切到 OFF。')


main()
EOF

python3 "$OUT/steps.py" "$OUT" "$V"
stop_base
echo "RESULT_DIR=$OUT" | tee -a "$OUT/report.txt"
