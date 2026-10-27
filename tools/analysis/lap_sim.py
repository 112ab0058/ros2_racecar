#!/usr/bin/env python3
"""
Offline lap simulator: lap_node's controller + CommandGate + a model of the gz Ackermann
plugin. No ROS, no Gazebo: one 2-lap run takes about a second, for screening controllers
and paths before the (slow) Gazebo runs. Every conclusion must be checked in Gazebo.

    python tools/analysis/lap_sim.py --half-side 0.58 --out C:/tmp/sim058
    python tools/analysis/lap_sim.py --controller lag_ff --set steer_tau=1.0 --set k_y=10

Model, one physics step = 5 ms (Gazebo's step):
  lap tick 20 Hz     LapController (the same code as lap_node) on the pose below
  CommandGate 20 Hz  wro2026_base_r550.command (the same code as sim_base_node), started
                     --gate-phase s after the lap tick; its output reaches the plugin
                     --cmd-delay s later
  gz plugin          turning radius vx / wz clamped to wheel_base / sin(steering_limit);
                     steering angle target atan(wheel_base / r); the actual angle follows
                     d(angle)/dt = steer_gain x (target - angle)   (gz steer_p_gain, default 1)
  car                curvature = (1 - rear_weight) tan(angle) / wheel_base
                     + rear_weight / r (the rear wheel speeds follow the target at once),
                     times curv_scale; rear axle speed = gate vx; no other slip
  pose               the true pose as published at 20 Hz (stale up to 50 ms); with
                     --gyro-scale-error / --odom-scale-error the controller instead gets dead
                     reckoning whose yaw rate / distance is off by that fraction (the EKF's
                     errors; Gazebo 2026-10-27: gyro -0.005, wheel odometry +0.019)
Defaults = Gazebo with the plugin's default steer_p_gain 1, fitted to the step test of the
2026-10-27 campaign (stage A2): steer_gain 1.25, rear_weight 0.2, curv_scale 0.92,
cmd_delay 0.05 (RMS curvature error 0.12 1/m). steer_p_gain 5: --steer-gain 5
--curv-scale 0.97; 10: --steer-gain 10 --rear-weight 0.1 --curv-scale 0.97.
Writes DIR/lap.csv and DIR/path.csv in lap_node's format (lap_metrics.py reads them).
"""
import argparse
import math
import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WHEEL_BASE = 0.144
STEERING_LIMIT = 0.424          # model.sdf; gz minimum radius = wheel_base / sin(limit)
PHYSICS_DT = 0.005


def import_lap_node():
    """Import lap_node and CommandGate; stub the ROS modules when ROS is not installed."""
    for p in (os.path.join(REPO, 'src', 'wro2026_sim_r550'),
              os.path.join(REPO, 'src', 'wro2026_base_r550')):
        if p not in sys.path:
            sys.path.insert(0, p)
    try:
        import rclpy  # noqa: F401
    except ImportError:
        for name in ('rclpy', 'rclpy.node', 'rclpy.executors', 'geometry_msgs',
                     'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg'):
            sys.modules.setdefault(name, types.ModuleType(name))
        sys.modules['rclpy.node'].Node = object
        sys.modules['rclpy.executors'].ExternalShutdownException = Exception
        sys.modules['geometry_msgs.msg'].Twist = object
        sys.modules['nav_msgs.msg'].Odometry = object
    from wro2026_sim_r550 import lap_node
    from wro2026_base_r550.command import CommandGate
    return lap_node, CommandGate


def simulate(L, CommandGate, path, controller='feedforward', settings=None, laps=2,
             start=None, steer_gain=1.25, rear_weight=0.2, curv_scale=0.92, gate_phase=0.02,
             cmd_delay=0.05, gyro_scale_error=0.0, odom_scale_error=0.0, obstacles=(),
             max_time=None, pose_error=(0.0, 0.0, 0.0)):
    """Run the model; return the rows of lap.csv (lap_node's columns)."""
    ctl = L.LapController(path, controller, **(settings or {}))
    gate = CommandGate(enabled=True)
    n = len(path)
    if start is None:
        (x0, y0), (x1, y1) = path[0], path[1]
        start = (x0, y0, math.atan2(y1 - y0, x1 - x0))
    x, y, yaw = start
    # dead reckoning ("EKF"): starts at the pose the node is told (pose_error = placement
    # error the node does not know about), then integrates with the gyro / odometry errors
    ex, ey, eyaw = (a + b for a, b in zip(start, pose_error))
    r_min = WHEEL_BASE / math.sin(STEERING_LIMIT)
    angle = 0.0
    gz_vx = gz_wz = 0.0
    pending = []                      # gate outputs on their way to the plugin
    seen = None                       # last published true pose (20 Hz)
    rows, t_end, i_truth = [], None, None
    length = sum(math.dist(path[i], path[(i + 1) % n]) for i in range(n))
    max_time = max_time or laps * length / 0.1 + 20.0
    steps_per_tick = int(round(0.05 / PHYSICS_DT))
    gate_offset = int(round(gate_phase / PHYSICS_DT)) % steps_per_tick
    v_true = kappa = 0.0
    started = False
    for step in range(int(max_time / PHYSICS_DT)):
        t = step * PHYSICS_DT
        phase = step % steps_per_tick
        if phase == 0:                                    # truth published + lap tick
            seen = (x, y, yaw)
            dead_reckoning = (gyro_scale_error != 0.0 or odom_scale_error != 0.0
                              or any(pose_error))
            pose = (ex, ey, eyaw) if dead_reckoning else seen
            if not started:
                ctl.start(pose)
                i_truth = ctl.i_near
                started = True
            if t_end is None:
                v, wz = ctl.step(pose)
                if ctl.progress >= laps * n:
                    t_end = t
                    v = wz = 0.0
                gate.set_cmd(t, v, wz)
                i_truth = L.nearest(path, x, y, i_truth)
                lat = L.lateral_error(path, i_truth, x, y)
                tp = (x, y, yaw)
                rows.append((t, x, y, yaw, ex, ey, eyaw, math.dist(path[i_truth], (x, y)),
                             L.clearance(tp), v, wz, v_true, float(ctl.corner[i_truth]), lat,
                             L.outer_clearance(tp), L.island_clearance(tp), t, t, v_true * kappa,
                             min(L.obstacle_clearance(tp, obstacles), 9.9999), 0.0))
            else:
                gate.set_cmd(t, 0.0, 0.0)
                if t - t_end > 1.0:
                    break
        if phase == gate_offset:
            gate.set_voltage(t, 12.0)
            out_vx, out_wz, _ = gate.update(t, t)
            pending.append((t + cmd_delay, out_vx, out_wz))
        while pending and pending[0][0] <= t + 1e-9:
            _, gz_vx, gz_wz = pending.pop(0)
        # gz AckermannSteering
        if abs(gz_wz) < 0.001:
            r = 1e9
        else:
            r = gz_vx / gz_wz if abs(gz_vx) > 0 else math.copysign(r_min, gz_wz)
            if 0.0 <= r < r_min:
                r = r_min
            elif -r_min < r <= 0.0:
                r = -r_min
        target = math.atan(WHEEL_BASE / r)
        angle += steer_gain * (target - angle) * PHYSICS_DT
        kappa = curv_scale * ((1.0 - rear_weight) * math.tan(angle) / WHEEL_BASE
                              + rear_weight / r)
        v_true = gz_vx
        yaw_mid = yaw + 0.5 * v_true * kappa * PHYSICS_DT
        x += v_true * math.cos(yaw_mid) * PHYSICS_DT
        y += v_true * math.sin(yaw_mid) * PHYSICS_DT
        yaw = L.wrap(yaw + v_true * kappa * PHYSICS_DT)
        e_mid = eyaw + 0.5 * v_true * kappa * (1.0 + gyro_scale_error) * PHYSICS_DT
        ex += v_true * (1.0 + odom_scale_error) * math.cos(e_mid) * PHYSICS_DT
        ey += v_true * (1.0 + odom_scale_error) * math.sin(e_mid) * PHYSICS_DT
        eyaw = L.wrap(eyaw + v_true * kappa * (1.0 + gyro_scale_error) * PHYSICS_DT)
    return rows


COLUMNS = ('t,truth_x,truth_y,truth_yaw,ekf_x,ekf_y,ekf_yaw,xte,clearance,vx,wz,truth_v,'
           'corner,lat,outer,island,truth_stamp,ekf_stamp,truth_wz,obstacle,tick_ms')


def write(out, path, curv, corner, rows):
    """Write lap.csv and path.csv like lap_node."""
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'path.csv'), 'w') as f:
        f.write('x,y,curvature,corner\n')
        for (x, y), k, c in zip(path, curv, corner):
            f.write('%.5f,%.5f,%.4f,%d\n' % (x, y, k, c))
    with open(os.path.join(out, 'lap.csv'), 'w') as f:
        f.write(COLUMNS + '\n')
        for r in rows:
            f.write(','.join('%.4f' % v for v in r) + '\n')


def summary(rows, t_line_xte=0.03):
    """Return a dict of the main numbers of one run."""
    t_line = next((r[0] for r in rows if r[7] < t_line_xte), None)
    on = [r for r in rows if t_line is not None and r[0] >= t_line]
    lats = sorted(abs(r[13]) for r in on) or [0.0]
    signed = [r[13] for r in on] or [0.0]
    return {'outer': min(r[14] for r in rows), 'island': min(r[15] for r in rows),
            'lat_p95': lats[int(0.95 * (len(lats) - 1))], 'lat_max': lats[-1],
            'lat_in': max(signed), 'lat_out': min(signed),
            'obstacle': min(r[19] for r in rows), 'time': rows[-1][0]}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--half-side', type=float, default=0.55)
    ap.add_argument('--radius', type=float, default=0.42)
    ap.add_argument('--transition', type=float, default=0.0)
    ap.add_argument('--inset', type=float, default=0.0)
    ap.add_argument('--offsets', type=float, nargs=4, default=None)
    ap.add_argument('--controller', default='feedforward')
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                    help='LapController setting, e.g. k_y=10 (repeatable)')
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--steer-gain', type=float, default=1.25)
    ap.add_argument('--rear-weight', type=float, default=0.2)
    ap.add_argument('--curv-scale', type=float, default=0.92)
    ap.add_argument('--gate-phase', type=float, default=0.02)
    ap.add_argument('--cmd-delay', type=float, default=0.05)
    ap.add_argument('--gyro-scale-error', type=float, default=0.0)
    ap.add_argument('--odom-scale-error', type=float, default=0.0)
    ap.add_argument('--start-offset', type=float, default=0.0,
                    help='lateral start offset (m, + = toward the island), mid south side')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    L, CommandGate = import_lap_node()
    settings = {k: float(v) for k, v in (s.split('=', 1) for s in a.set)}
    path = L.make_path(a.half_side, a.radius, transition=a.transition, offsets=a.offsets,
                       inset=a.inset)
    south = (a.offsets[0] if a.offsets else a.half_side)
    rows = simulate(L, CommandGate, path, a.controller, settings, a.laps,
                    start=(0.0, -south + a.start_offset, 0.0), steer_gain=a.steer_gain,
                    rear_weight=a.rear_weight, curv_scale=a.curv_scale,
                    gate_phase=a.gate_phase, cmd_delay=a.cmd_delay,
                    gyro_scale_error=a.gyro_scale_error, odom_scale_error=a.odom_scale_error)
    s = summary(rows)
    print('outer wall %.3f m, island %.3f m, |lat| p95 %.3f max %.3f m (in %+.3f / out %+.3f), '
          'time %.1f s' % (s['outer'], s['island'], s['lat_p95'], s['lat_max'], s['lat_in'],
                           s['lat_out'], s['time']))
    if a.out:
        ctl = L.LapController(path, a.controller, **settings)
        write(a.out, path, ctl.curv, ctl.corner, rows)


if __name__ == '__main__':
    main()
