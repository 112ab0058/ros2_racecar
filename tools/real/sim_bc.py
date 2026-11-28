"""
Windows / WSL: can the 5-label imitation network drive the 2-D field? (no ROS, no car)

Teacher = the corner rule (corner.py, timing predict) on the measured car (0.403 m left /
0.292 m right at full lock), the measured blind spot (partial), camera max 0.5 m, scan
memory: exactly the field setup of orin_auto.sh. While the teacher drives a straight,
now and then a random left / right nudge of 0.3-0.6 s is executed instead and the
teacher's answer is recorded (so the data also shows how to get back to the centre).
Student = MLP 44-32-16-5 on scan_features of the filled scan, Driver like bc_node.
Test = new start poses, both directions, 3 laps, no nudges.

    python tools/real/sim_bc.py [out.json]       (about 70 min on Ray's laptop)

2026-11-25 result (both directions demonstrated, 72 teacher runs with nudges, 43,750 samples):
teacher 24/24; network mirror none ccw 8/12 cw 12/12, turns 3/12 11/12, all 0/12 11/12;
ccw data only: mirror turns ccw 5/12 cw 0/12, all 0/12 0/12. Misses are mostly at the first
corner from the second half of the straight (too-close stops): the rule remembers the
island wall behind the car, the network sees one scan. Validation accuracy (89-92 %) did
not tell which network can drive.
"""
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src')
sys.path[:0] = [os.path.join(ROOT, 'wro2026_imitation'), os.path.join(ROOT, 'wro2026_lidar_n10p')]

from wro2026_imitation.corner import CornerPolicy, FOLLOW  # noqa: E402
from wro2026_imitation.features import LABELS, N_BINS, scan_features  # noqa: E402
from wro2026_imitation.field_sim import camera as camera_view, Car, Field, lidar  # noqa: E402
from wro2026_imitation.model import balanced_accuracy, confusion, MLP, train  # noqa: E402
from wro2026_imitation.policy import Driver, model_policy, STOP  # noqa: E402
from wro2026_imitation.train import class_weights, mirrored  # noqa: E402
from wro2026_lidar_n10p.scan_memory import ScanMemory  # noqa: E402

INC = 2.0 * math.pi / 480
ISLAND = 0.80


def start_pose(direction, start, lat, head):
    y = -(0.8 + ISLAND / 2.0) / 2.0 + lat
    if direction == 'ccw':
        return -ISLAND / 2.0 + start, y, math.radians(head)
    return ISLAND / 2.0 - start, y, math.pi - math.radians(head)


def run(direction, policy, laps, start, lat, head, seed, record=False, nudge=0.0):
    """Drive; return (ok, why, laps done, min clearance, X, Y)."""
    field = Field(1.6, ISLAND)
    car = Car(*start_pose(direction, start, lat, head))
    driver = Driver(policy, 0.1, 0.5, stop_margin=0.02, full_turn_radius=0.35)
    mem = ScanMemory()
    rng = np.random.default_rng(seed)
    dt, t, vx, wz, turned, last, clear = 0.02, 0.0, 0.0, 0.0, 0.0, car.yaw, 9.0
    xs, ys = [], []
    nudge_left, nudge_wz = 0, 0.0
    while t < laps * 50.0:
        if int(round(t / dt)) % 5 == 0:
            driver.on_odom(t, car.x, car.y, car.yaw, vx, vx * car.curv)
            mem.on_odom(car.x, car.y, car.yaw)
            mem.add_camera(t, camera_view(field, car, rng=rng, max_range=0.5), 0.0, INC)
            r = lidar(field, car, rng=rng, partial=True)
            r, remembered = mem.on_scan(t, r, 0.0, INC)
            driver.on_scan(t, r, 0.0, INC, remembered)
            vx, wz, decision, reason = driver.command(t)
            if decision == STOP:
                return False, reason, abs(turned) / (2 * math.pi), clear, xs, ys
            if record:
                xs.append(scan_features(r)[0])
                ys.append(decision)
                if nudge_left == 0 and getattr(policy, 'state', None) == FOLLOW \
                        and rng.random() < nudge:
                    nudge_left = int(rng.integers(3, 7))
                    nudge_wz = float(rng.choice([-0.2, 0.2]))
                if nudge_left > 0:
                    nudge_left -= 1
                    vx, wz = 0.1, nudge_wz
        car.step(vx, wz, dt)
        turned += car.yaw - last
        last = car.yaw
        clear = min(clear, field.clearance(car.x, car.y, car.yaw))
        if clear < 0:
            return False, 'crash', abs(turned) / (2 * math.pi), clear, xs, ys
        if abs(turned) >= laps * 2 * math.pi:
            return True, 'done', abs(turned) / (2 * math.pi), clear, xs, ys
        t += dt
    return False, 'too slow', abs(turned) / (2 * math.pi), clear, xs, ys


TRAIN_POSES = [(d, s, la, h) for d in ('ccw', 'cw') for s in (0.05, 0.2, 0.35, 0.5)
               for la in (-0.04, 0.0, 0.04) for h in (-6.0, 0.0, 6.0)]
TEST_POSES = [(d, s, la, h) for d in ('ccw', 'cw') for s in (0.1, 0.3, 0.45)
              for la in (-0.03, 0.03) for h in (-4.0, 4.0)]


def collect(nudge, directions=('ccw', 'cw')):
    runs = []
    for k, (d, s, la, h) in enumerate(TRAIN_POSES):
        if d not in directions:
            continue
        ok, why, laps, clear, xs, ys = run(d, CornerPolicy(d, timing='predict'), 2, s, la, h,
                                           seed=k, record=True, nudge=nudge)
        if not ok:          # as bc_extract: drop the last 1.5 s before the failure
            xs, ys = xs[:-15], ys[:-15]
        runs.append((k, d, ok, why, np.array(xs).reshape(-1, N_BINS), np.array(ys, int)))
    return runs


def fit(runs, mirror, seed=0):
    tr = [r for i, r in enumerate(runs) if i % 5 != 4]
    va = [r for i, r in enumerate(runs) if i % 5 == 4]
    x_tr = np.concatenate([r[4] for r in tr])
    y_tr = np.concatenate([r[5] for r in tr])
    x_va = np.concatenate([r[4] for r in va])
    y_va = np.concatenate([r[5] for r in va])
    x_m, y_m = mirrored(x_tr, y_tr, mirror)
    x_tr, y_tr = np.concatenate([x_tr, x_m]), np.concatenate([y_tr, y_m])
    model = MLP(N_BINS, hidden=(32, 16), n_out=len(LABELS), seed=seed)
    train(model, x_tr, y_tr, x_va, y_va, epochs=300, seed=seed,
          sample_weight=class_weights(y_tr), val_weight=class_weights(y_va))
    table = confusion(y_va, model.predict(x_va), n=len(LABELS))
    return model, float(table.trace() / table.sum()), float(balanced_accuracy(table)), table


def evaluate(make_policy, poses=TEST_POSES, laps=3):
    out = []
    for k, (d, s, la, h) in enumerate(poses):
        ok, why, done, clear, _, _ = run(d, make_policy(d), laps, s, la, h, seed=100 + k)
        out.append({'dir': d, 'start': s, 'lat': la, 'head': h, 'ok': ok, 'why': why,
                    'laps': round(done, 2), 'clear': round(clear, 3)})
    return out


def summary(name, res):
    by = {d: [r for r in res if r['dir'] == d] for d in ('ccw', 'cw')}
    text = '%-34s' % name + '  '.join(
        '%s %2d/%2d (laps %.1f avg)' % (d, sum(r['ok'] for r in rs), len(rs),
                                        np.mean([r['laps'] for r in rs]))
        for d, rs in by.items())
    print(text, flush=True)
    return text


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'sim_bc.json'
    t0 = time.time()
    report = {}
    teacher = evaluate(lambda d: CornerPolicy(d, timing='predict'))
    report['teacher'] = {'text': summary('teacher (corner rule, predict)', teacher),
                         'runs': teacher}
    runs = collect(nudge=0.03)
    n = sum(len(r[5]) for r in runs)
    counts = np.bincount(np.concatenate([r[5] for r in runs]), minlength=len(LABELS))
    print('demonstrations: %d runs, %d ok, %d samples: %s (%.0f s)' % (
        len(runs), sum(r[2] for r in runs), n,
        ', '.join('%s %d' % (a, c) for a, c in zip(LABELS, counts)), time.time() - t0),
        flush=True)
    report['data'] = {'runs': len(runs), 'ok': int(sum(r[2] for r in runs)), 'samples': n,
                      'counts': counts.tolist()}
    for mirror in ('turns', 'all', 'none'):
        model, acc, bal, table = fit(runs, mirror)
        res = evaluate(lambda d, m=model: model_policy(m))
        name = 'network, mirror %s (val %.0f%%/%.0f%%)' % (mirror, 100 * acc, 100 * bal)
        report['mirror_' + mirror] = {'text': summary(name, res), 'val_acc': acc,
                                      'val_bal': bal, 'table': table.tolist(), 'runs': res}
        model.save(out.replace('.json', '_%s.npz' % mirror), labels=list(LABELS))
    # only counter-clockwise demonstrations: does mirroring give the clockwise laps?
    ccw = [r for r in runs if r[1] == 'ccw']
    for mirror in ('turns', 'all'):
        model, acc, bal, _ = fit(ccw, mirror)
        res = evaluate(lambda d, m=model: model_policy(m))
        name = 'ccw data only, mirror %s' % mirror
        report['ccw_only_' + mirror] = {'text': summary(name, res), 'runs': res}
    report['seconds'] = round(time.time() - t0)
    with open(out, 'w') as f:
        json.dump(report, f, indent=1)
    print('done in %.0f s -> %s' % (time.time() - t0, out))


if __name__ == '__main__':
    main()
