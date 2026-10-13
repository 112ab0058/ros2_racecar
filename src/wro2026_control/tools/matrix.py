#!/usr/bin/env python3
"""Run the offline scenario matrix in parallel and print one line per run.

    python3 tools/matrix.py                  # default matrix
    python3 tools/matrix.py --quick          # obstacle-free + SDF only
    python3 tools/matrix.py --plots out_dir  # also save trajectory plots
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import sim2d  # noqa: E402


def cases(quick: bool, stress: bool = False):
    scen = ["none", "sdf"] if quick else sorted(k for k in sim2d.SCENARIOS if not k.startswith("stress"))
    if stress:
        scen = sorted(k for k in sim2d.SCENARIOS if k.startswith("stress"))
    out = []
    for d in ("ccw", "cw"):
        for sc in scen:
            out.append(dict(direction=d, scenario=sc))
    return out


def run_case(case: dict, plot_dir: str | None = None) -> dict:
    d = case["direction"]
    start = case.get("start") or ((-0.05, -0.50, 0.0) if d == "ccw" else (-0.05, -0.50, math.pi))
    if case.get("start_shift"):
        sgn = 1.0 if d == "ccw" else -1.0          # move the start along the driving direction
        start = (start[0] + sgn * case["start_shift"], start[1], start[2])
    if case.get("start_yaw"):
        start = (start[0], start[1], start[2] + case["start_yaw"])
    sc = case["scenario"]
    layout = (sim2d.random_layout(int(sc[6:])) if sc.startswith("random")
              else sim2d.SCENARIOS[sc])
    world = sim2d.load_world(obstacles=layout)
    cfg = sim2d.SimConfig(start=start, yaw_drift=case.get("yaw_drift", 0.0),
                          odom_scale=case.get("odom_scale", 1.0),
                          ackermann_min_radius=case.get("ackermann", 0.0),
                          line_miss_prob=case.get("line_miss", 0.0),
                          lidar_noise=case.get("lidar_noise", 0.005),
                          seed=case.get("seed", 0))
    params = None
    if case.get("ackermann"):
        from wro2026_control.core import Params
        params = Params(min_turn_radius=case["ackermann"])
    res = sim2d.run(world, cfg, params=params, keep_trajectory=bool(plot_dir))
    if plot_dir:
        name = "_".join(f"{k}-{v}" for k, v in case.items() if k != "start")
        sim2d.plot(world, res, os.path.join(plot_dir, name + ".png"), title=name)
    reason = next((e[3] for e in reversed(res.events) if e[2] == "DONE"), "")
    # stop within 8 cm of the start along the lane (sideways it may be
    # offset when an obstacle sits next to the start)
    ok = res.done and res.corners == 12 and not res.collided and res.stop_along < 0.08
    return dict(case=case, ok=ok, corners=res.corners, time=res.time,
                collided=res.collided, min_clear=res.min_clearance,
                stop=res.stop_along, backoffs=res.backoffs, reason=reason)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--stress", action="store_true", help="stress layouts: only 'no collision' is required")
    ap.add_argument("--plots", default=None)
    ap.add_argument("--random", type=int, default=0, help="N random fair layouts per direction")
    ap.add_argument("--seed0", type=int, default=1)
    ap.add_argument("--robust", action="store_true",
                    help="sensor / odometry errors, Ackermann limit, other start poses")
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()
    if args.plots:
        os.makedirs(args.plots, exist_ok=True)
    cs = cases(args.quick, args.stress)
    if args.robust:
        cs = []
        for d in ("ccw", "cw"):
            for extra in (dict(yaw_drift=0.003), dict(odom_scale=0.95), dict(odom_scale=1.05),
                          dict(lidar_noise=0.012), dict(line_miss=0.6, seed=3),
                          dict(ackermann=0.25), dict(start_shift=0.35), dict(start_yaw=0.12)):
                cs.append(dict(direction=d, scenario="sdf", **extra))
    if args.random:
        cs = [dict(direction=d, scenario=f"random{seed}")
              for seed in range(args.seed0, args.seed0 + args.random) for d in ("ccw", "cw")]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        results = list(ex.map(run_case, cs, [args.plots] * len(cs)))
    passed = 0
    for r in results:
        c = r["case"]
        if args.stress:
            r["ok"] = not r["collided"]
        passed += r["ok"]
        extra = " ".join(f"{k}={v}" for k, v in c.items() if k not in ("direction", "scenario"))
        print(f"{'PASS' if r['ok'] else 'FAIL'} {c['direction']:3s} {c['scenario']:13s} {extra:18s} "
              f"corners={r['corners']:2d} t={r['time']:6.1f}s min_clear={r['min_clear']*1000:5.1f}mm "
              f"stop_err={r['stop']*100:4.1f}cm backoffs={r['backoffs']:2d} {r['reason']}")
    print(f"{passed}/{len(results)} passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
