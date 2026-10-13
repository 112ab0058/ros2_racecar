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


def cases(quick: bool):
    scen = ["none", "sdf"] if quick else sorted(sim2d.SCENARIOS)
    out = []
    for d in ("ccw", "cw"):
        for sc in scen:
            out.append(dict(direction=d, scenario=sc))
    return out


def run_case(case: dict, plot_dir: str | None = None) -> dict:
    d = case["direction"]
    start = case.get("start") or ((-0.05, -0.50, 0.0) if d == "ccw" else (-0.05, -0.50, math.pi))
    world = sim2d.load_world(obstacles=sim2d.SCENARIOS[case["scenario"]])
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
    ok = res.done and res.corners == 12 and not res.collided and res.stop_offset < 0.1
    return dict(case=case, ok=ok, corners=res.corners, time=res.time,
                collided=res.collided, min_clear=res.min_clearance,
                stop=res.stop_offset, backoffs=res.backoffs, reason=reason)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--plots", default=None)
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()
    if args.plots:
        os.makedirs(args.plots, exist_ok=True)
    cs = cases(args.quick)
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        results = list(ex.map(run_case, cs, [args.plots] * len(cs)))
    passed = 0
    for r in results:
        c = r["case"]
        passed += r["ok"]
        print(f"{'PASS' if r['ok'] else 'FAIL'} {c['direction']:3s} {c['scenario']:13s} "
              f"corners={r['corners']:2d} t={r['time']:6.1f}s min_clear={r['min_clear']*1000:5.1f}mm "
              f"stop_offset={r['stop']*100:5.1f}cm backoffs={r['backoffs']:2d} {r['reason']}")
    print(f"{passed}/{len(results)} passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
