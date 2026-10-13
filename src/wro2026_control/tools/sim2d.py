#!/usr/bin/env python3
"""Offline 2D simulator for the pose-free race controller.

Loads walls, obstacles and turn lines from the same Gazebo SDF the race
launch uses, then drives ``RaceController`` with simulated LiDAR, line
triggers and (optionally noisy) odometry.  It is a kinematic simulator: it
checks the controller's decisions and geometry, not tyre physics.

Usage:
    python3 tools/sim2d.py --direction ccw
    python3 tools/sim2d.py --direction cw --scenario three --plot run.png
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from shapely.geometry import LineString, Polygon, box
from shapely import affinity

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from wro2026_control.core import Params, RaceController, Scan, State  # noqa: E402

DEFAULT_SDF = os.path.join(
    HERE, "..", "..", "wro2026_sim", "worlds", "wro2026_field.sdf")


# ----------------------------------------------------------------- world
@dataclass
class World:
    walls: list            # shapely polygons (static walls)
    obstacles: list        # shapely polygons
    lines: list            # (colour, LineString)

    @property
    def solids(self):
        return self.walls + self.obstacles


def _pose(text: str):
    v = [float(x) for x in text.split()]
    return v[0], v[1], v[5] if len(v) > 5 else 0.0


def load_world(sdf_path: str = DEFAULT_SDF, obstacles: Optional[list] = None) -> World:
    """Read static boxes and lines from the SDF.

    ``obstacles`` replaces the yellow obstacle positions: a list of (x, y)
    centres (0.06 m boxes).  ``None`` keeps the SDF placement.
    """
    root = ET.parse(sdf_path).getroot()
    walls, obs, lines = [], [], []
    for model in root.iter("model"):
        name = model.get("name", "")
        pose_el = model.find("pose")
        if pose_el is None:
            continue
        x, y, yaw = _pose(pose_el.text)
        if "_line_" in name:
            size = model.find(".//visual/geometry/box/size")
            length = float(size.text.split()[0])
            dx, dy = math.cos(yaw) * length / 2, math.sin(yaw) * length / 2
            colour = "orange" if name.startswith("orange") else "blue"
            lines.append((colour, LineString([(x - dx, y - dy), (x + dx, y + dy)])))
            continue
        size = model.find(".//collision/geometry/box/size")
        if size is None:
            continue
        sx, sy, sz = (float(v) for v in size.text.split())
        if sz < 0.05:          # floors / tape
            continue
        poly = affinity.rotate(box(x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2),
                               yaw, origin=(x, y), use_radians=True)
        (obs if name.startswith("obstacle") else walls).append(poly)
    if obstacles is not None:
        obs = [box(ox - 0.03, oy - 0.03, ox + 0.03, oy + 0.03) for ox, oy in obstacles]
    return World(walls, obs, lines)


SCENARIOS = {
    # SDF placement: one obstacle in the north lane, two parked on the island
    "sdf": None,
    "none": [],
    "north_outer": [(0.05, 0.60)],
    "three": [(0.00, 0.42), (-0.52, -0.05), (0.30, -0.58)],
    "four_lanes": [(0.05, 0.56), (-0.45, 0.10), (-0.35, -0.58), (0.55, -0.05)],
}


# ----------------------------------------------------------------- robot
@dataclass
class SimConfig:
    start: tuple = (-0.05, -0.50, 0.0)
    dt: float = 0.01
    control_hz: float = 20.0
    scan_hz: float = 8.0
    camera_hz: float = 10.0
    max_time: float = 240.0
    lidar_noise: float = 0.005
    yaw_drift: float = 0.0          # rad/s bias in reported yaw
    odom_scale: float = 1.0         # reported / true path length
    ackermann_min_radius: float = 0.0
    line_miss_prob: float = 0.0     # chance a whole line crossing is not reported
    seed: int = 0
    # footprint relative to base_link (TurtleBot3 burger collision box + wheels)
    foot_x: tuple = (-0.102, 0.038)
    foot_half_w: float = 0.089
    lidar_x: float = -0.032
    # camera trigger window on the floor, base frame
    cam_x: tuple = (0.10, 0.26)
    cam_half_w: float = 0.10


@dataclass
class Result:
    done: bool
    state: str
    corners: int
    time: float
    collided: bool
    min_clearance: float
    final_pose: tuple
    stop_offset: float              # distance between stop and start pose (m)
    backoffs: int
    trajectory: list = field(repr=False, default_factory=list)
    events: list = field(repr=False, default_factory=list)


def footprint(cfg: SimConfig, x, y, yaw) -> Polygon:
    p = box(cfg.foot_x[0], -cfg.foot_half_w, cfg.foot_x[1], cfg.foot_half_w)
    p = affinity.rotate(p, yaw, origin=(0, 0), use_radians=True)
    return affinity.translate(p, x, y)


def lidar_scan(world: World, cfg: SimConfig, x, y, yaw, rng) -> Scan:
    n = 360
    angles = np.arange(n) * (2 * math.pi / n)
    ox, oy = x + cfg.lidar_x * math.cos(yaw), y + cfg.lidar_x * math.sin(yaw)
    ranges = np.full(n, np.inf)
    dirs = np.stack([np.cos(angles + yaw), np.sin(angles + yaw)], axis=1)
    for poly in world.solids:
        coords = np.asarray(poly.exterior.coords)
        a, b = coords[:-1], coords[1:]
        e = b - a
        for (ax, ay), (ex, ey) in zip(a, e):
            # ray: o + t d ; segment: a + u e
            den = dirs[:, 0] * ey - dirs[:, 1] * ex
            with np.errstate(divide="ignore", invalid="ignore"):
                t = ((ax - ox) * ey - (ay - oy) * ex) / den
                u = ((ax - ox) * dirs[:, 1] - (ay - oy) * dirs[:, 0]) / den
            hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
            ranges = np.where(hit & (t < ranges), t, ranges)
    ranges = ranges + rng.normal(0.0, cfg.lidar_noise, n)
    ranges[ranges > 3.5] = np.inf
    return Scan(angle_min=0.0, angle_increment=2 * math.pi / n,
                ranges=ranges.tolist(), range_min=0.12, range_max=3.5)


def camera_lines(world: World, cfg: SimConfig, x, y, yaw) -> tuple[bool, bool]:
    win = box(cfg.cam_x[0], -cfg.cam_half_w, cfg.cam_x[1], cfg.cam_half_w)
    win = affinity.translate(affinity.rotate(win, yaw, origin=(0, 0), use_radians=True), x, y)
    orange = blue = False
    for colour, line in world.lines:
        if win.intersects(line.buffer(0.01)):
            if colour == "orange":
                orange = True
            else:
                blue = True
    return orange, blue


def run(world: World, cfg: SimConfig, params: Optional[Params] = None,
        keep_trajectory: bool = False) -> Result:
    rng = np.random.default_rng(cfg.seed)
    prng = random.Random(cfg.seed)
    params = params or Params()
    if cfg.ackermann_min_radius > 0:
        params.min_turn_radius = max(params.min_turn_radius, cfg.ackermann_min_radius)
    ctl = RaceController(params)
    ctl.start()
    x, y, yaw = cfg.start
    t, s_true = 0.0, 0.0
    cmd_v = cmd_w = 0.0
    scan = None
    orange = blue = False
    missed = {"orange": False, "blue": False}
    prev_seen = {"orange": False, "blue": False}
    next_ctrl = next_scan = next_cam = 0.0
    min_clear = math.inf
    collided = False
    traj, events = [], []
    last_state = ctl.state
    solids = world.solids
    while t < cfg.max_time:
        if t >= next_scan:
            scan = lidar_scan(world, cfg, x, y, yaw, rng)
            next_scan += 1.0 / cfg.scan_hz
        if t >= next_cam:
            o, b = camera_lines(world, cfg, x, y, yaw)
            for colour, seen in (("orange", o), ("blue", b)):
                if seen and not prev_seen[colour]:      # new crossing
                    missed[colour] = prng.random() < cfg.line_miss_prob
                prev_seen[colour] = seen
            orange = o and not missed["orange"]
            blue = b and not missed["blue"]
            next_cam += 1.0 / cfg.camera_hz
        if t >= next_ctrl:
            yaw_meas = yaw + cfg.yaw_drift * t
            cmd = ctl.step(t, yaw_meas, s_true * cfg.odom_scale, scan, orange, blue)
            cmd_v, cmd_w = cmd.linear, cmd.angular
            next_ctrl += 1.0 / cfg.control_hz
            if ctl.state is not last_state:
                events.append((round(t, 2), last_state.value, ctl.state.value, ctl.reason))
                last_state = ctl.state
            if ctl.state is State.DONE:
                break
        w = cmd_w
        if cfg.ackermann_min_radius > 0:
            w_max = abs(cmd_v) / cfg.ackermann_min_radius
            w = max(-w_max, min(w_max, w))
        x += cmd_v * math.cos(yaw) * cfg.dt
        y += cmd_v * math.sin(yaw) * cfg.dt
        yaw = math.atan2(math.sin(yaw + w * cfg.dt), math.cos(yaw + w * cfg.dt))
        s_true += cmd_v * cfg.dt
        t += cfg.dt
        fp = footprint(cfg, x, y, yaw)
        clear = min(fp.distance(p) for p in solids)
        min_clear = min(min_clear, clear)
        if clear <= 0.0:
            collided = True
            events.append((round(t, 2), "COLLISION", ctl.state.value, f"{x:.3f},{y:.3f}"))
            break
        if keep_trajectory and int(round(t / cfg.dt)) % 5 == 0:
            traj.append((x, y, yaw, ctl.state.value))
    sx, sy, _ = cfg.start
    return Result(done=ctl.state is State.DONE, state=ctl.state.value,
                  corners=ctl.corners, time=round(t, 2), collided=collided,
                  min_clearance=round(min_clear, 4), final_pose=(round(x, 3), round(y, 3), round(yaw, 3)),
                  stop_offset=round(math.hypot(x - sx, y - sy), 3), backoffs=ctl.backoffs,
                  trajectory=traj, events=events)


def plot(world: World, result: Result, path: str, title: str = "") -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 6))
    for poly in world.walls:
        ax.fill(*poly.exterior.xy, color="#333333")
    for poly in world.obstacles:
        ax.fill(*poly.exterior.xy, color="#e0b000")
    for colour, line in world.lines:
        ax.plot(*line.xy, color="#ff7a00" if colour == "orange" else "#1f5bd6", lw=2)
    if result.trajectory:
        xs = [p[0] for p in result.trajectory]
        ys = [p[1] for p in result.trajectory]
        ax.plot(xs, ys, color="#c0392b", lw=0.8)
    ax.plot(*result.final_pose[:2], "ko")
    ax.set_aspect("equal")
    ax.set_xlim(-0.85, 0.85)
    ax.set_ylim(-0.85, 0.85)
    ax.set_title(title or f"done={result.done} corners={result.corners} t={result.time}s "
                 f"min_clear={result.min_clearance*1000:.0f}mm")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--direction", choices=["ccw", "cw"], default="ccw")
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="sdf")
    ap.add_argument("--sdf", default=DEFAULT_SDF)
    ap.add_argument("--start", type=float, nargs=3, default=None, metavar=("X", "Y", "YAW"))
    ap.add_argument("--yaw-drift", type=float, default=0.0)
    ap.add_argument("--odom-scale", type=float, default=1.0)
    ap.add_argument("--ackermann", type=float, default=0.0, help="min turn radius (m)")
    ap.add_argument("--line-miss", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()
    world = load_world(args.sdf, SCENARIOS[args.scenario])
    start = tuple(args.start) if args.start else (
        (-0.05, -0.50, 0.0) if args.direction == "ccw" else (-0.05, -0.50, math.pi))
    cfg = SimConfig(start=start, yaw_drift=args.yaw_drift, odom_scale=args.odom_scale,
                    ackermann_min_radius=args.ackermann, line_miss_prob=args.line_miss,
                    seed=args.seed)
    res = run(world, cfg, keep_trajectory=bool(args.plot))
    for ev in res.events:
        print(*ev)
    print(res)
    if args.plot:
        plot(world, res, args.plot)
        print("plot:", args.plot)


if __name__ == "__main__":
    main()
