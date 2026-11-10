"""
Lidar scan -> input vector of the imitation network. Pure numpy, no ROS.

Training and driving call the same code, so the network sees identical inputs.

Only the front field of view is used (+-110 deg). The sectors masked for our own
car (168-243 deg, wro2026_lidar_n10p) lie behind it, so they cannot leak into the
input, and the vector stays exactly left-right symmetric, which mirroring needs.
"""
from functools import lru_cache

import numpy as np

HALF_FOV_DEG = 110.0
BIN_DEG = 5.0
N_BINS = int(round(2 * HALF_FOV_DEG / BIN_DEG))
MAX_RANGE = 3.0     # m; farther returns and beams without a return count as this

LEFT, STRAIGHT, RIGHT = 0, 1, 2
LABELS = ('left', 'straight', 'right')


@lru_cache(maxsize=4)
def bin_beams(n_beams):
    """
    Return the beam indices of each bin; bin 0 is at -110 deg (car's right).

    ROS angles grow counter-clockwise, so positive angles are on the car's left
    and use the upper half of the vector. A beam exactly at 0 deg goes to both
    middle bins, which keeps the two halves mirror images of each other.
    """
    bins = [[] for _ in range(N_BINS)]
    half = N_BINS // 2
    for k in range(n_beams):
        deg = k * 360.0 / n_beams
        theta = deg if deg <= 180.0 else deg - 360.0
        slot = int(abs(theta) // BIN_DEG)
        if slot >= half:
            continue
        if theta >= 0.0:
            bins[half + slot].append(k)
        if theta <= 0.0:
            bins[half - 1 - slot].append(k)
    return tuple(tuple(b) for b in bins)


def scan_features(ranges):
    """
    Return one row of N_BINS values in 0..1 for each scan (input: 1-D or 2-D).

    A value is the closest obstacle in that bin divided by MAX_RANGE; a beam
    without a return (inf) counts as far away.
    """
    r = np.atleast_2d(np.asarray(ranges, dtype=np.float32))
    r = np.where(np.isfinite(r), r, MAX_RANGE)
    r = np.clip(r, 0.0, MAX_RANGE) / MAX_RANGE
    return np.stack([r[:, list(beams)].min(axis=1) for beams in bin_beams(r.shape[1])], axis=1)


def mirror(x, y):
    """Return left-right mirrored copies: reversed inputs, left and right labels swapped."""
    swap = np.array([RIGHT, STRAIGHT, LEFT])
    return x[:, ::-1], swap[y]
