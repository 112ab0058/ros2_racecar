"""Turn recorded scans and commands into labelled samples. Pure numpy, no ROS."""
import numpy as np

from .features import FULL_LEFT, FULL_RIGHT, LEFT, RIGHT, STRAIGHT

MIN_FORWARD = 0.05      # m/s; slower frames (stopped, ramping, reverse) are not used
MIN_TURN = 0.05         # rad/s; below this the command counts as straight
MAX_CMD_AGE = 0.2       # s; a scan without a command this fresh is not used
# m; a commanded radius below this is full lock. Every driver of the car commands
# either 0.5 m (left / right: phone keys, joystick, corner policy) or 0.35 m (full lock:
# the firmware's minimum); 0.42 m lies between them.
FULL_SPLIT = 0.42
NO_LABEL = -1


def label_of(vx, wz):
    """Return the label of a command (features.LABELS), or NO_LABEL if not driving forward."""
    if vx < MIN_FORWARD:
        return NO_LABEL
    if abs(wz) <= MIN_TURN:
        return STRAIGHT
    full = vx / abs(wz) < FULL_SPLIT
    if wz > 0:
        return FULL_LEFT if full else LEFT
    return FULL_RIGHT if full else RIGHT


def label_scans(scan_t, cmd_t, cmd_vx, cmd_wz, max_age=MAX_CMD_AGE):
    """Return the label of each scan: the last command sent at or before it."""
    scan_t, cmd_t = np.asarray(scan_t), np.asarray(cmd_t)
    last = np.searchsorted(cmd_t, scan_t, side='right') - 1
    labels = np.full(len(scan_t), NO_LABEL, dtype=np.int8)
    for s, c in enumerate(last):
        if c >= 0 and scan_t[s] - cmd_t[c] <= max_age:
            labels[s] = label_of(cmd_vx[c], cmd_wz[c])
    return labels


def command_stats(cmd_vx, cmd_wz):
    """
    Return (speed, turn radius, full-lock radius) of a recording: medians (m/s, m, m).

    bc_node drives with the same values, so the network's left / right / full lock mean
    the same curves as in the recording. A radius is nan if that kind of turn never came.
    """
    vx, wz = np.asarray(cmd_vx, dtype=float), np.asarray(cmd_wz, dtype=float)
    forward = vx >= MIN_FORWARD
    if not forward.any():
        return float('nan'), float('nan'), float('nan')
    turning = forward & (np.abs(wz) > MIN_TURN)
    radius = np.full(len(vx), np.inf)
    radius[turning] = vx[turning] / np.abs(wz[turning])
    full = turning & (radius < FULL_SPLIT)
    normal = turning & ~full
    return (float(np.median(vx[forward])),
            float(np.median(radius[normal])) if normal.any() else float('nan'),
            float(np.median(radius[full])) if full.any() else float('nan'))


def scene_moved(t, features, window=1.0, min_change=0.02, max_range=3.0):
    """
    Return, per scan, True if the lidar's view changed since the scan `window` s earlier.

    features: scan_features rows (0..1 per bin, x max_range = metres). The change is the
    75th percentile of |difference| over the bins (m); at least min_change counts as
    moved. The first `window` seconds have nothing to compare with and count as moved.
    On the stand the wheels turn and commands look like driving, but the room stays
    still (scan noise ~0.5 cm): those samples would teach the network nonsense. On the
    floor at 0.10 m/s the car moves 10 cm in a second and the walls ahead and at an
    angle move with it.
    """
    t = np.asarray(t, dtype=float)
    f = np.asarray(features, dtype=float) * max_range
    prev = np.searchsorted(t, t - window, side='right') - 1
    moved = np.ones(len(t), dtype=bool)
    ok = prev >= 0
    change = np.percentile(np.abs(f[ok] - f[prev[ok]]), 75, axis=1) if ok.any() else []
    moved[ok] = np.asarray(change) >= min_change
    return moved


def before_events(t, events, seconds):
    """Return a mask of the times t that lie less than `seconds` before any event time."""
    t = np.asarray(t, dtype=float)
    mask = np.zeros(len(t), dtype=bool)
    for e in events:
        mask |= (t <= e) & (t > e - seconds)
    return mask


def block_split(t, block=3.0, every=5, guard=0.5):
    """
    Return boolean masks (train, val) that split one recording by time.

    Neighbouring scans are almost identical, so a random split would put near
    copies of the training data into validation and report a score that is too
    good. Every `every`-th block of `block` seconds is validation instead, and
    training frames closer than `guard` seconds to a validation block are dropped.
    """
    t = np.asarray(t)
    pos = (t - t[0]) / block
    b = np.floor(pos).astype(int)
    frac = pos - b
    val = (b % every) == every - 1
    before = ((b + 1) % every == every - 1) & (frac > 1.0 - guard / block)
    after = ((b - 1) % every == every - 1) & (frac < guard / block)
    return ~val & ~before & ~after, val
