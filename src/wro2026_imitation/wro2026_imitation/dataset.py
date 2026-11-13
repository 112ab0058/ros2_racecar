"""Turn recorded scans and commands into labelled samples. Pure numpy, no ROS."""
import numpy as np

from .features import LEFT, RIGHT, STRAIGHT

MIN_FORWARD = 0.05      # m/s; slower frames (stopped, ramping, reverse) are not used
MIN_TURN = 0.05         # rad/s; below this the command counts as straight
MAX_CMD_AGE = 0.2       # s; a scan without a command this fresh is not used
NO_LABEL = -1


def label_of(vx, wz):
    """Return left / straight / right for a command, or NO_LABEL if not driving forward."""
    if vx < MIN_FORWARD:
        return NO_LABEL
    if wz > MIN_TURN:
        return LEFT
    if wz < -MIN_TURN:
        return RIGHT
    return STRAIGHT


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
    Return (speed, turn radius) of a recording: medians over the forward commands.

    bc_node drives with the same values, so the network's left / right mean the
    same curve as in the recording. The radius is nan if there was no turn.
    """
    vx, wz = np.asarray(cmd_vx, dtype=float), np.asarray(cmd_wz, dtype=float)
    forward = vx >= MIN_FORWARD
    if not forward.any():
        return float('nan'), float('nan')
    turning = forward & (np.abs(wz) > MIN_TURN)
    radius = np.median(vx[turning] / np.abs(wz[turning])) if turning.any() else np.nan
    return float(np.median(vx[forward])), float(radius)


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
