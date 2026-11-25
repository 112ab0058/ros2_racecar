"""
Train the imitation network from dataset.npz files. Reads files only, no motors.

    ros2 run wro2026_imitation bc_train ~/r550_map/*/dataset.npz -o ~/r550_map/model.npz

Mirroring (--mirror): a left-right mirrored copy of the training data doubles it. The car
does not steer the same both ways (2026-11-25: full lock drives 0.403 m to the left and
0.292 m to the right, a 0.5 m turn 0.496 / 0.373 m; the firmware gives the left wheel too
little angle), so mirrored samples teach one side the other side's timing. 2-D sim
(tools/real/sim_bc.py, 2026-11-25, corner rule as teacher, 24 new start poses, 3 laps):
none ccw 8/12 cw 12/12, turns (left / straight / right only) 3/12 and 11/12, all 0/12 and
11/12. Default 'none': record both directions instead.
"""
import argparse
import os
import sys
import time

import numpy as np

from .dataset import block_split
from .extract import HAND, RULE
from .features import (BIN_DEG, FULL_LEFT, FULL_RIGHT, HALF_FOV_DEG, LABELS, MAX_RANGE, mirror,
                       N_BINS, scan_features)
from .model import balanced_accuracy, confusion, MLP, train

N_CLASSES = len(LABELS)


def load(paths, source='all'):
    """
    Return (inputs, labels, train mask, validation mask, driving, scan topics).

    driving has one row (speed, turn radius, full-lock radius) per recording; nan where
    not stored. source: 'all', 'hand' (Ray drove) or 'rule' (the corner rule drove).
    """
    xs, ys, tr, va, driving, topics = [], [], [], [], [], set()
    for path in paths:
        with np.load(path) as d:
            src = d['source'] if 'source' in d.files else np.full(len(d['label']), HAND)
            pick = np.ones(len(src), dtype=bool) if source == 'all' else (
                src == (HAND if source == 'hand' else RULE))
            if not pick.any():
                continue
            is_train, is_val = block_split(d['t'])
            xs.append(scan_features(d['ranges'][pick]))
            ys.append(d['label'][pick].astype(int))
            tr.append(is_train[pick])
            va.append(is_val[pick])
            driving.append([float(d[k]) if k in d.files else np.nan
                            for k in ('speed', 'turn_radius', 'full_turn_radius')])
            topics.add(str(d['scan_topic']) if 'scan_topic' in d.files else '/scan')
    if not xs:
        return None
    return (np.concatenate(xs), np.concatenate(ys), np.concatenate(tr), np.concatenate(va),
            np.array(driving), sorted(topics))


def recorded(values, name):
    """Return the median of a driving value over the recordings, or None if none stored it."""
    v = values[np.isfinite(values)]
    if not len(v):
        print('%s: not in the datasets; bc_node will need it as a parameter' % name)
        return None
    if v.max() > 1.1 * v.min():
        print('WARNING: %s differs between recordings: %s'
              % (name, ', '.join('%.3f' % a for a in v)))
    return float(np.median(v))


def class_weights(y, n=N_CLASSES):
    """Return one weight per sample so that every class counts the same in total."""
    counts = np.maximum(np.bincount(y, minlength=n), 1)
    return (len(y) / (np.count_nonzero(np.bincount(y, minlength=n)) * counts))[y]


def mirrored(x, y, how):
    """Return the mirrored copies to add ('none', 'turns' = no full lock, 'all')."""
    if how == 'none':
        return x[:0], y[:0]
    pick = np.ones(len(y), dtype=bool) if how == 'all' else ~np.isin(y, (FULL_LEFT, FULL_RIGHT))
    return mirror(x[pick], y[pick])


def report(name, model, x, y):
    """Print accuracy and balanced accuracy, and return the confusion table."""
    table = confusion(y, model.predict(x), n=N_CLASSES)
    print('%-22s accuracy %5.1f %%   balanced %5.1f %%   (%d samples)'
          % (name, 100.0 * table.trace() / max(table.sum(), 1),
             100.0 * balanced_accuracy(table), table.sum()))
    return table


def main(argv=None):
    """Entry point."""
    ap = argparse.ArgumentParser(description='Train the imitation network.')
    ap.add_argument('datasets', nargs='+', help='dataset.npz files from bc_extract')
    ap.add_argument('-o', '--output', required=True, help='model file to write (.npz)')
    ap.add_argument('--hidden', default='32,16', help='hidden layer sizes')
    ap.add_argument('--epochs', type=int, default=300)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--source', choices=('all', 'hand', 'rule'), default='all',
                    help='whose driving to learn: Ray (hand), the corner rule, or both')
    ap.add_argument('--mirror', choices=('none', 'turns', 'all'), default='none',
                    help='mirrored copies: none (default; the car steers unlike both ways), '
                         'turns = left / straight / right only, all')
    args = ap.parse_args(argv)

    loaded = load(args.datasets, args.source)
    if loaded is None:
        print('no samples of source %s in these datasets' % args.source)
        return 1
    x, y, is_train, is_val, driving, topics = loaded
    counts = np.bincount(y, minlength=N_CLASSES)
    print('samples: %d (%s) from %d recording(s): %s' % (
        len(y), args.source, len(driving), ', '.join(
            '%s %d' % (name, c) for name, c in zip(LABELS, counts))))
    if len(topics) > 1:
        print('WARNING: the recordings used different scan topics: %s' % ', '.join(topics))
    speed = recorded(driving[:, 0], 'speed')
    radius = recorded(driving[:, 1], 'turn radius')
    full = recorded(driving[:, 2], 'full-lock radius')
    if not is_train.any() or not is_val.any():
        print('not enough data to split into training and validation')
        return 1
    x_val, y_val = x[is_val], y[is_val]
    x_tr, y_tr = x[is_train], y[is_train]
    x_m, y_m = mirrored(x_tr, y_tr, args.mirror)
    x_tr, y_tr = np.concatenate([x_tr, x_m]), np.concatenate([y_tr, y_m])
    print('split by time: train %d (+ %d mirrored, %s), validation %d'
          % (is_train.sum(), len(y_m), args.mirror, is_val.sum()))

    hidden = tuple(int(v) for v in args.hidden.split(','))
    model = MLP(N_BINS, hidden=hidden, n_out=N_CLASSES, seed=args.seed)
    epoch, loss = train(model, x_tr, y_tr, x_val, y_val, epochs=args.epochs, seed=args.seed,
                        sample_weight=class_weights(y_tr), val_weight=class_weights(y_val))
    print('network %d-%s-%d, %d parameters; best epoch %d, validation loss %.3f'
          % (N_BINS, '-'.join(str(h) for h in hidden), N_CLASSES, model.n_parameters(),
             epoch, loss))

    seen = np.bincount(y_val, minlength=N_CLASSES)
    print('always guess the most common class: %.1f %%; balanced %.1f %%'
          % (100.0 * seen.max() / len(y_val), 100.0 / max(np.count_nonzero(seen), 1)))
    report('train', model, x_tr, y_tr)
    table = report('validation', model, x_val, y_val)
    print('validation table (rows: what the driver did, columns: what the network chose)')
    print('%11s' % '' + ''.join('%11s' % name for name in LABELS))
    for name, row in zip(LABELS, table):
        print('%11s' % name + ''.join('%11d' % v for v in row))

    model.save(args.output, half_fov_deg=HALF_FOV_DEG, bin_deg=BIN_DEG, max_range=MAX_RANGE,
               labels=LABELS, hidden=hidden, speed=speed, turn_radius=radius,
               full_turn_radius=full, scan_topics=topics, source=args.source,
               mirror=args.mirror, samples=int(len(y)),
               val_accuracy=float(table.trace() / max(table.sum(), 1)),
               val_balanced=float(balanced_accuracy(table)),
               trained=time.strftime('%Y-%m-%d %H:%M'),
               datasets=[os.path.basename(os.path.dirname(os.path.abspath(p)))
                         for p in args.datasets])
    print('saved %s' % args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
