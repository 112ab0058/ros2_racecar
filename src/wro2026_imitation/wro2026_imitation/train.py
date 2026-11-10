"""
Train the imitation network from dataset.npz files. Reads files only, no motors.

    ros2 run wro2026_imitation bc_train ~/r550_map/*/dataset.npz -o ~/r550_map/model.npz
"""
import argparse
import sys

import numpy as np

from .dataset import block_split
from .features import BIN_DEG, HALF_FOV_DEG, LABELS, MAX_RANGE, mirror, N_BINS, scan_features
from .model import balanced_accuracy, confusion, MLP, train


def load(paths):
    """
    Return (inputs, labels, train mask, validation mask, driving) over all recordings.

    driving has one row (speed, turn radius) per recording; nan where not stored.
    """
    xs, ys, tr, va, driving = [], [], [], [], []
    for path in paths:
        with np.load(path) as d:
            xs.append(scan_features(d['ranges']))
            ys.append(d['label'].astype(int))
            is_train, is_val = block_split(d['t'])
            tr.append(is_train)
            va.append(is_val)
            driving.append([float(d[k]) if k in d.files else np.nan
                            for k in ('speed', 'turn_radius')])
    return (np.concatenate(xs), np.concatenate(ys), np.concatenate(tr), np.concatenate(va),
            np.array(driving))


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


def class_weights(y, n=3):
    """Return one weight per sample so that every class counts the same in total."""
    counts = np.maximum(np.bincount(y, minlength=n), 1)
    return (len(y) / (n * counts))[y]


def report(name, model, x, y):
    """Print accuracy and balanced accuracy, and return the confusion table."""
    table = confusion(y, model.predict(x))
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
    args = ap.parse_args(argv)

    x, y, is_train, is_val, driving = load(args.datasets)
    counts = np.bincount(y, minlength=len(LABELS))
    print('samples: %d from %d recording(s): %s' % (len(y), len(args.datasets), ', '.join(
        '%s %d' % (name, c) for name, c in zip(LABELS, counts))))
    speed = recorded(driving[:, 0], 'speed')
    radius = recorded(driving[:, 1], 'turn radius')
    if not is_train.any() or not is_val.any():
        print('not enough data to split into training and validation')
        return 1
    x_val, y_val = x[is_val], y[is_val]
    x_tr, y_tr = x[is_train], y[is_train]
    x_m, y_m = mirror(x_tr, y_tr)
    x_tr, y_tr = np.concatenate([x_tr, x_m]), np.concatenate([y_tr, y_m])
    print('split by time: train %d (mirrored: %d), validation %d'
          % (is_train.sum(), len(y_tr), is_val.sum()))

    hidden = tuple(int(v) for v in args.hidden.split(','))
    model = MLP(N_BINS, hidden=hidden, n_out=len(LABELS), seed=args.seed)
    epoch, loss = train(model, x_tr, y_tr, x_val, y_val, epochs=args.epochs, seed=args.seed,
                        sample_weight=class_weights(y_tr), val_weight=class_weights(y_val))
    print('network %d-%s-%d, %d parameters; best epoch %d, validation loss %.3f'
          % (N_BINS, '-'.join(str(h) for h in hidden), len(LABELS), model.n_parameters(),
             epoch, loss))

    majority = np.bincount(y_val, minlength=len(LABELS)).max() / len(y_val)
    print('always guess the most common class: %.1f %%; balanced 33.3 %%' % (100.0 * majority))
    report('train (mirrored)', model, x_tr, y_tr)
    table = report('validation', model, x_val, y_val)
    print('validation table (rows: what you pressed, columns: what the network chose)')
    print('%10s' % '' + ''.join('%10s' % name for name in LABELS))
    for name, row in zip(LABELS, table):
        print('%10s' % name + ''.join('%10d' % v for v in row))

    model.save(args.output, half_fov_deg=HALF_FOV_DEG, bin_deg=BIN_DEG, max_range=MAX_RANGE,
               labels=LABELS, hidden=hidden, speed=speed, turn_radius=radius)
    print('saved %s' % args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
