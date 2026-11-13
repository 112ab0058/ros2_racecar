"""Inputs, labels, the time split, the network and the training command (no ROS, no motors)."""
import numpy as np
import pytest
from wro2026_imitation import extract
from wro2026_imitation.dataset import block_split, command_stats, label_of, label_scans, NO_LABEL
from wro2026_imitation.features import (bin_beams, LEFT, MAX_RANGE, mirror, N_BINS, RIGHT,
                                        scan_features, STRAIGHT)
from wro2026_imitation.model import balanced_accuracy, confusion, MLP, train
from wro2026_imitation.policy import check_model
from wro2026_imitation.train import class_weights, main

BEAMS = 480


def scan(far=np.inf):
    return np.full(BEAMS, far, dtype=np.float32)


def beam_at(deg):
    return int(round((deg % 360.0) / 0.75)) % BEAMS


def test_bins_cover_the_front_only():
    bins = bin_beams(BEAMS)
    assert len(bins) == N_BINS == 44
    assert all(len(b) >= 6 for b in bins)
    used = {k for b in bins for k in b}
    for k in used:
        assert not 168.0 <= k * 0.75 <= 243.0     # the masked sectors of our own car


def test_bins_are_exact_mirror_images():
    bins = bin_beams(BEAMS)
    for i in range(N_BINS):
        assert sorted((BEAMS - k) % BEAMS for k in bins[i]) == sorted(bins[N_BINS - 1 - i])


def test_no_return_counts_as_far():
    assert scan_features(scan()).tolist() == [[1.0] * N_BINS]


def test_bin_takes_the_closest_obstacle():
    r = scan()
    r[beam_at(30.0)] = 1.5
    r[beam_at(31.0)] = 0.6
    x = scan_features(r)[0]
    assert x.min() == pytest.approx(0.6 / MAX_RANGE)


def test_left_of_the_car_is_the_upper_half():
    r = scan()
    r[beam_at(45.0)] = 0.5      # +45 deg = car's left in ROS
    assert np.argmin(scan_features(r)[0]) >= N_BINS // 2


def test_mirror_reverses_inputs_and_swaps_left_right():
    x = np.arange(N_BINS, dtype=float)[None]
    m, y = mirror(x, np.array([LEFT]))
    assert m[0, 0] == N_BINS - 1 and y[0] == RIGHT
    assert mirror(x, np.array([STRAIGHT]))[1][0] == STRAIGHT


@pytest.mark.parametrize('vx, wz, expected', [
    (0.1, 0.286, LEFT), (0.1, -0.286, RIGHT), (0.1, 0.0, STRAIGHT),
    (0.0, 0.0, NO_LABEL), (0.03, 0.2, NO_LABEL), (-0.1, 0.0, NO_LABEL)])
def test_labels(vx, wz, expected):
    assert label_of(vx, wz) == expected


def test_scan_gets_the_last_command_before_it():
    labels = label_scans([1.0, 1.1, 2.0, 5.0], [1.05, 1.95], [0.1, 0.1], [0.3, -0.3])
    assert labels.tolist() == [NO_LABEL, LEFT, RIGHT, NO_LABEL]    # first: none yet; last: stale


def test_command_stats_are_the_driven_speed_and_radius():
    # stopped, ramping, then 0.1 m/s with 0.5 m turns; the ramp keeps the curvature
    vx = [0.0, 0.05, 0.1, 0.1, 0.1, 0.1]
    wz = [0.0, 0.1, 0.2, 0.2, 0.0, -0.2]
    assert command_stats(vx, wz) == pytest.approx((0.1, 0.5))
    assert np.isnan(command_stats([0.1, 0.1], [0.0, 0.0])[1])     # never turned


def test_time_split_has_no_overlap_and_keeps_a_gap():
    t = np.arange(0.0, 120.0, 0.1)
    tr, va = block_split(t)
    assert not (tr & va).any()
    assert 0.15 < va.mean() < 0.25
    assert np.abs(t[tr][:, None] - t[va][None]).min() > 0.45


def test_gradients_match_numerical_ones():
    rng = np.random.default_rng(1)
    x, y = rng.random((12, 5)), rng.integers(0, 3, 12)
    model = MLP(5, hidden=(4, 3), seed=2)
    s = rng.random(12) + 0.5
    _, gw, gb = model.loss_and_grads(x, y, s)
    for arrays, grads in ((model.w, gw), (model.b, gb)):
        for a, g in zip(arrays, grads):
            for index in np.ndindex(a.shape):
                old = a[index]
                a[index] = old + 1e-6
                up = model.loss_and_grads(x, y, s)[0]
                a[index] = old - 1e-6
                down = model.loss_and_grads(x, y, s)[0]
                a[index] = old
                assert g[index] == pytest.approx((up - down) / 2e-6, abs=1e-5)


def test_network_learns_a_simple_rule():
    rng = np.random.default_rng(3)
    x = rng.random((600, N_BINS))
    y = (x[:, :22].mean(axis=1) > x[:, 22:].mean(axis=1)).astype(int)
    model = MLP(N_BINS, n_out=2, seed=4)
    train(model, x[:500], y[:500], x[500:], y[500:], epochs=150)
    assert (model.predict(x[500:]) == y[500:]).mean() > 0.9


def test_save_and_load_give_the_same_outputs(tmp_path):
    x = np.random.default_rng(5).random((4, N_BINS))
    model = MLP(N_BINS)
    path = str(tmp_path / 'm.npz')
    model.save(path, labels=['a'])
    loaded, meta = MLP.load(path)
    assert meta == {'labels': ['a']}
    assert np.array_equal(model.predict_proba(x), loaded.predict_proba(x))


def test_balanced_accuracy_ignores_missing_classes():
    table = confusion([0, 0, 1], [0, 1, 1])
    assert balanced_accuracy(table) == pytest.approx((0.5 + 1.0) / 2)


def test_class_weights_balance_the_classes():
    y = np.array([0] * 90 + [1] * 10)
    w = class_weights(y, n=2)
    assert w[y == 0].sum() == pytest.approx(w[y == 1].sum())


def test_recording_without_driving_writes_no_dataset(tmp_path, monkeypatch):
    def no_driving(uri):
        return np.array([1.0, 1.1]), [scan(), scan()], np.array([1.0]), np.array([0.0]), \
            np.array([0.0])

    monkeypatch.setattr(extract, 'read_bag', no_driving)
    assert extract.extract(str(tmp_path)) == 0
    assert not (tmp_path / 'dataset.npz').exists()


def test_recording_stores_speed_and_radius(tmp_path, monkeypatch):
    def driving(uri):
        return np.array([1.0, 1.1, 1.2]), [scan()] * 3, np.array([0.95, 1.05, 1.15]), \
            np.array([0.1, 0.1, 0.1]), np.array([0.0, 0.2, -0.2])

    monkeypatch.setattr(extract, 'read_bag', driving)
    assert extract.extract(str(tmp_path)) == 3
    with np.load(tmp_path / 'dataset.npz') as d:
        assert d['label'].tolist() == [STRAIGHT, LEFT, RIGHT]
        assert (float(d['speed']), float(d['turn_radius'])) == pytest.approx((0.1, 0.5))


def test_train_command_end_to_end(tmp_path, capsys):
    """A wall close on one side means steer away: the command must learn that."""
    rng = np.random.default_rng(6)
    ranges, labels = [], []
    for _ in range(900):
        c = rng.integers(0, 3)
        r = np.full(BEAMS, 2.5, dtype=np.float32) + rng.normal(0.0, 0.02, BEAMS).astype(np.float32)
        if c == LEFT:       # wall close on the right
            r[[beam_at(-d) for d in range(30, 90)]] = 0.3
        elif c == RIGHT:    # wall close on the left
            r[[beam_at(d) for d in range(30, 90)]] = 0.3
        ranges.append(r)
        labels.append(c)
    data = str(tmp_path / 'dataset.npz')
    np.savez(data, ranges=np.stack(ranges), label=np.array(labels, np.int8),
             t=np.arange(900) * 0.1, speed=0.1, turn_radius=0.5)
    out = str(tmp_path / 'model.npz')
    assert main([data, '-o', out, '--epochs', '60']) == 0
    model, meta = MLP.load(out)
    check_model(model, meta)                    # bc_node will accept it
    assert (meta['speed'], meta['turn_radius']) == pytest.approx((0.1, 0.5))
    r = np.full((1, BEAMS), 2.5, dtype=np.float32)
    r[0, [beam_at(d) for d in range(30, 90)]] = 0.3
    assert model.predict(scan_features(r))[0] == RIGHT
    assert 'validation' in capsys.readouterr().out
