"""Inputs, labels, the time split, the network and the training command (no ROS, no motors)."""
import numpy as np
import pytest
from wro2026_imitation import extract
from wro2026_imitation.dataset import (before_events, block_split, command_stats, label_of,
                                       label_scans, NO_LABEL, scene_moved)
from wro2026_imitation.features import (bin_beams, FULL_LEFT, FULL_RIGHT, LABELS, LEFT,
                                        MAX_RANGE, mirror, N_BINS, RIGHT, scan_features,
                                        STRAIGHT)
from wro2026_imitation.model import balanced_accuracy, confusion, MLP, train
from wro2026_imitation.policy import check_model
from wro2026_imitation.train import class_weights, main, mirrored

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
    assert mirror(x, np.array([FULL_LEFT]))[1][0] == FULL_RIGHT


def test_mirroring_leaves_out_full_lock_by_default():
    """The car turns 0.403 m left but 0.292 m right at full lock: no mirrored timing."""
    x = np.zeros((5, N_BINS))
    y = np.array([LEFT, STRAIGHT, RIGHT, FULL_LEFT, FULL_RIGHT])
    assert sorted(mirrored(x, y, 'turns')[1].tolist()) == [LEFT, STRAIGHT, RIGHT]
    assert len(mirrored(x, y, 'all')[1]) == 5 and len(mirrored(x, y, 'none')[1]) == 0


@pytest.mark.parametrize('vx, wz, expected', [
    (0.1, 0.2, LEFT), (0.1, -0.2, RIGHT), (0.1, 0.0, STRAIGHT),
    (0.1, 0.1 / 0.35, FULL_LEFT), (0.1, -0.1 / 0.35, FULL_RIGHT),    # keys, joystick, corner
    (0.06, 0.12, LEFT), (0.06, 0.06 / 0.35, FULL_LEFT),              # radius, not wz, decides
    (0.0, 0.0, NO_LABEL), (0.03, 0.2, NO_LABEL), (-0.1, 0.0, NO_LABEL)])
def test_labels(vx, wz, expected):
    assert label_of(vx, wz) == expected


def test_scan_gets_the_last_command_before_it():
    labels = label_scans([1.0, 1.1, 2.0, 5.0], [1.05, 1.95], [0.1, 0.1], [0.2, -0.2])
    assert labels.tolist() == [NO_LABEL, LEFT, RIGHT, NO_LABEL]    # first: none yet; last: stale


def test_command_stats_are_the_driven_speed_and_radii():
    # stopped, ramping, then 0.1 m/s with 0.5 m turns and full lock (0.35 m)
    vx = [0.0, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1]
    wz = [0.0, 0.1, 0.2, 0.2, 0.0, -0.2, -0.1 / 0.35]
    assert command_stats(vx, wz) == pytest.approx((0.1, 0.5, 0.35))
    stats = command_stats([0.1, 0.1], [0.0, 0.0])                  # never turned
    assert stats[0] == pytest.approx(0.1) and np.isnan(stats[1]) and np.isnan(stats[2])


def test_view_standing_still_while_driving_is_found():
    """Wheels in the air: commands say driving, the room stays put (stand tests)."""
    t = np.arange(0.0, 5.0, 0.1)
    rng = np.random.default_rng(0)
    room = np.full((len(t), N_BINS), 0.5) + rng.normal(0.0, 0.002, (len(t), N_BINS))
    assert scene_moved(t, room)[t >= 1.0].sum() == 0
    driving = 0.5 - 0.1 * t[:, None] / MAX_RANGE * np.linspace(0.3, 1.0, N_BINS)[None]
    assert scene_moved(t, driving).all()          # walls 3-10 cm closer each second


def test_samples_before_a_stop_are_marked():
    t = np.arange(0.0, 10.0, 0.5)
    assert t[before_events(t, [5.0], 1.5)].tolist() == [4.0, 4.5, 5.0]


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


def bag(scan_t, ranges, cmd_t, vx, wz, stops=(), topic='/scan_filled'):
    return {'scan_topic': topic, 'scan_t': np.array(scan_t), 'ranges': ranges,
            'cmd_t': np.array(cmd_t), 'vx': np.array(vx), 'wz': np.array(wz),
            'stops': list(stops)}


def test_recording_without_driving_writes_no_dataset(tmp_path, monkeypatch):
    monkeypatch.setattr(extract, 'read_bag', lambda uri, topic: bag(
        [1.0, 1.1], [scan(), scan()], [1.0], [0.0], [0.0]))
    assert extract.extract(str(tmp_path)) == 0
    assert not (tmp_path / 'dataset.npz').exists()


def test_recording_stores_labels_who_drove_speed_and_radii(tmp_path, monkeypatch):
    t = np.arange(0.0, 4.0, 0.1)
    # a corridor whose walls come 1 cm closer every scan: the car really drives
    ranges = [np.float32(2.0 - 0.01 * k) * np.ones(BEAMS, np.float32) for k in range(len(t))]
    wz = np.where(t < 1.0, 0.0, np.where(t < 2.0, 0.2, np.where(t < 3.0, -0.1 / 0.35, 0.2)))
    monkeypatch.setattr(extract, 'read_bag', lambda uri, topic: bag(
        t, ranges, t - 0.01, np.full(len(t), 0.1), wz, stops=[3.95]))
    (tmp_path / 'web.log').write_text(
        '[INFO] [0.500000000] [web_teleop_node]: phone: 直走 (vx +0.10, wz +0.00)\n'
        '[INFO] [1.950000000] [web_teleop_node]: phone: 自動 (vx +0.10, wz -0.29)\n',
        encoding='utf-8')
    n = extract.extract(str(tmp_path))
    with np.load(tmp_path / 'dataset.npz') as d:
        labels, src = d['label'].tolist(), d['source'].tolist()
        assert n == len(labels) and str(d['scan_topic']) == '/scan_filled'
        assert labels[:10] == [STRAIGHT] * 10 and FULL_RIGHT in labels and LEFT in labels
        assert d['t'].max() <= 3.95 - 1.5                     # dropped before the stop
        assert src[:10] == [extract.HAND] * 10 and src[-1] == extract.RULE
        stats = (float(d['speed']), float(d['turn_radius']), float(d['full_turn_radius']))
        assert stats == pytest.approx((0.1, 0.5, 0.35))


def test_stand_recording_gives_no_samples(tmp_path, monkeypatch):
    t = np.arange(0.0, 5.0, 0.1)
    ranges = [np.full(BEAMS, 1.5, np.float32) for _ in t]       # the room does not move
    monkeypatch.setattr(extract, 'read_bag', lambda uri, topic: bag(
        t, ranges, t, np.full(len(t), 0.1), np.zeros(len(t))))
    assert extract.extract(str(tmp_path)) <= 10                 # only the first second


def test_train_command_end_to_end(tmp_path, capsys):
    """A wall close on one side means steer away: the command must learn that."""
    rng = np.random.default_rng(6)
    ranges, labels = [], []
    for _ in range(1500):
        c = rng.integers(0, 5)
        r = np.full(BEAMS, 2.5, dtype=np.float32) + rng.normal(0.0, 0.02, BEAMS).astype(np.float32)
        if c == LEFT:       # wall close on the right
            r[[beam_at(-d) for d in range(30, 90)]] = 0.3
        elif c == RIGHT:    # wall close on the left
            r[[beam_at(d) for d in range(30, 90)]] = 0.3
        elif c == FULL_LEFT:    # wall ahead, open on the left
            r[[beam_at(d) for d in range(-90, 20)]] = 0.25
        elif c == FULL_RIGHT:   # wall ahead, open on the right
            r[[beam_at(d) for d in range(-20, 90)]] = 0.25
        ranges.append(r)
        labels.append(c)
    data = str(tmp_path / 'dataset.npz')
    np.savez(data, ranges=np.stack(ranges), label=np.array(labels, np.int8),
             t=np.arange(1500) * 0.1, source=np.zeros(1500, np.int8), speed=0.1,
             turn_radius=0.5, full_turn_radius=0.35, scan_topic='/scan_filled')
    out = str(tmp_path / 'model.npz')
    assert main([data, '-o', out, '--epochs', '80']) == 0
    model, meta = MLP.load(out)
    check_model(model, meta)                    # bc_node will accept it
    assert meta['labels'] == list(LABELS) and meta['source'] == 'all'
    driving = (meta['speed'], meta['turn_radius'], meta['full_turn_radius'])
    assert driving == pytest.approx((0.1, 0.5, 0.35))
    assert meta['val_accuracy'] > 0.9 and meta['scan_topics'] == ['/scan_filled']
    r = np.full((1, BEAMS), 2.5, dtype=np.float32)
    r[0, [beam_at(d) for d in range(30, 90)]] = 0.3
    assert model.predict(scan_features(r))[0] == RIGHT
    r = np.full((1, BEAMS), 2.5, dtype=np.float32)
    r[0, [beam_at(d) for d in range(-90, 20)]] = 0.25
    assert model.predict(scan_features(r))[0] == FULL_LEFT
    assert 'validation' in capsys.readouterr().out
    assert main([data, '-o', out, '--source', 'rule']) == 1    # nobody but Ray drove
