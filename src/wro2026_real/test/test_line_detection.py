"""Line detection on numpy-generated images (no camera, no OpenCV needed)."""
import numpy as np
import pytest

from wro2026_real.line_detection import (bgr_to_hsv, DEFAULTS, detect_lines, dilate, erode,
                                         image_to_bgr, largest_blob, open_close, to_array)

H, W = 240, 320
ORANGE = (0, 140, 255)   # BGR -> HSV about (16, 255, 255)
BLUE = (255, 50, 0)      # BGR -> HSV about (114, 255, 255)
GRAY = (128, 128, 128)   # saturation 0: never orange or blue


def canvas(color=GRAY):
    img = np.empty((H, W, 3), dtype=np.uint8)
    img[:] = color
    return img


def stripe(img, color, x0, x1, y0=150, y1=170):
    img[y0:y1, x0:x1] = color
    return img


def test_only_orange():
    img = stripe(canvas(), ORANGE, 40, 140)
    out = to_array(detect_lines(img))
    assert out[0] == 1.0
    assert out[1] == pytest.approx(89.0, abs=1)  # centre of x 40..139
    assert out[2] == 100 * 20
    assert out[3] == 1.0  # 2000 >= trigger 1000
    assert out[4:] == [0.0, -1.0, 0.0, 0.0, 0.0]


def test_only_blue_left_means_clockwise():
    img = stripe(canvas(), BLUE, 20, 120)
    out = to_array(detect_lines(img))
    assert out[:4] == [0.0, -1.0, 0.0, 0.0]
    assert out[4] == 1.0
    assert out[5] < W / 2
    assert out[8] == 1.0


def test_no_line():
    img = canvas()
    img[:60] = (255, 255, 255)
    img[60:120] = (0, 0, 0)
    img[120:180] = (0, 200, 0)  # green: saturated but outside both hue ranges
    out = to_array(detect_lines(img))
    assert out == [0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0]


def test_both_lines_blue_right_means_counter_clockwise():
    img = stripe(canvas(), ORANGE, 30, 130)
    stripe(img, BLUE, 200, 300, 100, 140)
    det = detect_lines(img)
    out = to_array(det)
    assert out[0] == 1.0 and out[4] == 1.0
    assert out[1] == pytest.approx(79.5, abs=1)
    assert out[5] == pytest.approx(249.5, abs=1)
    assert out[6] == 100 * 40
    assert out[8] == -1.0
    assert len(out) == 9


def test_small_blob_is_reported_but_not_detected():
    img = stripe(canvas(), ORANGE, 100, 110, 100, 110)  # 100 px < area_threshold 200
    out = to_array(detect_lines(img))
    assert out[0] == 0.0 and out[1] == -1.0
    assert out[2] == 100.0
    assert out[3] == 0.0


def test_trigger_threshold():
    img = stripe(canvas(), ORANGE, 100, 130, 100, 110)  # 300 px: detected, no trigger
    out = to_array(detect_lines(img))
    assert out[0] == 1.0 and out[3] == 0.0
    out = to_array(detect_lines(img, {'trigger_area': 300}))
    assert out[3] == 1.0


def test_roi_excludes_top_and_keeps_full_image_cx():
    img = stripe(canvas(), ORANGE, 40, 140, 20, 40)  # only in the top sixth
    assert to_array(detect_lines(img, {'roi_top_ratio': 0.5}))[0] == 0.0
    img = stripe(canvas(), BLUE, 200, 300)
    det = detect_lines(img, {'roi_left_ratio': 0.5})
    assert det.roi == (0, H, 160, W)
    assert det.blue.cx == pytest.approx(249.5, abs=1)  # full-image coordinates


def test_hsv_ranges_are_parameters():
    img = stripe(canvas(), (0, 200, 0), 40, 140)  # green, H = 60
    assert to_array(detect_lines(img))[0] == 0.0
    params = {'orange_lower': (50, 100, 100), 'orange_upper': (70, 255, 255)}
    assert to_array(detect_lines(img, params))[0] == 1.0


def test_largest_blob_wins_and_diagonals_connect():
    mask = np.zeros((20, 20), dtype=bool)
    mask[1:3, 1:3] = True       # 4 px
    for i in range(10):          # diagonal line, 8-connected: 10 px
        mask[8 + i, 5 + i] = True
    area, cx = largest_blob(mask)
    assert area == 10
    assert cx == pytest.approx(9.5)
    assert largest_blob(np.zeros((5, 5), dtype=bool)) == (0, None)


def test_open_close_removes_specks_and_keeps_bars():
    mask = np.zeros((30, 30), dtype=bool)
    mask[5, 5] = True            # speck
    mask[10:20, 2:28] = True     # bar
    out = open_close(mask, 5)
    assert not out[5, 5]
    assert out[10:20, 2:28].all()
    assert erode(mask, 1) is mask and dilate(mask, 1) is mask


def test_image_encodings():
    bgr = stripe(canvas(), BLUE, 20, 120)
    rgb = bgr[..., ::-1].copy()
    assert np.array_equal(image_to_bgr(rgb.tobytes(), H, W, 'rgb8'), bgr)
    assert np.array_equal(image_to_bgr(bgr.tobytes(), H, W, 'bgr8'), bgr)
    rgba = np.concatenate([rgb, np.full((H, W, 1), 255, np.uint8)], axis=2)
    assert np.array_equal(image_to_bgr(rgba.tobytes(), H, W, 'rgba8'), bgr)
    padded = np.zeros((H, W * 3 + 8), dtype=np.uint8)  # row padding (step > width * 3)
    padded[:, :W * 3] = bgr.reshape(H, W * 3)
    assert np.array_equal(image_to_bgr(padded.tobytes(), H, W, 'bgr8', W * 3 + 8), bgr)
    assert image_to_bgr(b'', H, W, 'yuv422') is None
    assert image_to_bgr(b'123', H, W, 'bgr8') is None


def test_defaults_match_the_simulation():
    assert DEFAULTS['orange_lower'] == (8, 150, 150)
    assert DEFAULTS['orange_upper'] == (30, 255, 255)
    assert DEFAULTS['blue_lower'] == (100, 150, 50)
    assert DEFAULTS['blue_upper'] == (130, 255, 255)
    assert DEFAULTS['area_threshold'] == 200
    assert DEFAULTS['trigger_area'] == 1000
    assert DEFAULTS['roi_top_ratio'] == 0.0


# ---- cross-checks against OpenCV, only where OpenCV is installed ----------


def test_hsv_matches_opencv():
    cv2 = pytest.importorskip('cv2')
    rng = np.random.default_rng(0)
    img = rng.integers(0, 256, size=(64, 64, 3), dtype=np.uint8)
    ours = bgr_to_hsv(img).astype(int)
    ref = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(int)
    dh = np.abs(ours[..., 0] - ref[..., 0])
    dh = np.minimum(dh, 180 - dh)  # hue wraps
    assert dh.max() <= 1
    assert np.abs(ours[..., 1:] - ref[..., 1:]).max() <= 1


def test_morphology_matches_opencv():
    cv2 = pytest.importorskip('cv2')
    rng = np.random.default_rng(1)
    mask = rng.random((60, 80)) > 0.6
    kernel = np.ones((5, 5), np.uint8)
    m8 = mask.astype(np.uint8) * 255
    ref = cv2.morphologyEx(cv2.morphologyEx(m8, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
    assert np.array_equal(open_close(mask, 5), ref > 0)


def test_detection_close_to_simulation_algorithm():
    cv2 = pytest.importorskip('cv2')
    img = stripe(canvas(), ORANGE, 30, 130)
    stripe(img, BLUE, 200, 300, 100, 140)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    det = detect_lines(img)
    for ours, lo, hi in ((det.orange, (8, 150, 150), (30, 255, 255)),
                         (det.blue, (100, 150, 50), (130, 255, 255))):
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest = max(contours, key=cv2.contourArea)
        m = cv2.moments(largest)
        assert ours.detected
        assert abs(ours.cx - int(m['m10'] / m['m00'])) <= 1
        assert ours.area == pytest.approx(cv2.contourArea(largest), rel=0.1)
