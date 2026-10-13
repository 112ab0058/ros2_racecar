"""
Orange / blue line detection on one image (numpy only, no OpenCV, no rclpy).

Re-implementation of src/wro2026_sim/scripts/line_detector.py with every
threshold as a parameter. The result keeps the simulation's
/line_detection layout (Float32MultiArray, 9 values):

  [0] orange_detected  1 = yes, 0 = no
  [1] orange_cx        centroid x in full-image pixels, -1 when not detected
  [2] orange_area      area of the largest orange blob (pixels)
  [3] orange_trigger   1 = area >= trigger_area
  [4] blue_detected
  [5] blue_cx
  [6] blue_area
  [7] blue_trigger
  [8] direction        1 = clockwise (blue left of centre), -1 = counter-
                       clockwise, 0 = unknown (no blue line)

Differences from the simulation version (OpenCV):
  * area is the pixel count of the largest 8-connected blob; OpenCV's
    contourArea of the outer contour is slightly smaller ((w-1)*(h-1) for a
    w x h rectangle).
  * cx is the pixel centroid, not the contour-polygon centroid.
  * the ROI can also be cropped at the bottom / left / right; cx is still
    reported in full-image coordinates.
"""
from collections import namedtuple

import numpy as np

DEFAULTS = {
    # OpenCV 8-bit HSV: H 0..179, S 0..255, V 0..255 (same as the simulation)
    'orange_lower': (8, 150, 150),
    'orange_upper': (30, 255, 255),
    'blue_lower': (100, 150, 50),
    'blue_upper': (130, 255, 255),
    'area_threshold': 200,
    'trigger_area': 1000,
    'roi_top_ratio': 0.0,
    'roi_bottom_ratio': 1.0,
    'roi_left_ratio': 0.0,
    'roi_right_ratio': 1.0,
    'morph_kernel': 5,
}

ColorResult = namedtuple('ColorResult', 'detected cx area trigger mask')
LineDetection = namedtuple('LineDetection', 'orange blue direction roi')


def bgr_to_hsv(image):
    """
    Convert a uint8 BGR image to OpenCV-style 8-bit HSV.

    H = hue / 2 (0..179), S and V 0..255, rounded like cv2.cvtColor.
    """
    img = image.astype(np.float32)
    b, g, r = img[..., 0], img[..., 1], img[..., 2]
    v = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    diff = v - mn
    with np.errstate(divide='ignore', invalid='ignore'):
        s = np.where(v > 0, diff * 255.0 / v, 0.0)
        safe = np.where(diff > 0, diff, 1.0)
        h = np.where(v == r, 60.0 * (g - b) / safe,
                     np.where(v == g, 120.0 + 60.0 * (b - r) / safe,
                              240.0 + 60.0 * (r - g) / safe))
    h = np.where(diff > 0, h, 0.0)
    h = np.where(h < 0, h + 360.0, h)
    hsv = np.empty(image.shape, dtype=np.uint8)
    hsv[..., 0] = np.clip(np.round(h / 2.0), 0, 180).astype(np.uint8) % 180
    hsv[..., 1] = np.clip(np.round(s), 0, 255).astype(np.uint8)
    hsv[..., 2] = v.astype(np.uint8)
    return hsv


def in_range(hsv, lower, upper):
    """Return a bool mask of pixels with lower <= hsv <= upper on every channel."""
    lower = np.asarray(lower, dtype=np.int16)
    upper = np.asarray(upper, dtype=np.int16)
    px = hsv.astype(np.int16)
    return np.all((px >= lower) & (px <= upper), axis=-1)


def _shift_reduce(mask, k, axis, op, pad_value):
    """Apply op (logical and/or) over a window of k pixels along axis."""
    if k <= 1:
        return mask
    before = (k - 1) // 2
    after = k - 1 - before
    pad = [(0, 0), (0, 0)]
    pad[axis] = (before, after)
    padded = np.pad(mask, pad, constant_values=pad_value)
    n = mask.shape[axis]
    out = None
    for i in range(k):
        window = padded[i:i + n, :] if axis == 0 else padded[:, i:i + n]
        out = window.copy() if out is None else op(out, window)
    return out


def erode(mask, k):
    """Erode with a k x k square; pixels outside the image do not erode."""
    m = _shift_reduce(mask, k, 1, np.logical_and, True)
    return _shift_reduce(m, k, 0, np.logical_and, True)


def dilate(mask, k):
    """Dilate with a k x k square."""
    m = _shift_reduce(mask, k, 1, np.logical_or, False)
    return _shift_reduce(m, k, 0, np.logical_or, False)


def open_close(mask, k):
    """Morphological opening then closing (removes specks, fills small holes)."""
    if k <= 1:
        return mask
    opened = dilate(erode(mask, k), k)
    return erode(dilate(opened, k), k)


def _row_runs(row):
    padded = np.concatenate(([False], row, [False])).astype(np.int8)
    d = np.diff(padded)
    return np.flatnonzero(d == 1), np.flatnonzero(d == -1)


def largest_blob(mask):
    """
    Return (area, cx) of the largest 8-connected blob, or (0, None).

    Run-length labelling: each row is split into runs of True pixels, runs
    that touch a run of the previous row (diagonals included) are merged
    with union-find. Fast for the few runs a line produces.
    """
    parent = []

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    runs = []  # (start, end) per run id, end exclusive
    prev = []
    for y in range(mask.shape[0]):
        starts, ends = _row_runs(mask[y])
        cur = []
        for a, b in zip(starts.tolist(), ends.tolist()):
            rid = len(parent)
            parent.append(rid)
            runs.append((a, b))
            for pa, pb, pid in prev:
                if pa <= b and a <= pb:  # [a, b) and [pa, pb) touch (8-connected)
                    ra, rb = find(rid), find(pid)
                    if ra != rb:
                        parent[ra] = rb
            cur.append((a, b, rid))
        prev = cur
    if not runs:
        return 0, None
    area = {}
    sum_x = {}
    for rid, (a, b) in enumerate(runs):
        root = find(rid)
        n = b - a
        area[root] = area.get(root, 0) + n
        sum_x[root] = sum_x.get(root, 0.0) + (a + b - 1) * n / 2.0
    best = max(area, key=lambda r: area[r])
    return area[best], sum_x[best] / area[best]


def roi_bounds(shape, params):
    """Return (top, bottom, left, right) pixel bounds of the ROI."""
    h, w = shape[:2]
    top = int(h * params['roi_top_ratio'])
    bottom = int(round(h * params['roi_bottom_ratio']))
    left = int(w * params['roi_left_ratio'])
    right = int(round(w * params['roi_right_ratio']))
    top, bottom = max(0, min(top, h)), max(0, min(bottom, h))
    left, right = max(0, min(left, w)), max(0, min(right, w))
    return top, bottom, left, right


def detect_color(hsv, lower, upper, params, x_offset=0):
    """Detect one color in an HSV ROI; cx is shifted by x_offset into image coordinates."""
    mask = open_close(in_range(hsv, lower, upper), int(params['morph_kernel']))
    area, cx = largest_blob(mask)
    if area == 0:
        return ColorResult(False, -1, 0.0, False, mask)
    if area < params['area_threshold']:
        return ColorResult(False, -1, float(area), False, mask)
    detected_cx = int(cx) + x_offset
    return ColorResult(True, detected_cx, float(area), area >= params['trigger_area'], mask)


def judge_direction(blue, image_width):
    """Return 1.0 when the blue line is left of centre, -1.0 when right, 0.0 when absent."""
    if not blue.detected:
        return 0.0
    return 1.0 if blue.cx < image_width / 2.0 else -1.0


def merge_params(params=None):
    """Return DEFAULTS updated with params."""
    out = dict(DEFAULTS)
    if params:
        out.update(params)
    return out


def detect_lines(image_bgr, params=None):
    """Run the full detection on a uint8 BGR image."""
    p = merge_params(params)
    top, bottom, left, right = roi_bounds(image_bgr.shape, p)
    roi = image_bgr[top:bottom, left:right]
    if roi.size == 0:
        empty = np.zeros(roi.shape[:2], dtype=bool)
        none = ColorResult(False, -1, 0.0, False, empty)
        return LineDetection(none, none, 0.0, (top, bottom, left, right))
    hsv = bgr_to_hsv(roi)
    orange = detect_color(hsv, p['orange_lower'], p['orange_upper'], p, left)
    blue = detect_color(hsv, p['blue_lower'], p['blue_upper'], p, left)
    direction = judge_direction(blue, image_bgr.shape[1])
    return LineDetection(orange, blue, direction, (top, bottom, left, right))


def to_array(det):
    """Return the 9-float /line_detection layout (same as the simulation)."""
    o, b = det.orange, det.blue
    return [
        float(o.detected), float(o.cx), float(o.area), float(o.trigger),
        float(b.detected), float(b.cx), float(b.area), float(b.trigger),
        float(det.direction),
    ]


def image_to_bgr(data, height, width, encoding, step=None):
    """
    Convert raw sensor_msgs/Image bytes to a uint8 BGR array, or return None.

    Supports rgb8, bgr8, rgba8 and bgra8 without cv_bridge / OpenCV. Other
    encodings (e.g. yuv422, mono8, compressed images) need cv_bridge.
    """
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(encoding)
    if channels is None:
        return None
    step = step or width * channels
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size < step * height:
        return None
    rows = buf[:step * height].reshape(height, step)[:, :width * channels]
    img = rows.reshape(height, width, channels)
    if encoding in ('rgb8', 'rgba8'):
        return np.ascontiguousarray(img[..., 2::-1])
    return np.ascontiguousarray(img[..., :3])
