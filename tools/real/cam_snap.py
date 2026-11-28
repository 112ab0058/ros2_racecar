#!/usr/bin/env python3
"""
Orin: save a few C70 frames in the calibration's mode (READ ONLY: camera only, no motors).

    python3 ~/racecar_ws/tools/cam_snap.py LABEL [N]     # default 3 photos, 0.3 s apart

MJPG 1280x720 like cam_calib_capture.py, so docs/real/calib/c70_ost_1280x720.yaml and
c70_extrinsics.yaml apply. The first 15 frames are dropped (auto exposure settles).
Saves ~/r550_camfield/LABEL_k.jpg (k = 0..N-1). Run next to lidar_snap.py
(`python3 lidar_snap.py LABEL 3 & python3 cam_snap.py LABEL 3; wait`) to record the
camera and the raw lidar at the same moment; used on 2026-11-13 at the field (LOGBOOK).
"""
import os
import sys
import time

import cv2


def main():
    label = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    out = os.path.expanduser('~/r550_camfield')
    os.makedirs(out, exist_ok=True)
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        sys.exit('相機打不開：/dev/video0')
    for _ in range(15):
        cap.read()
    saved, img = 0, None
    for k in range(n):
        ok, frame = cap.read()
        if not ok:
            print('frame %d failed' % k)
            continue
        img = frame
        cv2.imwrite(os.path.join(out, '%s_%d.jpg' % (label, k)), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        saved += 1
        time.sleep(0.3)
    cap.release()
    if img is None:
        sys.exit('%s: no photo saved' % label)
    h, w = img.shape[:2]
    print('%s: %d photos %dx%d, mean brightness %.0f' % (label, saved, w, h, img.mean()))


if __name__ == '__main__':
    main()
