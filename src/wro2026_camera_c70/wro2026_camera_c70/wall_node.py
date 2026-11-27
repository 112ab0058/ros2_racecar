"""
C70 wall node: camera -> camera/wall_scan (sensor_msgs/LaserScan). READ ONLY.

Only reads the camera (V4L2, MJPG 1280x720 like the calibration); never touches the
motors or any serial port. A reader thread keeps taking every frame, so the frame
used is always the newest one (OpenCV would otherwise hand out old buffered frames).
`rate` times a second the newest frame goes through wall.py: wall bottoms -> floor points
-> one range per beam from the lidar.

lidar_fix (2026-11-27): the only subscription is /scan, read only. Per frame the pitch is
corrected so that the wall points 0.5-1.0 m away agree with the lidar's ranges in the same
beams (wall.lidar_pitch_fix); the camera sits ~10 cm above the floor, so a tilted car or
a wall standing a few mm higher than the floor under the car moves its points by cm.

camera/wall_scan has the same beams as /scan (frame laser_link, angle_min 0, bins beams),
stamped with the time the frame was read. It is not a lidar: each range is where the
camera saw a wall bottom in that direction, inf where it saw none. scan_memory_node puts
these points into its memory, which fills the lidar's blind beams with them.

image_rate > 0 also publishes the frames used (JPEG, same stamp) on
camera/image/compressed, at most image_rate a second, so a bag can be replayed through
wall.py later (orin_auto.sh records it).
"""
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan

from .wall import FloorCamera, lidar_pitch_fix, to_ranges, wall_bottom

SCAN_MAX_AGE = 0.15     # s; lidar_fix uses the newest /scan only if it is this fresh

# name, default, description. Defaults must match config/c70_wall.yaml.
PARAMETERS = [
    ('device', '/dev/video0', 'camera device (V4L2)'),
    ('intrinsics', '', 'camera_info yaml; empty = config/c70_ost_1280x720.yaml of this package'),
    ('extrinsics', '', 'extrinsics yaml; empty = config/c70_extrinsics.yaml of this package'),
    ('frame_id', 'laser_link', 'frame of camera/wall_scan (ranges are seen from the lidar)'),
    ('laser_x', 0.13, 'm; lidar in front of base_link (urdf laser_link)'),
    ('bins', 480, 'beams per scan, as /scan'),
    ('rate', 10.0, 'wall scans per second (the camera gives about 28 frames per second)'),
    ('threshold', 110, 'grey level 0-255: darker is wall (2026-11-13 field photos: walls '
                       '14-89, floor just below them 130-254, 5-95 %)'),
    ('run', 12, 'dark pixels in a row (upwards) that make a wall'),
    ('max_saturation', 60, 'HSV saturation 0-255: the floor below a wall bottom must be '
                           'whiter than this (beige tape on the walls: 93-103, floor: 9-27)'),
    ('step', 4, 'use every step-th image column'),
    ('max_range', 0.5, 'm from the lidar; farther wall points are dropped (the lidar sees '
                       'there; the camera error grows with the distance squared)'),
    ('lidar_fix', True, 'per frame, correct the pitch from the wall points 0.5-1.0 m away '
                        'against /scan (wall.lidar_pitch_fix); false = calibration only'),
    ('fix_min_pairs', 8, 'lidar_fix: at least this many far points the lidar also sees'),
    ('fix_search_deg', 3.0, 'lidar_fix: largest pitch correction tried (deg)'),
    ('frame_timeout', 1.0, '/diagnostics ERROR when no camera frame for this long (s)'),
    ('image_rate', 0.0, 'frames per second also published as JPEG on camera/image/compressed '
                        '(for bags); 0 = none'),
    ('jpeg_quality', 80, 'JPEG quality of camera/image/compressed'),
]


class WallNode(Node):
    """ROS wiring around wall.py."""

    def __init__(self):
        super().__init__('c70_wall')
        self.p = {n: self.declare_parameter(n, d).value for n, d, _ in PARAMETERS}
        share = get_package_share_directory('wro2026_camera_c70')
        intr = self.p['intrinsics'] or os.path.join(share, 'config', 'c70_ost_1280x720.yaml')
        extr = self.p['extrinsics'] or os.path.join(share, 'config', 'c70_extrinsics.yaml')
        self.cam = FloorCamera.from_files(intr, extr)
        w, h = self.cam.size
        self.cap = cv2.VideoCapture(self.p['device'], cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not self.cap.isOpened():
            raise RuntimeError('cannot open the camera %s' % self.p['device'])
        got = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
               int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got != (w, h):
            raise RuntimeError('camera gives %dx%d, the calibration is for %dx%d' % (got + (w, h)))
        self.pub = self.create_publisher(LaserScan, 'camera/wall_scan', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.image_pub = (self.create_publisher(CompressedImage, 'camera/image/compressed', 5)
                          if self.p['image_rate'] > 0 else None)
        self.last_image = None       # monotonic time of the last image published
        self.scan = None             # (monotonic time received, ranges) of the newest /scan
        self.fix_deg = 0.0           # newest pitch correction (deg)
        self.fixed = self.processed = 0
        if self.p['lidar_fix']:
            # read only: the lidar's ranges, to correct the camera's pitch (wall.lidar_pitch_fix)
            self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.frame = None            # (image, ROS stamp, monotonic time)
        self.used = None             # monotonic time of the last frame processed
        self.frames = 0
        self.points = 0
        self.proc_ms = 0.0
        self.read_error = None
        self.running = True
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()
        self.create_timer(1.0 / self.p['rate'], self._process)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info('%s %dx%d -> camera/wall_scan (READ ONLY)'
                               % (self.p['device'], w, h))

    def _read(self):
        while self.running:
            ok, img = self.cap.read()
            if not ok:
                self.read_error = 'camera read failed'
                time.sleep(0.1)
                continue
            self.read_error = None
            with self.lock:
                self.frame = (img, self.get_clock().now().to_msg(), time.monotonic())
                self.frames += 1

    def _process(self):
        with self.lock:
            frame = self.frame
        if frame is None or frame[2] == self.used:
            return
        img, stamp, t_read = frame
        self.used = t_read
        t0 = time.monotonic()
        px = wall_bottom(img, self.p['threshold'], self.p['run'], self.p['step'],
                         self.p['max_saturation'])
        change = 0.0
        scan = self.scan
        if scan is not None and abs(t_read - scan[0]) <= SCAN_MAX_AGE:
            change, _ = lidar_pitch_fix(self.cam, px, scan[1], self.p['laser_x'],
                                        self.p['bins'], min_pairs=self.p['fix_min_pairs'],
                                        search_deg=self.p['fix_search_deg'])
        self.processed += 1
        if change:
            self.fixed += 1
            self.fix_deg = change
        pts = self.cam.floor_points(px, change)
        ranges = to_ranges(pts, self.p['laser_x'], self.p['bins'], self.p['max_range'])
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.p['frame_id']
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / self.p['bins']
        msg.angle_max = msg.angle_increment * (self.p['bins'] - 1)
        msg.range_min = 0.0
        msg.range_max = float(self.p['max_range'])
        msg.ranges = [float(r) for r in ranges]
        self.pub.publish(msg)
        self.points = len(pts)
        self.proc_ms = 1000.0 * (time.monotonic() - t0)
        if self.image_pub and (self.last_image is None
                               or t_read - self.last_image >= 1.0 / self.p['image_rate']):
            self.last_image = t_read
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY,
                                                 int(self.p['jpeg_quality'])])
            if ok:
                im = CompressedImage(format='jpeg', data=jpg.tobytes())
                im.header.stamp = stamp
                im.header.frame_id = 'c70'
                self.image_pub.publish(im)

    def _on_scan(self, msg):
        if len(msg.ranges) == self.p['bins']:
            self.scan = (time.monotonic(), msg.ranges)

    def _publish_diagnostics(self):
        fixed, processed = self.fixed, self.processed
        self.fixed = self.processed = 0
        with self.lock:
            last = self.frame[2] if self.frame else None
            frames, self.frames = self.frames, 0
        age = None if last is None else time.monotonic() - last
        st = DiagnosticStatus(name='c70_wall', hardware_id=self.p['device'])
        if self.read_error or age is None or age > self.p['frame_timeout']:
            st.level = DiagnosticStatus.ERROR
            st.message = self.read_error or 'no camera frame'
        else:
            st.level = DiagnosticStatus.OK
            st.message = 'ok'
        st.values = [KeyValue(key='frames_per_s', value=str(frames)),
                     KeyValue(key='wall_points', value=str(self.points)),
                     KeyValue(key='process_ms', value='%.1f' % self.proc_ms),
                     KeyValue(key='pitch_fix_deg', value='%+.2f' % self.fix_deg),
                     KeyValue(key='pitch_fixed_frames', value='%d/%d' % (fixed, processed)),
                     KeyValue(key='frame_age_s', value='-' if age is None else '%.2f' % age)]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [st]
        self.diag_pub.publish(msg)

    def destroy_node(self):
        """Stop the reader thread and release the camera."""
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main():
    """Entry point."""
    rclpy.init()
    node = WallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
