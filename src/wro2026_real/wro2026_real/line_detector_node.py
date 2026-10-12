"""
Parameterised orange / blue line detector node for the real car.

Subscribes an image topic (parameter image_topic) and publishes
/line_detection (std_msgs/Float32MultiArray) in the same 9-value layout as
the simulation's line_detector.py; see line_detection.py for the layout.
That layout is fragile (meaning by position only) and should become a
custom message later.

Image conversion is done with numpy for rgb8 / bgr8 / rgba8 / bgra8. Other
encodings (yuv422, mono8, compressed) need cv_bridge, which is not
installed on the Jetson; such images are skipped with a warning.

This node only observes. It never publishes /cmd_vel.
"""
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from .line_detection import DEFAULTS, detect_lines, image_to_bgr, to_array

HSV_KEYS = ('orange_lower', 'orange_upper', 'blue_lower', 'blue_upper')


class LineDetector(Node):
    """ROS wiring around line_detection.detect_lines."""

    def __init__(self):
        super().__init__('line_detector')
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.output_topic = self.declare_parameter('output_topic', '/line_detection').value
        self.debug = self.declare_parameter('debug', False).value
        self.params = {}
        for key, default in DEFAULTS.items():
            if key in HSV_KEYS:
                value = self.declare_parameter(key, list(default)).value
                if len(value) != 3:
                    raise ValueError(f'{key} must have 3 values (H, S, V)')
                self.params[key] = tuple(int(v) for v in value)
            else:
                self.params[key] = self.declare_parameter(key, default).value
        self._log_cnt = 0
        self.sub = self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.pub = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        self.pub_debug = (self.create_publisher(Image, self.output_topic + '/debug_image', 10)
                          if self.debug else None)
        self.get_logger().info(
            f'line_detector: {self.image_topic} -> {self.output_topic} | '
            f"area_thr={self.params['area_threshold']} trigger={self.params['trigger_area']} "
            f"roi top/bottom/left/right={self.params['roi_top_ratio']}/"
            f"{self.params['roi_bottom_ratio']}/{self.params['roi_left_ratio']}/"
            f"{self.params['roi_right_ratio']}")

    def _on_image(self, msg):
        frame = image_to_bgr(msg.data, msg.height, msg.width, msg.encoding, msg.step)
        if frame is None:
            self.get_logger().warn(f'unsupported image encoding {msg.encoding!r} '
                                   '(needs cv_bridge)', throttle_duration_sec=10.0)
            return
        det = detect_lines(frame, self.params)
        self.pub.publish(Float32MultiArray(data=to_array(det)))
        if self.pub_debug is not None:
            self.pub_debug.publish(self._debug_image(frame, det, msg.header))
        self._log_cnt += 1
        if self._log_cnt % 20 == 0:
            o, b = det.orange, det.blue
            self.get_logger().info(
                f'O={o.detected}({o.area:.0f}){"TRIG" if o.trigger else "    "} | '
                f'B={b.detected}({b.area:.0f}){"TRIG" if b.trigger else "    "} | '
                f'dir={det.direction}')

    def _debug_image(self, frame, det, header):
        top, bottom, left, right = det.roi
        out = frame[..., ::-1].copy()  # RGB
        roi = out[top:bottom, left:right]
        roi[det.orange.mask] = (255, 100, 0)
        roi[det.blue.mask] = (0, 100, 255)
        if top < out.shape[0]:
            out[top, :] = (255, 255, 0)
        img = Image()
        img.header = header
        img.height, img.width = out.shape[:2]
        img.encoding = 'rgb8'
        img.step = img.width * 3
        img.data = np.ascontiguousarray(out).tobytes()
        return img


def main():
    """Spin the line detector."""
    rclpy.init()
    node = LineDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
