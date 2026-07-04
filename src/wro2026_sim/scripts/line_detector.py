#!/usr/bin/env python3
"""
line_detector.py - WRO 2026 Step 5
發布 /line_detection (Float32MultiArray):
  [0] orange_detected  1=有 0=無
  [1] orange_cx        重心x像素 無則-1
  [2] orange_area      面積
  [3] orange_trigger   1=面積超過觸發門檻
  [4] blue_detected    1=有 0=無
  [5] blue_cx          重心x像素 無則-1
  [6] blue_area        面積
  [7] blue_trigger     1=面積超過觸發門檻
  [8] direction        1=順時針 -1=逆時針 0=未知
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

ORANGE_LOWER   = np.array([8,  150, 150])
ORANGE_UPPER   = np.array([30, 255, 255])
BLUE_LOWER     = np.array([100, 150,  50])
BLUE_UPPER     = np.array([130, 255, 255])
AREA_THRESHOLD = 200
TRIGGER_AREA   = 1000
ROI_TOP_RATIO  = 0.0

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.declare_parameter('image_topic',   '/camera/image_raw')
        self.declare_parameter('debug',          False)
        self.declare_parameter('area_threshold', AREA_THRESHOLD)
        self.declare_parameter('trigger_area',   TRIGGER_AREA)
        self.declare_parameter('roi_top_ratio',  ROI_TOP_RATIO)
        image_topic      = self.get_parameter('image_topic').value
        self.debug       = self.get_parameter('debug').value
        self.area_thr    = self.get_parameter('area_threshold').value
        self.trigger_thr = self.get_parameter('trigger_area').value
        self.roi_top     = self.get_parameter('roi_top_ratio').value
        self.bridge      = CvBridge() if CV_BRIDGE_AVAILABLE else None
        self._log_cnt    = 0
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub_detect = self.create_publisher(Float32MultiArray, '/line_detection', 10)
        if self.debug:
            self.pub_debug = self.create_publisher(Image, '/line_detection/debug_image', 10)
        self.get_logger().info(
            f'line_detector 啟動 | area_thr={self.area_thr} trigger={self.trigger_thr}')

    def decode_image(self, msg):
        if CV_BRIDGE_AVAILABLE:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return None

    def detect_color(self, hsv, lower, upper):
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, -1, 0.0, mask
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.area_thr:
            return False, -1, area, mask
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return False, -1, area, mask
        cx = int(M['m10'] / M['m00'])
        return True, cx, area, mask

    def judge_direction(self, blue_detected, blue_cx, img_width):
        if not blue_detected:
            return 0.0
        return 1.0 if blue_cx < img_width / 2.0 else -1.0

    def draw_debug(self, frame, roi_y,
                   orange_det, orange_cx, orange_area, orange_trigger, orange_mask,
                   blue_det,   blue_cx,   blue_area,   blue_trigger,   blue_mask,
                   direction):
        debug = frame.copy()
        cv2.line(debug, (0, roi_y), (frame.shape[1], roi_y), (255, 255, 0), 2)
        o_ov = np.zeros_like(frame)
        o_ov[roi_y:][orange_mask > 0] = (0, 100, 255)
        debug = cv2.addWeighted(debug, 1.0, o_ov, 0.5, 0)
        b_ov = np.zeros_like(frame)
        b_ov[roi_y:][blue_mask > 0] = (255, 100, 0)
        debug = cv2.addWeighted(debug, 1.0, b_ov, 0.5, 0)
        if orange_det:
            color = (0, 0, 255) if orange_trigger else (0, 165, 255)
            cv2.circle(debug, (orange_cx, roi_y + 20), 8, color, -1)
            label = f'O {orange_area:.0f}{"  TRIGGER!" if orange_trigger else ""}'
            cv2.putText(debug, label, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        if blue_det:
            color = (255, 0, 0) if blue_trigger else (255, 100, 0)
            cv2.circle(debug, (blue_cx, roi_y + 20), 8, color, -1)
            label = f'B {blue_area:.0f}{"  TRIGGER!" if blue_trigger else ""}'
            cv2.putText(debug, label, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        dir_str   = {1.0: 'CW', -1.0: 'CCW', 0.0: '?'}
        dir_color = {1.0: (0,255,0), -1.0: (0,0,255), 0.0: (128,128,128)}
        cv2.putText(debug, dir_str.get(direction, '?'), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, dir_color.get(direction, (255,255,255)), 2)
        return debug

    def image_cb(self, msg):
        frame = self.decode_image(msg)
        if frame is None:
            return
        h, w = frame.shape[:2]
        roi_y = int(h * self.roi_top)
        roi   = frame[roi_y:, :]
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        orange_det, orange_cx, orange_area, orange_mask = \
            self.detect_color(hsv, ORANGE_LOWER, ORANGE_UPPER)
        blue_det, blue_cx, blue_area, blue_mask = \
            self.detect_color(hsv, BLUE_LOWER, BLUE_UPPER)
        orange_trigger = orange_det and orange_area >= self.trigger_thr
        blue_trigger   = blue_det   and blue_area   >= self.trigger_thr
        direction = self.judge_direction(blue_det, blue_cx, w)
        result = Float32MultiArray()
        result.data = [
            float(orange_det), float(orange_cx), float(orange_area),
            float(orange_trigger),
            float(blue_det),   float(blue_cx),   float(blue_area),
            float(blue_trigger),
            direction,
        ]
        self.pub_detect.publish(result)
        if self.debug and CV_BRIDGE_AVAILABLE:
            dbg = self.draw_debug(
                frame, roi_y,
                orange_det, orange_cx, orange_area, orange_trigger, orange_mask,
                blue_det,   blue_cx,   blue_area,   blue_trigger,   blue_mask,
                direction)
            dm = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            dm.header = msg.header
            self.pub_debug.publish(dm)
        self._log_cnt += 1
        if self._log_cnt % 20 == 0:
            self.get_logger().info(
                f'O={orange_det}({orange_area:.0f})'
                f'{"TRIG" if orange_trigger else "    "} | '
                f'B={blue_det}({blue_area:.0f})'
                f'{"TRIG" if blue_trigger else "    "} | '
                f'dir={direction}')

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
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
