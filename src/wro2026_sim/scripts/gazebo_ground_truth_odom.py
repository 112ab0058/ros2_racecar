#!/usr/bin/env python3

import math
import threading

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node as RosNode
from tf2_ros import TransformBroadcaster

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboGroundTruthOdom(RosNode):
    def __init__(self):
        super().__init__("gazebo_ground_truth_odom")

        self.declare_parameter("gz_pose_topic", "/world/wro2026/dynamic_pose/info")
        self.declare_parameter("model_name", "turtlebot3_burger")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("child_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)

        self.gz_pose_topic = self.get_parameter("gz_pose_topic").value
        self.model_name = self.get_parameter("model_name").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.gz_node = GzNode()
        self.lock = threading.Lock()
        self.last_sample = None
        self.active = True

        ok = self.gz_node.subscribe(Pose_V, self.gz_pose_topic, self.on_gz_pose)
        if not ok:
            raise RuntimeError(f"Failed to subscribe to Gazebo topic {self.gz_pose_topic}")

        self.get_logger().info(
            f"Publishing /odom from Gazebo ground truth topic {self.gz_pose_topic}, "
            f"model={self.model_name}, child_frame={self.child_frame}"
        )

    def stamp_from_gz(self, msg):
        stamp = Time()
        stamp.sec = int(msg.header.stamp.sec)
        stamp.nanosec = int(msg.header.stamp.nsec)
        return stamp

    def on_gz_pose(self, msg):
        if not self.active or not rclpy.ok():
            return

        pose = next((p for p in msg.pose if p.name == self.model_name), None)
        if pose is None:
            return

        stamp = self.stamp_from_gz(msg)
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        with self.lock:
            if self.last_sample is not None:
                last_stamp, last_x, last_y, last_yaw = self.last_sample
                dt = (stamp.sec - last_stamp.sec) + (
                    stamp.nanosec - last_stamp.nanosec
                ) * 1e-9
                if dt > 1e-6:
                    dx_world = (x - last_x) / dt
                    dy_world = (y - last_y) / dt
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    linear_x = cos_yaw * dx_world + sin_yaw * dy_world
                    linear_y = -sin_yaw * dx_world + cos_yaw * dy_world
                    angular_z = normalize_angle(yaw - last_yaw) / dt

            self.last_sample = (stamp, x, y, yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z

        odom.pose.covariance[0] = 1e-6
        odom.pose.covariance[7] = 1e-6
        odom.pose.covariance[35] = 1e-6
        odom.twist.covariance[0] = 1e-5
        odom.twist.covariance[7] = 1e-5
        odom.twist.covariance[35] = 1e-5

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.child_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        self.active = False
        try:
            self.gz_node.unsubscribe(self.gz_pose_topic)
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = GazeboGroundTruthOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
