#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class CorridorTrackNode(Node):
    def __init__(self):
        super().__init__("corridor_track_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("left_angle_deg", 55.0)
        self.declare_parameter("right_angle_deg", -55.0)
        self.declare_parameter("front_angle_deg", 0.0)
        self.declare_parameter("sample_half_width_deg", 8.0)
        self.declare_parameter("wall_balance_gain", 0.9)
        self.declare_parameter("forward_gain", 0.35)
        self.declare_parameter("max_steering", 0.34)
        self.declare_parameter("min_speed", 0.35)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("front_slowdown_distance", 2.5)
        self.declare_parameter("base_speed", 0.8)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.max_range = float(self.get_parameter("max_range").value)
        self.left_angle = math.radians(float(self.get_parameter("left_angle_deg").value))
        self.right_angle = math.radians(float(self.get_parameter("right_angle_deg").value))
        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.sample_half_width = math.radians(float(self.get_parameter("sample_half_width_deg").value))
        self.wall_balance_gain = float(self.get_parameter("wall_balance_gain").value)
        self.forward_gain = float(self.get_parameter("forward_gain").value)
        self.max_steering = float(self.get_parameter("max_steering").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.front_slowdown_distance = float(self.get_parameter("front_slowdown_distance").value)
        self.base_speed = float(self.get_parameter("base_speed").value)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.get_logger().info(f"Listening on {self.scan_topic}, publishing {self.drive_topic}")

    def scan_callback(self, scan_msg: LaserScan):
        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        left_dist = self.sample_sector(ranges, scan_msg, self.left_angle)
        right_dist = self.sample_sector(ranges, scan_msg, self.right_angle)
        front_dist = self.sample_sector(ranges, scan_msg, self.front_angle)

        wall_error = right_dist - left_dist
        front_bias = self.compute_front_bias(ranges, scan_msg)

        steering_angle = self.wall_balance_gain * wall_error + self.forward_gain * front_bias
        steering_angle = float(np.clip(steering_angle, -self.max_steering, self.max_steering))

        speed = self.base_speed
        if front_dist < self.front_slowdown_distance:
            speed *= max(front_dist / self.front_slowdown_distance, 0.35)
        speed *= max(0.4, 1.0 - abs(steering_angle) / max(self.max_steering, 1e-6))
        speed = float(np.clip(speed, self.min_speed, self.max_speed))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

    def sample_sector(self, ranges, scan_msg, center_angle):
        center_index = int(round((center_angle - scan_msg.angle_min) / scan_msg.angle_increment))
        half_window = max(1, int(round(self.sample_half_width / scan_msg.angle_increment)))
        start = max(0, center_index - half_window)
        end = min(len(ranges), center_index + half_window + 1)
        sector = ranges[start:end]
        valid = sector[sector > 0.05]
        if valid.size == 0:
            return self.max_range
        return float(np.percentile(valid, 60))

    def compute_front_bias(self, ranges, scan_msg):
        front_left = self.sample_sector(ranges, scan_msg, math.radians(20.0))
        front_right = self.sample_sector(ranges, scan_msg, math.radians(-20.0))
        return front_right - front_left


def main(args=None):
    rclpy.init(args=args)
    node = CorridorTrackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
