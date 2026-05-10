#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarTrackNode(Node):
    def __init__(self):
        super().__init__("lidar_track_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("field_of_view_deg", 200.0)
        self.declare_parameter("bubble_radius", 12)
        self.declare_parameter("best_point_window", 10)
        self.declare_parameter("steering_gain", 1.0)
        self.declare_parameter("max_steering", 0.34)
        self.declare_parameter("min_speed", 0.6)
        self.declare_parameter("max_speed", 1.5)
        self.declare_parameter("straights_speed", 2.0)
        self.declare_parameter("front_slowdown_distance", 2.0)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.max_range = float(self.get_parameter("max_range").value)
        self.field_of_view = math.radians(float(self.get_parameter("field_of_view_deg").value))
        self.bubble_radius = int(self.get_parameter("bubble_radius").value)
        self.best_point_window = int(self.get_parameter("best_point_window").value)
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.max_steering = float(self.get_parameter("max_steering").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.straights_speed = float(self.get_parameter("straights_speed").value)
        self.front_slowdown_distance = float(self.get_parameter("front_slowdown_distance").value)
        self.scan_counter = 0

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info(f"Listening on {self.scan_topic}, publishing {self.drive_topic}")

    def scan_callback(self, scan_msg: LaserScan):
        self.scan_counter += 1
        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        center_index = len(ranges) // 2
        half_window = int((self.field_of_view / scan_msg.angle_increment) / 2.0)
        start = max(0, center_index - half_window)
        end = min(len(ranges), center_index + half_window)
        proc_ranges = ranges[start:end].copy()
        if proc_ranges.size == 0:
            self.publish_drive(0.0, self.min_speed)
            return

        closest_idx = int(np.argmin(np.where(proc_ranges > 0.0, proc_ranges, np.inf)))
        bubble_start = max(0, closest_idx - self.bubble_radius)
        bubble_end = min(proc_ranges.size, closest_idx + self.bubble_radius + 1)
        proc_ranges[bubble_start:bubble_end] = 0.0

        best_idx = self.find_best_point(proc_ranges)
        if best_idx is None:
            if self.scan_counter % 50 == 0:
                self.get_logger().info("No valid gap found, publishing fallback command")
            self.publish_drive(0.0, self.min_speed)
            return

        global_idx = start + best_idx
        steering_angle = scan_msg.angle_min + global_idx * scan_msg.angle_increment
        steering_angle = float(np.clip(self.steering_gain * steering_angle, -self.max_steering, self.max_steering))

        front_distance = float(np.max(ranges[max(0, center_index - 8):min(len(ranges), center_index + 9)]))
        speed = self.compute_speed(steering_angle, front_distance)

        self.publish_drive(steering_angle, speed)

    def find_best_point(self, proc_ranges):
        nonzero = proc_ranges > 0.0
        if not np.any(nonzero):
            return None

        best_start = 0
        best_end = 0
        current_start = None
        for idx, valid in enumerate(nonzero):
            if valid and current_start is None:
                current_start = idx
            elif not valid and current_start is not None:
                if idx - current_start > best_end - best_start:
                    best_start, best_end = current_start, idx
                current_start = None
        if current_start is not None and len(nonzero) - current_start > best_end - best_start:
            best_start, best_end = current_start, len(nonzero)

        if best_end <= best_start:
            return None

        gap = proc_ranges[best_start:best_end]
        if gap.size == 0:
            return None

        peak = int(np.argmax(gap))
        left = max(0, peak - self.best_point_window)
        right = min(gap.size, peak + self.best_point_window + 1)
        averaged = np.mean(gap[left:right])
        candidates = np.where(gap[left:right] >= averaged)[0]
        chosen = peak if candidates.size == 0 else left + int(candidates[len(candidates) // 2])
        return best_start + chosen

    def compute_speed(self, steering_angle, front_distance):
        steering_ratio = min(abs(steering_angle) / self.max_steering, 1.0)
        corner_speed = self.straights_speed - steering_ratio * (self.straights_speed - self.min_speed)
        if front_distance < self.front_slowdown_distance:
            slowdown = max(front_distance / self.front_slowdown_distance, 0.35)
            corner_speed *= slowdown
        return float(np.clip(corner_speed, self.min_speed, self.max_speed))

    def publish_drive(self, steering_angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTrackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
