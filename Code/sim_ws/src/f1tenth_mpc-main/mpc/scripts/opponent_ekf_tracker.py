#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan


class OpponentEKFTracker(Node):
    def __init__(self):
        super().__init__('opp_ekf_tracker')

        self.declare_parameter('measurement_source', 'lidar')
        self.declare_parameter('measurement_topic', '/ego_racecar/opp_odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('output_topic', '/ego_racecar/opp_odom_ekf')
        self.declare_parameter('output_pose_topic', '/ego_racecar/opp_odom_ekf_pose')
        self.declare_parameter('process_noise_pos', 0.05)
        self.declare_parameter('process_noise_vel', 0.5)
        self.declare_parameter('measurement_noise_pos', 0.08)
        self.declare_parameter('measurement_noise_vel', 0.8)
        self.declare_parameter('lidar_fov_deg', 120.0)
        self.declare_parameter('lidar_min_range', 0.2)
        self.declare_parameter('lidar_max_range', 8.0)
        self.declare_parameter('cluster_gap', 0.35)
        self.declare_parameter('min_cluster_points', 9)
        self.declare_parameter('max_cluster_points', 100)
        self.declare_parameter('min_cluster_width', 0.05)
        self.declare_parameter('max_cluster_width', 0.75)
        self.declare_parameter('association_gate', 1.2)
        self.declare_parameter('reacquisition_gate', 1.5)
        self.declare_parameter('reacquisition_after_missed_scans', 3)

        measurement_source = self.get_parameter('measurement_source').value
        measurement_topic = self.get_parameter('measurement_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        ego_odom_topic = self.get_parameter('ego_odom_topic').value
        output_topic = self.get_parameter('output_topic').value
        output_pose_topic = self.get_parameter('output_pose_topic').value

        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, output_pose_topic, 10)
        if measurement_source == 'lidar':
            self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
            self.ego_odom_sub = self.create_subscription(Odometry, ego_odom_topic, self.ego_odom_callback, 10)
        else:
            self.sub = self.create_subscription(Odometry, measurement_topic, self.odom_callback, 10)

        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 0.5
        self.last_stamp = None
        self.initialized = False
        self.ego_odom = None
        self.last_lidar_detection = None
        self.lidar_fov = math.radians(float(self.get_parameter('lidar_fov_deg').value))
        self.lidar_min_range = float(self.get_parameter('lidar_min_range').value)
        self.lidar_max_range = float(self.get_parameter('lidar_max_range').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.max_cluster_points = int(self.get_parameter('max_cluster_points').value)
        self.min_cluster_width = float(self.get_parameter('min_cluster_width').value)
        self.max_cluster_width = float(self.get_parameter('max_cluster_width').value)
        self.association_gate = float(self.get_parameter('association_gate').value)
        self.reacquisition_gate = max(
            self.association_gate,
            float(self.get_parameter('reacquisition_gate').value),
        )
        self.reacquisition_after_missed_scans = max(
            1, int(self.get_parameter('reacquisition_after_missed_scans').value)
        )
        self.missed_lidar_scans = 0

        q_pos = float(self.get_parameter('process_noise_pos').value)
        q_vel = float(self.get_parameter('process_noise_vel').value)
        r_pos = float(self.get_parameter('measurement_noise_pos').value)
        r_vel = float(self.get_parameter('measurement_noise_vel').value)
        self.Q_base = np.diag([q_pos, q_pos, q_vel, q_vel])
        self.R = np.diag([r_pos, r_pos, r_vel, r_vel])
        self.H = np.eye(4)

    def odom_callback(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        z = np.array(
            [
                [msg.pose.pose.position.x],
                [msg.pose.pose.position.y],
                [msg.twist.twist.linear.x],
                [msg.twist.twist.linear.y],
            ]
        )
        self.update_filter(z, stamp)
        self.publish_estimate(msg.header.frame_id, msg.child_frame_id or 'opp_racecar/base_link', msg.header.stamp)

    def ego_odom_callback(self, msg: Odometry) -> None:
        self.ego_odom = msg

    def scan_callback(self, msg: LaserScan) -> None:
        if self.ego_odom is None:
            return

        detection = self.extract_lidar_target(msg)
        if detection is None:
            self.missed_lidar_scans += 1
            return
        self.missed_lidar_scans = 0

        rel_x, rel_y = detection
        ego_yaw = self.yaw_from_odom(self.ego_odom)
        ego_x = self.ego_odom.pose.pose.position.x
        ego_y = self.ego_odom.pose.pose.position.y
        global_x = ego_x + math.cos(ego_yaw) * rel_x - math.sin(ego_yaw) * rel_y
        global_y = ego_y + math.sin(ego_yaw) * rel_x + math.cos(ego_yaw) * rel_y

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vx = 0.0
        vy = 0.0
        if self.last_lidar_detection is not None:
            last_stamp, last_x, last_y = self.last_lidar_detection
            dt = max(stamp - last_stamp, 1e-3)
            vx = (global_x - last_x) / dt
            vy = (global_y - last_y) / dt
        self.last_lidar_detection = (stamp, global_x, global_y)

        z = np.array([[global_x], [global_y], [vx], [vy]])
        self.update_filter(z, stamp)
        self.publish_estimate('map', 'opp_racecar/base_link', msg.header.stamp)

    def extract_lidar_target(self, msg: LaserScan):
        points = []
        half_fov = self.lidar_fov * 0.5
        for idx, dist in enumerate(msg.ranges):
            if not math.isfinite(dist):
                continue
            if dist < self.lidar_min_range or dist > self.lidar_max_range:
                continue
            angle = msg.angle_min + idx * msg.angle_increment
            if abs(angle) > half_fov:
                continue
            points.append((dist * math.cos(angle), dist * math.sin(angle)))

        if len(points) < self.min_cluster_points:
            return None

        clusters = []
        current = [points[0]]
        for point in points[1:]:
            prev = current[-1]
            if math.hypot(point[0] - prev[0], point[1] - prev[1]) <= self.cluster_gap:
                current.append(point)
            else:
                if len(current) >= self.min_cluster_points:
                    clusters.append(current)
                current = [point]
        if len(current) >= self.min_cluster_points:
            clusters.append(current)

        if not clusters:
            return None

        candidates = []
        for cluster in clusters:
            if len(cluster) > self.max_cluster_points:
                continue
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            distance = math.hypot(cx, cy)
            width = self.cluster_width(cluster)
            if width < self.min_cluster_width or width > self.max_cluster_width:
                continue
            if cx <= 0.0:
                continue
            candidates.append((distance, cx, cy))

        if not candidates:
            return None

        if self.initialized and self.ego_odom is not None:
            pred_rel_x, pred_rel_y = self.global_to_ego_frame(float(self.x[0, 0]), float(self.x[1, 0]))
            gated = [
                (math.hypot(cx - pred_rel_x, cy - pred_rel_y), cx, cy)
                for _distance, cx, cy in candidates
            ]
            best_error, cx, cy = min(gated, key=lambda item: item[0])
            gate = (
                self.reacquisition_gate
                if self.missed_lidar_scans >= self.reacquisition_after_missed_scans
                else self.association_gate
            )
            if best_error <= gate and cx > 0.0:
                return cx, cy

            # Keep predicting rather than snapping to a wall-like cluster when
            # no candidate is close to the EKF-predicted opponent location.
            return None

        _distance, cx, cy = min(candidates, key=lambda item: item[0])
        return cx, cy

    def cluster_width(self, cluster) -> float:
        max_width = 0.0
        for i, p0 in enumerate(cluster):
            for p1 in cluster[i + 1:]:
                max_width = max(max_width, math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
        return max_width

    def global_to_ego_frame(self, global_x: float, global_y: float):
        ego_yaw = self.yaw_from_odom(self.ego_odom)
        ego_x = self.ego_odom.pose.pose.position.x
        ego_y = self.ego_odom.pose.pose.position.y
        dx = global_x - ego_x
        dy = global_y - ego_y
        rel_x = math.cos(ego_yaw) * dx + math.sin(ego_yaw) * dy
        rel_y = -math.sin(ego_yaw) * dx + math.cos(ego_yaw) * dy
        return rel_x, rel_y

    def update_filter(self, z, stamp) -> None:
        if not self.initialized:
            self.x = z.copy()
            self.last_stamp = stamp
            self.initialized = True
            return

        dt = max(stamp - self.last_stamp, 1e-3)
        self.last_stamp = stamp

        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        Q = self.Q_base * max(dt, 0.05)

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        innovation = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def publish_estimate(self, frame_id, child_frame_id, stamp) -> None:
        vx = float(self.x[2, 0])
        vy = float(self.x[3, 0])
        yaw = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-4 else 0.0
        quat = Rotation.from_euler('z', yaw).as_quat()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id
        odom.pose.pose.position.x = float(self.x[0, 0])
        odom.pose.pose.position.y = float(self.x[1, 0])
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

    def yaw_from_odom(self, odom: Odometry) -> float:
        q = odom.pose.pose.orientation
        quat = Rotation.from_quat([q.x, q.y, q.z, q.w])
        return quat.as_euler('zxy', degrees=False)[0]


def main(args=None):
    rclpy.init(args=args)
    node = OpponentEKFTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
