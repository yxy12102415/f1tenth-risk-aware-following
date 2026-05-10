#!/usr/bin/env python3
import math
from dataclasses import dataclass

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class PIDState:
    integral: float = 0.0
    previous_error: float = 0.0


class EgoEKFFollower(Node):
    def __init__(self):
        super().__init__('ego_ekf_follower')

        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('target_odom_topic', '/ego_racecar/opp_odom_ekf')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('debug_drive_topic', '/ego_pid_debug_cmd')
        self.declare_parameter('follow_distance', 1.5)
        self.declare_parameter('lookahead_distance', 1.2)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('min_accel', -1.5)
        self.declare_parameter('min_speed', 0.2)
        self.declare_parameter('max_steer', 0.4189)
        self.declare_parameter('kp_steer', 1.4)
        self.declare_parameter('ki_steer', 0.0)
        self.declare_parameter('kd_steer', 0.15)
        self.declare_parameter('max_steer_rate', 2.5)
        self.declare_parameter('steer_smoothing', 0.8)
        self.declare_parameter('kp_gap', 1.0)
        self.declare_parameter('ki_gap', 0.0)
        self.declare_parameter('kd_gap', 0.05)
        self.declare_parameter('max_accel', 1.5)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('target_timeout', 0.3)

        ego_topic = self.get_parameter('ego_odom_topic').value
        target_topic = self.get_parameter('target_odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        debug_drive_topic = self.get_parameter('debug_drive_topic').value

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_accel = float(self.get_parameter('min_accel').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.kp_steer = float(self.get_parameter('kp_steer').value)
        self.ki_steer = float(self.get_parameter('ki_steer').value)
        self.kd_steer = float(self.get_parameter('kd_steer').value)
        self.max_steer_rate = float(self.get_parameter('max_steer_rate').value)
        self.steer_smoothing = float(self.get_parameter('steer_smoothing').value)
        self.kp_gap = float(self.get_parameter('kp_gap').value)
        self.ki_gap = float(self.get_parameter('ki_gap').value)
        self.kd_gap = float(self.get_parameter('kd_gap').value)
        self.max_accel = float(self.get_parameter('max_accel').value)
        self.dt = float(self.get_parameter('dt').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.debug_drive_pub = self.create_publisher(AckermannDriveStamped, debug_drive_topic, 10)
        self.create_subscription(Odometry, ego_topic, self.ego_callback, 10)
        self.create_subscription(Odometry, target_topic, self.target_callback, 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.ego = None
        self.target = None
        self.target_stamp = None
        self.steer_pid = PIDState()
        self.gap_pid = PIDState()
        self.prev_steer_cmd = 0.0

    def yaw_from_odom(self, odom: Odometry) -> float:
        q = odom.pose.pose.orientation
        quat = Rotation.from_quat([q.x, q.y, q.z, q.w])
        return quat.as_euler('zxy', degrees=False)[0]

    def ego_callback(self, msg: Odometry) -> None:
        self.ego = msg

    def target_callback(self, msg: Odometry) -> None:
        self.target = msg
        self.target_stamp = self.get_clock().now()

    def control_loop(self) -> None:
        drive = AckermannDriveStamped()

        if self.ego is None or self.target is None or self.target_stamp is None:
            drive.drive.speed = 0.0
            drive.drive.steering_angle = 0.0
            self.drive_pub.publish(drive)
            return

        age = (self.get_clock().now() - self.target_stamp).nanoseconds * 1e-9
        if age > self.target_timeout:
            drive.drive.speed = 0.0
            drive.drive.steering_angle = 0.0
            self.drive_pub.publish(drive)
            return

        ex = float(self.ego.pose.pose.position.x)
        ey = float(self.ego.pose.pose.position.y)
        tx = float(self.target.pose.pose.position.x)
        ty = float(self.target.pose.pose.position.y)
        ego_yaw = self.yaw_from_odom(self.ego)
        current_speed = math.hypot(self.ego.twist.twist.linear.x, self.ego.twist.twist.linear.y)

        tvx = float(self.target.twist.twist.linear.x)
        tvy = float(self.target.twist.twist.linear.y)
        target_speed = math.hypot(tvx, tvy)
        target_yaw = self.yaw_from_odom(self.target)
        if target_speed > 1e-3:
            target_yaw = math.atan2(tvy, tvx)

        ref_x = tx - self.follow_distance * math.cos(target_yaw)
        ref_y = ty - self.follow_distance * math.sin(target_yaw)
        lookahead_x = ref_x + self.lookahead_distance * math.cos(target_yaw)
        lookahead_y = ref_y + self.lookahead_distance * math.sin(target_yaw)
        psi_ref = math.atan2(lookahead_y - ey, lookahead_x - ex)
        heading_error = wrap(psi_ref - ego_yaw)

        steering_nom = self.pid_step(
            heading_error,
            self.steer_pid,
            self.kp_steer,
            self.ki_steer,
            self.kd_steer,
            -self.max_steer,
            self.max_steer,
        )
        max_delta = self.max_steer_rate * self.dt
        steer_limited = float(np.clip(
            steering_nom,
            self.prev_steer_cmd - max_delta,
            self.prev_steer_cmd + max_delta,
        ))
        steering = (
            self.steer_smoothing * self.prev_steer_cmd
            + (1.0 - self.steer_smoothing) * steer_limited
        )
        steering = float(np.clip(steering, -self.max_steer, self.max_steer))
        self.prev_steer_cmd = steering

        gap_distance = math.hypot(tx - ex, ty - ey)
        gap_error = gap_distance - self.follow_distance
        accel_cmd = self.pid_step(
            gap_error,
            self.gap_pid,
            self.kp_gap,
            self.ki_gap,
            self.kd_gap,
            self.min_accel,
            self.max_accel,
        )
        if gap_distance <= self.follow_distance:
            accel_cmd = min(accel_cmd, 0.0)

        speed_cmd = current_speed + accel_cmd * self.dt
        if gap_distance > self.follow_distance + 0.1:
            speed_cmd = max(speed_cmd, self.min_speed)
        else:
            speed_cmd = 0.0
        speed_cmd = min(speed_cmd, self.max_speed)

        drive.drive.speed = float(speed_cmd)
        drive.drive.steering_angle = float(steering)
        self.drive_pub.publish(drive)

        debug_drive = AckermannDriveStamped()
        debug_drive.drive.speed = float(speed_cmd)
        debug_drive.drive.steering_angle = float(steering)
        debug_drive.drive.acceleration = float(accel_cmd)
        self.debug_drive_pub.publish(debug_drive)

    def pid_step(
        self,
        error: float,
        state: PIDState,
        kp: float,
        ki: float,
        kd: float,
        lower: float,
        upper: float,
    ) -> float:
        state.integral += error * self.dt
        derivative = (error - state.previous_error) / self.dt
        state.previous_error = error
        command = kp * error + ki * state.integral + kd * derivative
        return float(np.clip(command, lower, upper))


def main(args=None):
    rclpy.init(args=args)
    node = EgoEKFFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
