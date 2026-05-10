#!/usr/bin/env python3
"""Proof-friendly fixed-LTI MPC for straight-line path-following.

This version intentionally aligns the implementation with the lecture proof
structure:

1. A fixed discrete-time LTI error model is used in both implementation and
   theory.
2. The controller solves a finite-horizon constrained MPC problem online.
3. Terminal feedback, terminal cost, and terminal ellipsoid are built from the
   same fixed model.

The reference is an infinite straight path with constant speed, so the
controller regulates the local path-following error dynamics rather than a
rolling waypoint sequence.
"""

import csv
import math
import os
import sys
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.linalg import solve_discrete_are
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class mpc_config:
    NXK: int = 3  # error state: [e_y, e_psi, e_v]
    NU: int = 2  # input: [a, delta]
    TK: int = 8

    DTK: float = 0.05
    WB: float = 0.33

    REF_SPEED: float = 2.0
    REF_Y: float = 0.0
    REF_YAW: float = 0.0
    REF_PATH_LENGTH: float = 20.0
    REF_PATH_RESOLUTION: float = 0.2

    MAX_STEER: float = 0.4189
    MIN_STEER: float = -0.4189
    MAX_SPEED: float = 3.0
    MIN_SPEED: float = 0.0
    MAX_ACCEL: float = 1.0

    MAX_LATERAL_ERROR: float = 2.0
    MAX_HEADING_ERROR: float = 0.7

    Qk: list = field(
        default_factory=lambda: np.diag([30.0, 12.0, 1.0])
    )
    Qfk: list = field(
        default_factory=lambda: np.diag([30.0, 12.0, 1.0])
    )
    Rk: list = field(
        default_factory=lambda: np.diag([0.4, 8.0])
    )
    Rdk: list = field(
        default_factory=lambda: np.diag([0.08, 20.0])
    )


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    v: float = 0.0
    yaw: float = 0.0


class CourseWaypointMPC(Node):
    """Fixed-LTI straight-line MPC node."""

    def __init__(self):
        super().__init__("course_waypoint_mpc")
        self.share_directory = get_package_share_directory("mpc")
        self.output_directory = os.path.join(self.share_directory, "waypoints")
        self.trajectory_data = []

        self.declare_parameter("drive_topic", "/drive")
        self.drive_topic = self.get_parameter("drive_topic").get_parameter_value().string_value
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, self.drive_topic, 1)
        self.ref_path_vis_pub_ = self.create_publisher(Marker, "/ref_path_vis", 1)
        self.pred_path_vis_pub_ = self.create_publisher(Marker, "/pred_path_vis", 1)

        self.declare_parameter("pose_topic", "/ego_racecar/odom")
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.pose_sub_ = self.create_subscription(Odometry, self.pose_topic, self.pose_callback, 1)

        self.declare_parameter("record_trajectory", False)
        self.record_trajectory = self.get_parameter("record_trajectory").get_parameter_value().bool_value

        self.config = mpc_config()
        self.A, self.B = self.get_fixed_error_model()
        self.K_terminal, self.P_terminal = self.compute_terminal_lqr()
        self.config.Qfk = self.P_terminal.copy()
        self.terminal_rho = self.compute_terminal_set_radius()

        self.prev_u = None
        self.mpc_prob_init()
        self.visualize_reference_line()

        self.get_logger().info(
            "Initialized proof-friendly course MPC with fixed LTI error model, "
            "batch QP, terminal LQR ingredients, and terminal ellipsoid."
        )

    def write_trajectory_to_csv(self):
        filename = os.path.join(self.output_directory, "trajectory_recorded.csv")
        with open(filename, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y"])
            writer.writerows(self.trajectory_data)
        self.get_logger().info(f"Trajectory data saved to {filename}")

    def get_fixed_error_model(self):
        """Return the fixed discrete-time LTI path-following error model."""
        dt = self.config.DTK
        v0 = self.config.REF_SPEED
        wb = self.config.WB

        A = np.array(
            [
                [1.0, dt * v0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        B = np.array(
            [
                [0.0, 0.0],
                [0.0, dt * v0 / wb],
                [dt, 0.0],
            ]
        )
        return A, B

    def compute_terminal_lqr(self):
        """Compute terminal feedback and terminal cost from fixed LTI model."""
        P = solve_discrete_are(self.A, self.B, self.config.Qk, self.config.Rk)
        gain = np.linalg.solve(
            self.config.Rk + self.B.T @ P @ self.B,
            self.B.T @ P @ self.A,
        )
        return -gain, P

    def compute_terminal_set_radius(self):
        """Choose rho so X_f is admissible for the fixed terminal controller."""
        p_inv = np.linalg.inv(self.P_terminal)
        rho_candidates = []

        input_bounds = [self.config.MAX_ACCEL, self.config.MAX_STEER]
        for row, bound in zip(self.K_terminal, input_bounds):
            gain_energy = float(row @ p_inv @ row.T)
            if gain_energy > 1e-12:
                rho_candidates.append((bound**2) / gain_energy)

        speed_bound = min(
            self.config.REF_SPEED - self.config.MIN_SPEED,
            self.config.MAX_SPEED - self.config.REF_SPEED,
        )
        speed_selector = np.array([0.0, 0.0, 1.0])
        speed_energy = float(speed_selector @ p_inv @ speed_selector.T)
        if speed_bound > 0.0 and speed_energy > 1e-12:
            rho_candidates.append((speed_bound**2) / speed_energy)

        lateral_selector = np.array([1.0, 0.0, 0.0])
        lateral_energy = float(lateral_selector @ p_inv @ lateral_selector.T)
        if lateral_energy > 1e-12:
            rho_candidates.append((self.config.MAX_LATERAL_ERROR**2) / lateral_energy)

        heading_selector = np.array([0.0, 1.0, 0.0])
        heading_energy = float(heading_selector @ p_inv @ heading_selector.T)
        if heading_energy > 1e-12:
            rho_candidates.append((self.config.MAX_HEADING_ERROR**2) / heading_energy)

        if not rho_candidates:
            self.get_logger().warning(
                "Falling back to default terminal radius because no admissibility bound was derived."
            )
            return 1.0

        return 0.8 * min(rho_candidates)

    def mpc_prob_init(self):
        """Create the fixed-LTI finite-horizon constrained MPC problem."""
        nx = self.config.NXK
        nu = self.config.NU
        horizon = self.config.TK

        self.ek = cvxpy.Variable((nx, horizon + 1))
        self.uk = cvxpy.Variable((nu, horizon))
        self.e0k = cvxpy.Parameter(nx)
        self.e0k.value = np.zeros(nx)

        objective = 0
        constraints = [self.ek[:, 0] == self.e0k]

        for t in range(horizon):
            objective += cvxpy.quad_form(self.ek[:, t], self.config.Qk)
            objective += cvxpy.quad_form(self.uk[:, t], self.config.Rk)
            if t < horizon - 1:
                objective += cvxpy.quad_form(self.uk[:, t + 1] - self.uk[:, t], self.config.Rdk)

            constraints += [
                self.ek[:, t + 1] == self.A @ self.ek[:, t] + self.B @ self.uk[:, t],
                self.uk[0, t] <= self.config.MAX_ACCEL,
                self.uk[0, t] >= -self.config.MAX_ACCEL,
                self.uk[1, t] <= self.config.MAX_STEER,
                self.uk[1, t] >= self.config.MIN_STEER,
                self.ek[2, t] <= self.config.MAX_SPEED - self.config.REF_SPEED,
                self.ek[2, t] >= self.config.MIN_SPEED - self.config.REF_SPEED,
                cvxpy.abs(self.ek[0, t]) <= self.config.MAX_LATERAL_ERROR,
                cvxpy.abs(self.ek[1, t]) <= self.config.MAX_HEADING_ERROR,
            ]

        objective += cvxpy.quad_form(self.ek[:, horizon], self.config.Qfk)
        constraints += [
            self.ek[2, horizon] <= self.config.MAX_SPEED - self.config.REF_SPEED,
            self.ek[2, horizon] >= self.config.MIN_SPEED - self.config.REF_SPEED,
            cvxpy.abs(self.ek[0, horizon]) <= self.config.MAX_LATERAL_ERROR,
            cvxpy.abs(self.ek[1, horizon]) <= self.config.MAX_HEADING_ERROR,
            cvxpy.quad_form(self.ek[:, horizon], self.P_terminal) <= self.terminal_rho,
        ]

        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def pose_callback(self, pose_msg):
        state = self.extract_vehicle_state(pose_msg)
        error_state = self.compute_error_state(state)

        oa, odelta, ey_pred, x_pred = self.solve_mpc(error_state)
        if oa is None or odelta is None:
            return

        self.prev_u = np.vstack((oa, odelta))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(odelta[0])
        drive_msg.drive.speed = max(state.v + float(oa[0]) * self.config.DTK, 0.3)
        self.drive_pub_.publish(drive_msg)

        self.visualize_prediction(state.x, ey_pred)

        if self.record_trajectory:
            self.trajectory_data.append([state.x, state.y])

    def extract_vehicle_state(self, pose_msg):
        xp = pose_msg.pose.pose.position.x
        yp = pose_msg.pose.pose.position.y

        quat = [
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w,
        ]
        yaw = Rotation.from_quat(quat).as_euler("zxy", degrees=False)[0]
        lin_speed = [
            pose_msg.twist.twist.linear.x,
            pose_msg.twist.twist.linear.y,
        ]
        vp = max(np.linalg.norm(lin_speed, 2), 0.0)
        return VehicleState(x=xp, y=yp, v=vp, yaw=yaw)

    def compute_error_state(self, state):
        """Straight-line path-following error state around y_ref=0, yaw_ref=0, v_ref=v0."""
        return np.array(
            [
                state.y - self.config.REF_Y,
                normalize_angle(state.yaw - self.config.REF_YAW),
                state.v - self.config.REF_SPEED,
            ]
        )

    def solve_mpc(self, error_state):
        self.e0k.value = error_state

        if self.prev_u is not None:
            self.uk.value = self.prev_u

        self.MPC_prob.solve(solver=cvxpy.OSQP, warm_start=True, verbose=False)

        if self.MPC_prob.status not in (cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE):
            self.get_logger().warning("Unable to solve fixed-LTI course MPC QP at this step.")
            return None, None, None, None

        oa = np.array(self.uk.value[0, :]).flatten()
        odelta = np.array(self.uk.value[1, :]).flatten()
        ey_pred = np.array(self.ek.value[0, :]).flatten()
        e_pred = np.array(self.ek.value).copy()
        return oa, odelta, ey_pred, e_pred

    def visualize_reference_line(self):
        ref_strip = Marker()
        ref_strip.header.frame_id = "map"
        ref_strip.ns = "straight_ref"
        ref_strip.id = 10
        ref_strip.type = Marker.LINE_STRIP
        ref_strip.action = Marker.ADD
        ref_strip.scale.x = 0.12
        ref_strip.color.a = 0.45
        ref_strip.color.r = 1.0
        ref_strip.color.g = 0.0
        ref_strip.color.b = 1.0

        x_values = np.arange(
            0.0,
            self.config.REF_PATH_LENGTH + self.config.REF_PATH_RESOLUTION,
            self.config.REF_PATH_RESOLUTION,
        )
        for x_val in x_values:
            ref_strip.points.append(Point(x=float(x_val), y=self.config.REF_Y, z=0.0))

        self.ref_path_vis_pub_.publish(ref_strip)

    def visualize_prediction(self, current_x, ey_pred):
        pred_strip = Marker()
        pred_strip.header.frame_id = "map"
        pred_strip.ns = "pred_path"
        pred_strip.id = 1000
        pred_strip.type = Marker.LINE_STRIP
        pred_strip.action = Marker.ADD
        pred_strip.scale.x = 0.08
        pred_strip.color.a = 1.0
        pred_strip.color.r = 0.0
        pred_strip.color.g = 1.0
        pred_strip.color.b = 0.0

        for i, ey in enumerate(ey_pred):
            x_pred = current_x + i * self.config.REF_SPEED * self.config.DTK
            pred_strip.points.append(Point(x=float(x_pred), y=float(ey), z=0.0))

        self.pred_path_vis_pub_.publish(pred_strip)

    def destroy_node(self):
        if self.record_trajectory and self.trajectory_data:
            self.write_trajectory_to_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = CourseWaypointMPC()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Course waypoint MPC stopped cleanly.")
    except BaseException:
        print("Exception in course waypoint MPC:", file=sys.stderr)
        raise
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
