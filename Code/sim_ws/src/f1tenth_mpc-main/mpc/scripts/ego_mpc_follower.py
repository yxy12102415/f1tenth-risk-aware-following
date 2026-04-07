#!/usr/bin/env python3
import math
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.sparse import block_diag, csc_matrix
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


@dataclass
class mpc_config:
    NXK: int = 4
    NU: int = 2
    TK: int = 8
    Rk: list = field(default_factory=lambda: np.diag([0.01, 80.0]))
    Rdk: list = field(default_factory=lambda: np.diag([0.02, 220.0]))
    Qk: list = field(default_factory=lambda: np.diag([25.0, 25.0, 0.12, 0.8]))
    Qfk: list = field(default_factory=lambda: np.diag([25.0, 25.0, 0.12, 0.8]))
    DTK: float = 0.05
    WB: float = 0.33
    MIN_STEER: float = -0.4189
    MAX_STEER: float = 0.4189
    MAX_SPEED: float = 2.4
    MIN_SPEED: float = 0.0
    MAX_ACCEL: float = 0.8


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    v: float = 0.0
    yaw: float = 0.0


def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    quat = Rotation.from_quat([q.x, q.y, q.z, q.w])
    return quat.as_euler('zxy', degrees=False)[0]


class EgoMPCFollower(Node):
    def __init__(self):
        super().__init__('ego_mpc_follower')

        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('target_odom_topic', '/ego_racecar/opp_odom_ekf')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('follow_distance', 1.5)
        self.declare_parameter('target_timeout', 0.3)
        self.declare_parameter('min_speed_command', 0.3)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_accel', 0.8)
        self.declare_parameter('startup_delay', 1.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('wall_stop_distance', 0.45)
        self.declare_parameter('wall_slow_distance', 0.8)
        self.declare_parameter('side_wall_distance', 0.6)

        ego_topic = self.get_parameter('ego_odom_topic').value
        target_topic = self.get_parameter('target_odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.min_speed_command = float(self.get_parameter('min_speed_command').value)
        max_speed = float(self.get_parameter('max_speed').value)
        max_accel = float(self.get_parameter('max_accel').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.wall_stop_distance = float(self.get_parameter('wall_stop_distance').value)
        self.wall_slow_distance = float(self.get_parameter('wall_slow_distance').value)
        self.side_wall_distance = float(self.get_parameter('side_wall_distance').value)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.pred_path_pub = self.create_publisher(Marker, '/ego_mpc_pred_path_vis', 1)
        self.ref_path_pub = self.create_publisher(Marker, '/ego_mpc_ref_path_vis', 1)
        self.create_subscription(Odometry, ego_topic, self.ego_callback, 10)
        self.create_subscription(Odometry, target_topic, self.target_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.config = mpc_config()
        self.config.MAX_SPEED = max_speed
        self.config.MAX_ACCEL = max_accel
        self.ego = None
        self.target = None
        self.target_stamp = None
        self.scan = None
        self.start_time = self.get_clock().now()
        self.oa = None
        self.odelta = None

        self.mpc_prob_init()

    def ego_callback(self, msg: Odometry) -> None:
        self.ego = msg

    def target_callback(self, msg: Odometry) -> None:
        self.target = msg
        self.target_stamp = self.get_clock().now()

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

    def sector_min_distance(self, angle_min_deg: float, angle_max_deg: float) -> float:
        if self.scan is None:
            return float('inf')

        angle_min = math.radians(angle_min_deg)
        angle_max = math.radians(angle_max_deg)
        best = float('inf')

        for idx, dist in enumerate(self.scan.ranges):
            if not math.isfinite(dist):
                continue
            angle = self.scan.angle_min + idx * self.scan.angle_increment
            if angle_min <= angle <= angle_max:
                best = min(best, dist)

        return best

    def control_loop(self) -> None:
        drive = AckermannDriveStamped()

        if (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 < self.startup_delay:
            self.drive_pub.publish(drive)
            return

        if self.ego is None or self.target is None or self.target_stamp is None:
            self.drive_pub.publish(drive)
            return

        age = (self.get_clock().now() - self.target_stamp).nanoseconds * 1e-9
        if age > self.target_timeout:
            self.drive_pub.publish(drive)
            return

        ego_yaw = yaw_from_odom(self.ego)
        ego_speed = math.hypot(self.ego.twist.twist.linear.x, self.ego.twist.twist.linear.y)
        ego_state = State(
            x=self.ego.pose.pose.position.x,
            y=self.ego.pose.pose.position.y,
            v=max(ego_speed, 0.1),
            yaw=ego_yaw,
        )
        gap_distance = math.hypot(
            self.target.pose.pose.position.x - ego_state.x,
            self.target.pose.pose.position.y - ego_state.y,
        )
        x0 = [ego_state.x, ego_state.y, ego_state.v, ego_state.yaw]

        ref_path = self.build_reference_trajectory()
        (
            self.oa,
            self.odelta,
            ox,
            oy,
            _ov,
            _oyaw,
            _path_predict,
        ) = self.linear_mpc_control(ref_path, x0, self.oa, self.odelta)

        if self.oa is None or self.odelta is None:
            self.drive_pub.publish(drive)
            return

        steer_cmd = float(self.odelta[0])
        front_dist = self.sector_min_distance(-12.0, 12.0)
        left_dist = self.sector_min_distance(20.0, 65.0)
        right_dist = self.sector_min_distance(-65.0, -20.0)

        if left_dist < self.side_wall_distance and steer_cmd > 0.0:
            steer_cmd *= max(0.0, left_dist / self.side_wall_distance)
        if right_dist < self.side_wall_distance and steer_cmd < 0.0:
            steer_cmd *= max(0.0, right_dist / self.side_wall_distance)

        drive.drive.steering_angle = steer_cmd
        commanded_speed = ego_state.v + self.oa[0] * self.config.DTK
        if gap_distance <= 0.8 or front_dist <= self.wall_stop_distance:
            drive.drive.speed = 0.0
        else:
            safe_speed = commanded_speed
            if front_dist < self.wall_slow_distance:
                slowdown = (front_dist - self.wall_stop_distance) / max(
                    self.wall_slow_distance - self.wall_stop_distance, 1e-6
                )
                safe_speed *= np.clip(slowdown, 0.0, 1.0)
            drive.drive.speed = float(max(safe_speed, self.min_speed_command))
        self.drive_pub.publish(drive)

        self.visualize_path(ref_path, self.ref_path_pub, 0.9, 0.1, 0.9, 'ego_ref')
        self.visualize_xy_path(ox, oy, self.pred_path_pub, 0.1, 0.9, 0.1, 'ego_pred')

    def build_reference_trajectory(self):
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))

        ex = self.ego.pose.pose.position.x
        ey = self.ego.pose.pose.position.y
        tx = self.target.pose.pose.position.x
        ty = self.target.pose.pose.position.y
        tvx = float(self.target.twist.twist.linear.x)
        tvy = float(self.target.twist.twist.linear.y)
        target_speed = math.hypot(tvx, tvy)
        gap_distance = math.hypot(tx - ex, ty - ey)

        target_yaw = yaw_from_odom(self.target)
        if target_speed > 1e-3:
            target_yaw = math.atan2(tvy, tvx)

        # Slow down aggressively when the ego car is already near the desired
        # following gap so the controller does not drive into the opponent.
        if gap_distance <= 0.8:
            desired_speed = 0.0
        elif gap_distance < 2.0:
            desired_speed = min(target_speed * ((gap_distance - 0.8) / 1.2), self.config.MAX_SPEED)
        else:
            desired_speed = min(target_speed, self.config.MAX_SPEED)

        for i in range(self.config.TK + 1):
            dt = i * self.config.DTK
            pred_tx = tx + tvx * dt
            pred_ty = ty + tvy * dt

            ref_traj[0, i] = pred_tx - self.follow_distance * math.cos(target_yaw)
            ref_traj[1, i] = pred_ty - self.follow_distance * math.sin(target_yaw)
            ref_traj[2, i] = desired_speed
            ref_traj[3, i] = target_yaw

        return ref_traj

    def mpc_prob_init(self):
        self.xk = cvxpy.Variable((self.config.NXK, self.config.TK + 1))
        self.uk = cvxpy.Variable((self.config.NU, self.config.TK))
        objective = 0.0
        constraints = []

        self.x0k = cvxpy.Parameter((self.config.NXK,))
        self.x0k.value = np.zeros((self.config.NXK,))
        self.ref_traj_k = cvxpy.Parameter((self.config.NXK, self.config.TK + 1))
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))

        R_block = block_diag(tuple([self.config.Rk] * self.config.TK))
        Rd_block = block_diag(tuple([self.config.Rdk] * (self.config.TK - 1)))
        Q_block = [self.config.Qk] * self.config.TK
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))

        objective += cvxpy.quad_form(cvxpy.reshape(self.uk, (self.config.NU * self.config.TK, 1), order='F'), R_block)
        objective += cvxpy.quad_form(
            cvxpy.reshape(self.ref_traj_k - self.xk, (self.config.NXK * (self.config.TK + 1), 1), order='F'),
            Q_block,
        )
        objective += cvxpy.quad_form(
            cvxpy.reshape(self.uk[:, 1:] - self.uk[:, :-1], (self.config.NU * (self.config.TK - 1), 1), order='F'),
            Rd_block,
        )

        A_block = []
        B_block = []
        C_block = []
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        m, n = A_block.shape
        self.Annz_k = cvxpy.Parameter(A_block.nnz)
        data = np.ones(self.Annz_k.size)
        rows = A_block.row * n + A_block.col
        cols = np.arange(self.Annz_k.size)
        indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Annz_k.size))
        self.Annz_k.value = A_block.data
        self.Ak_ = cvxpy.reshape(indexer @ self.Annz_k, (m, n), order='C')

        m, n = B_block.shape
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)
        data = np.ones(self.Bnnz_k.size)
        rows = B_block.row * n + B_block.col
        cols = np.arange(self.Bnnz_k.size)
        indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Bnnz_k.size))
        self.Bnnz_k.value = B_block.data
        self.Bk_ = cvxpy.reshape(indexer @ self.Bnnz_k, (m, n), order='C')

        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        constraints += [
            cvxpy.reshape(self.xk[:, 1:], (self.config.NXK * self.config.TK, 1), order='F')
            == self.Ak_ @ cvxpy.reshape(self.xk[:, :-1], (self.config.NXK * self.config.TK, 1), order='F')
            + self.Bk_ @ cvxpy.reshape(self.uk, (self.config.NU * self.config.TK, 1), order='F')
            + cvxpy.reshape(self.Ck_, (self.config.NXK * self.config.TK, 1)),
            self.xk[2, :] <= self.config.MAX_SPEED,
            self.xk[2, :] >= self.config.MIN_SPEED,
            self.xk[:, 0] == self.x0k,
            self.uk[0, :] <= self.config.MAX_ACCEL,
            self.uk[0, :] >= -self.config.MAX_ACCEL,
            self.uk[1, :] <= self.config.MAX_STEER,
            self.uk[1, :] >= self.config.MIN_STEER,
        ]

        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], v=x0[2], yaw=x0[3])
        for ai, di, i in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict

    def update_state(self, state, a, delta):
        delta = max(min(delta, self.config.MAX_STEER), self.config.MIN_STEER)
        state.x += state.v * math.cos(state.yaw) * self.config.DTK
        state.y += state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw += (state.v / self.config.WB) * math.tan(delta) * self.config.DTK
        state.v += a * self.config.DTK
        state.v = max(min(state.v, self.config.MAX_SPEED), self.config.MIN_SPEED)
        return state

    def get_model_matrix(self, v, phi, delta):
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block
        self.ref_traj_k.value = ref_traj

        self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        if self.MPC_prob.status in (cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()
            return oa, odelta, ox, oy, ov, oyaw

        self.get_logger().warn('Ego MPC solve failed')
        return None, None, None, None, None, None

    def linear_mpc_control(self, ref_path, x0, oa, od):
        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        path_predict = self.predict_motion(x0, oa, od, ref_path)
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_v, mpc_yaw = self.mpc_prob_solve(ref_path, path_predict, x0)
        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_v, mpc_yaw, path_predict

    def visualize_path(self, ref_traj, publisher, r, g, b, namespace):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.12
        marker.color.a = 0.8
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        for i in range(ref_traj.shape[1]):
            marker.points.append(Point(x=float(ref_traj[0, i]), y=float(ref_traj[1, i]), z=0.0))
        publisher.publish(marker)

    def visualize_xy_path(self, ox, oy, publisher, r, g, b, namespace):
        if ox is None or oy is None:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.a = 0.9
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        for x, y in zip(ox, oy):
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = EgoMPCFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
