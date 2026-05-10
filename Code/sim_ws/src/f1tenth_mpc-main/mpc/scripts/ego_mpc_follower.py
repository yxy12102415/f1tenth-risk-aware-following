#!/usr/bin/env python3
import heapq
import math
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.interpolate import CubicSpline
from scipy.sparse import block_diag, csc_matrix
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


@dataclass
class mpc_config:
    NXK: int = 4
    NU: int = 2
    TK: int = 10
    Rk: list = field(default_factory=lambda: np.diag([0.01, 90.0]))
    Rdk: list = field(default_factory=lambda: np.diag([0.02, 260.0]))
    Qk: list = field(default_factory=lambda: np.diag([25.0, 25.0, 0.12, 0.8]))
    Qfk: list = field(default_factory=lambda: np.diag([25.0, 25.0, 0.12, 0.8]))
    DTK: float = 0.05
    WB: float = 0.33
    MIN_STEER: float = -0.50
    MAX_STEER: float = 0.50
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


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class EgoMPCFollower(Node):
    def __init__(self):
        super().__init__('ego_mpc_follower')

        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('target_odom_topic', '/ego_racecar/opp_odom_ekf')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('debug_drive_topic', '/ego_mpc_debug_cmd')
        self.declare_parameter('follow_distance', 1.5)
        self.declare_parameter('target_timeout', 0.3)
        self.declare_parameter('min_speed_command', 0.3)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_accel', 0.8)
        self.declare_parameter('max_steer', 0.4189)
        self.declare_parameter('startup_delay', 1.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('wall_stop_distance', 0.45)
        self.declare_parameter('wall_slow_distance', 0.8)
        self.declare_parameter('cbf_front_gamma', 2.0)
        self.declare_parameter('cbf_q_accel', 1.0)
        self.declare_parameter('cbf_q_steer', 30.0)
        self.declare_parameter('astar_resolution', 0.18)
        self.declare_parameter('astar_grid_radius', 5.5)
        self.declare_parameter('astar_obstacle_inflation', 0.35)
        self.declare_parameter('astar_max_scan_range', 6.0)
        self.declare_parameter('hybrid_astar_yaw_bins', 24)
        self.declare_parameter('hybrid_astar_step_distance', 0.25)
        self.declare_parameter('hybrid_astar_max_expansions', 5000)

        ego_topic = self.get_parameter('ego_odom_topic').value
        target_topic = self.get_parameter('target_odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        debug_drive_topic = self.get_parameter('debug_drive_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.min_speed_command = float(self.get_parameter('min_speed_command').value)
        max_speed = float(self.get_parameter('max_speed').value)
        max_accel = float(self.get_parameter('max_accel').value)
        max_steer = float(self.get_parameter('max_steer').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.wall_stop_distance = float(self.get_parameter('wall_stop_distance').value)
        self.wall_slow_distance = float(self.get_parameter('wall_slow_distance').value)
        self.cbf_front_gamma = float(self.get_parameter('cbf_front_gamma').value)
        self.cbf_q_accel = float(self.get_parameter('cbf_q_accel').value)
        self.cbf_q_steer = float(self.get_parameter('cbf_q_steer').value)
        self.astar_resolution = float(self.get_parameter('astar_resolution').value)
        self.astar_grid_radius = float(self.get_parameter('astar_grid_radius').value)
        self.astar_obstacle_inflation = float(self.get_parameter('astar_obstacle_inflation').value)
        self.astar_max_scan_range = float(self.get_parameter('astar_max_scan_range').value)
        self.hybrid_astar_yaw_bins = int(self.get_parameter('hybrid_astar_yaw_bins').value)
        self.hybrid_astar_step_distance = float(self.get_parameter('hybrid_astar_step_distance').value)
        self.hybrid_astar_max_expansions = int(self.get_parameter('hybrid_astar_max_expansions').value)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.debug_drive_pub = self.create_publisher(AckermannDriveStamped, debug_drive_topic, 10)
        self.pred_path_pub = self.create_publisher(Marker, '/ego_mpc_pred_path_vis', 1)
        self.ref_path_pub = self.create_publisher(Marker, '/ego_mpc_ref_path_vis', 1)
        self.astar_path_pub = self.create_publisher(Marker, '/ego_astar_path_vis', 1)
        self.create_subscription(Odometry, ego_topic, self.ego_callback, 10)
        self.create_subscription(Odometry, target_topic, self.target_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.config = mpc_config()
        self.config.MAX_SPEED = max_speed
        self.config.MAX_ACCEL = max_accel
        self.config.MAX_STEER = max_steer
        self.config.MIN_STEER = -max_steer
        self.ego = None
        self.target = None
        self.target_stamp = None
        self.scan = None
        self.latest_astar_path = []
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
        front_dist = self.sector_min_distance(-12.0, 12.0)
        self.update_safety_margin(front_dist)
        x0 = [ego_state.x, ego_state.y, ego_state.v, ego_state.yaw]

        ref_path = self.build_reference_trajectory(ego_state)
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

        accel_cmd = float(self.oa[0])
        steer_cmd = float(self.odelta[0])

        drive.drive.steering_angle = steer_cmd
        commanded_speed = np.clip(
            ego_state.v + accel_cmd * self.config.DTK,
            self.config.MIN_SPEED,
            self.config.MAX_SPEED,
        )
        drive.drive.speed = float(commanded_speed)
        self.drive_pub.publish(drive)

        debug_drive = AckermannDriveStamped()
        debug_drive.header = drive.header
        debug_drive.drive.steering_angle = steer_cmd
        debug_drive.drive.acceleration = accel_cmd
        debug_drive.drive.speed = drive.drive.speed
        self.debug_drive_pub.publish(debug_drive)

        self.visualize_path(ref_path, self.ref_path_pub, 0.9, 0.1, 0.9, 'ego_ref')
        self.visualize_xy_path(ox, oy, self.pred_path_pub, 0.1, 0.9, 0.1, 'ego_pred')
        self.visualize_point_path(self.latest_astar_path, self.astar_path_pub, 0.0, 0.45, 1.0, 'ego_astar')

    def build_reference_trajectory(self, ego_state: State):
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

        # Use a moderate catch-up profile so the ego car closes large gaps
        # smoothly without overly aggressive acceleration.
        if gap_distance <= 0.8:
            desired_speed = 0.0
        elif gap_distance < 1.6:
            ramp = (gap_distance - 0.8) / 0.6
            desired_speed = min(target_speed * (0.35 + 0.45 * np.clip(ramp, 0.0, 1.0)), self.config.MAX_SPEED)
        else:
            catchup_bonus = min(0.45, 0.2 * (gap_distance - self.follow_distance))
            desired_speed = min(target_speed + catchup_bonus, self.config.MAX_SPEED)

        goal_x = tx - self.follow_distance * math.cos(target_yaw)
        goal_y = ty - self.follow_distance * math.sin(target_yaw)
        path = self.plan_astar_path(ego_state, goal_x, goal_y)
        if len(path) < 2:
            path = [(ex, ey), (goal_x, goal_y)]
        self.latest_astar_path = path

        for i in range(self.config.TK + 1):
            progress = min(i * desired_speed * self.config.DTK, self.path_length(path))
            rx, ry, ryaw = self.sample_path(path, progress, target_yaw)
            ref_traj[0, i] = rx
            ref_traj[1, i] = ry
            ref_traj[2, i] = desired_speed
            ref_traj[3, i] = ryaw

        return ref_traj

    def plan_astar_path(self, ego_state: State, goal_x: float, goal_y: float):
        if self.scan is None:
            return [(ego_state.x, ego_state.y), (goal_x, goal_y)]

        resolution = max(self.astar_resolution, 0.05)
        radius = max(self.astar_grid_radius, resolution * 4.0)
        grid_size = int(math.ceil((2.0 * radius) / resolution)) + 1
        center = grid_size // 2
        occupancy = np.zeros((grid_size, grid_size), dtype=bool)
        inflation_cells = max(1, int(math.ceil(self.astar_obstacle_inflation / resolution)))

        def world_to_grid(x: float, y: float):
            gx = int(round((x - ego_state.x) / resolution)) + center
            gy = int(round((y - ego_state.y) / resolution)) + center
            return gx, gy

        def grid_to_world(cell):
            gx, gy = cell
            x = ego_state.x + (gx - center) * resolution
            y = ego_state.y + (gy - center) * resolution
            return x, y

        for idx, dist in enumerate(self.scan.ranges):
            if not math.isfinite(dist) or dist <= self.scan.range_min:
                continue
            if dist > min(self.scan.range_max, self.astar_max_scan_range):
                continue
            angle = ego_state.yaw + self.scan.angle_min + idx * self.scan.angle_increment
            ox = ego_state.x + dist * math.cos(angle)
            oy = ego_state.y + dist * math.sin(angle)
            gx, gy = world_to_grid(ox, oy)
            if not (0 <= gx < grid_size and 0 <= gy < grid_size):
                continue
            x0 = max(gx - inflation_cells, 0)
            x1 = min(gx + inflation_cells + 1, grid_size)
            y0 = max(gy - inflation_cells, 0)
            y1 = min(gy + inflation_cells + 1, grid_size)
            occupancy[x0:x1, y0:y1] = True

        start = (center, center)
        goal = world_to_grid(goal_x, goal_y)
        goal = (min(max(goal[0], 0), grid_size - 1), min(max(goal[1], 0), grid_size - 1))
        occupancy[start] = False
        goal = self.nearest_free_cell(goal, occupancy)
        if goal is None:
            return [(ego_state.x, ego_state.y), (goal_x, goal_y)]

        path = self.hybrid_astar_search(
            ego_state,
            goal,
            occupancy,
            world_to_grid,
            grid_to_world,
            resolution,
        )
        if not path:
            return [(ego_state.x, ego_state.y), (goal_x, goal_y)]

        path[0] = (ego_state.x, ego_state.y)
        smoothed = self.smooth_grid_path(path)
        splined = self.spline_smooth_path(smoothed)
        if self.path_exceeds_steering_limit(splined):
            return smoothed
        return splined

    def nearest_free_cell(self, cell, occupancy):
        if not occupancy[cell]:
            return cell

        width, height = occupancy.shape
        for radius in range(1, max(width, height)):
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if 0 <= candidate[0] < width and 0 <= candidate[1] < height and not occupancy[candidate]:
                        return candidate
            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if 0 <= candidate[0] < width and 0 <= candidate[1] < height and not occupancy[candidate]:
                        return candidate
        return None

    def hybrid_astar_search(self, start_state, goal, occupancy, world_to_grid, grid_to_world, resolution):
        width, height = occupancy.shape
        yaw_bins = max(self.hybrid_astar_yaw_bins, 8)
        step_distance = max(self.hybrid_astar_step_distance, resolution)
        goal_xy = grid_to_world(goal)
        goal_tolerance = max(1.5 * resolution, 0.22)
        steer_set = [
            self.config.MIN_STEER,
            0.5 * self.config.MIN_STEER,
            0.0,
            0.5 * self.config.MAX_STEER,
            self.config.MAX_STEER,
        ]

        def yaw_to_bin(yaw):
            wrapped = normalize_angle(yaw)
            return int(round((wrapped + math.pi) / (2.0 * math.pi) * yaw_bins)) % yaw_bins

        def pose_to_key(x, y, yaw):
            gx, gy = world_to_grid(x, y)
            return gx, gy, yaw_to_bin(yaw)

        def is_free(x, y):
            gx, gy = world_to_grid(x, y)
            if not (0 <= gx < width and 0 <= gy < height):
                return False
            return not occupancy[gx, gy]

        def rollout(x, y, yaw, steer):
            samples = max(3, int(math.ceil(step_distance / max(resolution, 1e-3))))
            nx, ny, nyaw = x, y, yaw
            for _ in range(samples):
                ds = step_distance / samples
                nx += ds * math.cos(nyaw)
                ny += ds * math.sin(nyaw)
                nyaw = normalize_angle(nyaw + (ds / self.config.WB) * math.tan(steer))
                if not is_free(nx, ny):
                    return None
            return nx, ny, nyaw

        start_key = pose_to_key(start_state.x, start_state.y, start_state.yaw)
        start_pose = (start_state.x, start_state.y, start_state.yaw)
        open_set = []
        counter = 0
        heapq.heappush(open_set, (0.0, counter, start_key))
        came_from = {}
        pose_by_key = {start_key: start_pose}
        g_score = {start_key: 0.0}

        def heuristic(x, y):
            return math.hypot(x - goal_xy[0], y - goal_xy[1])

        expansions = 0
        while open_set:
            _, _, current = heapq.heappop(open_set)
            x, y, yaw = pose_by_key[current]
            if heuristic(x, y) <= goal_tolerance:
                path = [(x, y)]
                while current in came_from:
                    current = came_from[current]
                    px, py, _ = pose_by_key[current]
                    path.append((px, py))
                path.reverse()
                return path

            expansions += 1
            if expansions >= self.hybrid_astar_max_expansions:
                break

            for steer in steer_set:
                next_pose = rollout(x, y, yaw, steer)
                if next_pose is None:
                    continue
                nx, ny, nyaw = next_pose
                next_key = pose_to_key(nx, ny, nyaw)
                if not (0 <= next_key[0] < width and 0 <= next_key[1] < height):
                    continue
                steer_penalty = 0.15 * abs(steer) / max(self.config.MAX_STEER, 1e-3)
                tentative = g_score[current] + step_distance * (1.0 + steer_penalty)
                if steer != 0.0:
                    tentative += 0.05
                if tentative >= g_score.get(next_key, float('inf')):
                    continue
                came_from[next_key] = current
                pose_by_key[next_key] = next_pose
                g_score[next_key] = tentative
                counter += 1
                priority = tentative + heuristic(nx, ny)
                heapq.heappush(open_set, (priority, counter, next_key))

        return []

    def smooth_grid_path(self, path):
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            prev_x, prev_y = smoothed[-1]
            cur_x, cur_y = path[i]
            next_x, next_y = path[i + 1]
            prev_heading = math.atan2(cur_y - prev_y, cur_x - prev_x)
            next_heading = math.atan2(next_y - cur_y, next_x - cur_x)
            if abs(normalize_angle(next_heading - prev_heading)) > 0.1:
                smoothed.append(path[i])
        smoothed.append(path[-1])
        return smoothed

    def spline_smooth_path(self, path):
        if len(path) < 4:
            return path

        distances = [0.0]
        filtered = [path[0]]
        for point in path[1:]:
            if math.hypot(point[0] - filtered[-1][0], point[1] - filtered[-1][1]) < 1e-4:
                continue
            filtered.append(point)
            distances.append(
                distances[-1] + math.hypot(filtered[-1][0] - filtered[-2][0], filtered[-1][1] - filtered[-2][1])
            )

        if len(filtered) < 4 or distances[-1] < 1e-3:
            return filtered

        s = np.array(distances)
        pts = np.array(filtered)
        spline_x = CubicSpline(s, pts[:, 0], bc_type='natural')
        spline_y = CubicSpline(s, pts[:, 1], bc_type='natural')
        sample_count = max(len(filtered) * 3, int(distances[-1] / 0.12))
        sample_s = np.linspace(0.0, distances[-1], sample_count)
        return [(float(spline_x(si)), float(spline_y(si))) for si in sample_s]

    def path_exceeds_steering_limit(self, path) -> bool:
        if len(path) < 3:
            return False
        max_curvature = math.tan(self.config.MAX_STEER) / max(self.config.WB, 1e-6)
        for p0, p1, p2 in zip(path[:-2], path[1:-1], path[2:]):
            a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            c = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
            area2 = abs(
                (p1[0] - p0[0]) * (p2[1] - p0[1])
                - (p1[1] - p0[1]) * (p2[0] - p0[0])
            )
            denom = max(a * b * c, 1e-9)
            curvature = 2.0 * area2 / denom
            if curvature > 1.08 * max_curvature:
                return True
        return False

    def path_length(self, path) -> float:
        return sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]))

    def sample_path(self, path, distance: float, fallback_yaw: float):
        remaining = max(distance, 0.0)
        for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len < 1e-6:
                continue
            if remaining <= seg_len:
                ratio = remaining / seg_len
                x = x0 + ratio * (x1 - x0)
                y = y0 + ratio * (y1 - y0)
                yaw = math.atan2(y1 - y0, x1 - x0)
                return x, y, yaw
            remaining -= seg_len

        if len(path) >= 2:
            x0, y0 = path[-2]
            x1, y1 = path[-1]
            yaw = math.atan2(y1 - y0, x1 - x0)
            return x1, y1, yaw
        return path[0][0], path[0][1], fallback_yaw

    def mpc_prob_init(self):
        self.xk = cvxpy.Variable((self.config.NXK, self.config.TK + 1))
        self.uk = cvxpy.Variable((self.config.NU, self.config.TK))
        objective = 0.0
        constraints = []

        self.x0k = cvxpy.Parameter((self.config.NXK,))
        self.x0k.value = np.zeros((self.config.NXK,))
        self.ref_traj_k = cvxpy.Parameter((self.config.NXK, self.config.TK + 1))
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))
        self.h_front_k = cvxpy.Parameter(nonneg=True, value=5.0)

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

        # Keep only the forward clearance constraint in the MPC.
        constraints += [
            self.config.DTK * self.xk[2, :-1] + (self.config.DTK ** 2) * self.uk[0, :]
            <= self.cbf_front_gamma * self.h_front_k,
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

    def update_safety_margin(self, front_dist: float) -> None:
        def finite_margin(dist: float, threshold: float) -> float:
            if not math.isfinite(dist):
                return 5.0
            return max(dist - threshold, 0.0)

        self.h_front_k.value = finite_margin(front_dist, self.wall_stop_distance)

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

    def visualize_point_path(self, path, publisher, r, g, b, namespace):
        if not path:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.16
        marker.color.a = 0.95
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        for x, y in path:
            marker.points.append(Point(x=float(x), y=float(y), z=0.04))
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
