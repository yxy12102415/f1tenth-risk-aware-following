#!/usr/bin/env python3
import csv
import json
import math
import os
from datetime import datetime, timezone

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class BaselineEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('baseline_evaluator')

        self.declare_parameter('scenario', 'constant_speed')
        self.declare_parameter('run_id', '')
        self.declare_parameter('results_root', 'results')
        self.declare_parameter('follow_distance', 0.5)
        self.declare_parameter('estimate_timeout', 0.4)
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('control_topic', '/drive')

        self.scenario = str(self.get_parameter('scenario').value)
        run_id = str(self.get_parameter('run_id').value)
        if not run_id:
            run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        results_root = os.path.abspath(os.path.expanduser(str(self.get_parameter('results_root').value)))
        self.output_dir = os.path.join(results_root, self.scenario, run_id)
        os.makedirs(self.output_dir, exist_ok=True)

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.estimate_timeout = float(self.get_parameter('estimate_timeout').value)
        sample_rate = max(float(self.get_parameter('sample_rate').value), 1.0)

        self.ego = None
        self.ground_truth = None
        self.estimate = None
        self.estimate_receive_time = None
        self.latest_planning_ms = float('nan')
        self.latest_solve_ms = float('nan')
        self.latest_planner_valid = None
        self.start_time = self.get_clock().now()
        self.last_steering = None
        self.last_control_time = None
        self.ever_had_estimate = False
        self.target_lost = False
        self.loss_start_time = None
        self.loss_count = 0
        self.reacquisition_times = []

        self.position_error_sq = []
        self.velocity_error_sq = []
        self.follow_error_sq = []
        self.lead_distances = []
        self.speed_error_sq = []
        self.steering_rates = []
        self.planning_times = []
        self.solve_times = []
        self.planner_results = []

        self.files = {}
        self.writers = {}
        self._open_csv('tracking', [
            'time_s', 'estimate_valid', 'estimate_age_s',
            'estimated_x', 'estimated_y', 'estimated_vx', 'estimated_vy',
            'ground_truth_x', 'ground_truth_y', 'ground_truth_vx', 'ground_truth_vy',
            'position_error_m', 'velocity_error_mps',
        ])
        self._open_csv('following', [
            'time_s', 'lead_distance_m', 'follow_distance_error_m',
            'ego_speed_mps', 'lead_speed_mps', 'speed_error_mps',
        ])
        self._open_csv('control', [
            'time_s', 'speed_command_mps', 'steering_angle_rad', 'steering_rate_radps',
        ])
        self._open_csv('runtime', [
            'time_s', 'planning_time_ms', 'mpc_solve_time_ms', 'planner_valid',
        ])

        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_callback, 20)
        # Ground truth is intentionally consumed only in this evaluator.
        self.create_subscription(Odometry, '/opp_racecar/odom', self.ground_truth_callback, 20)
        self.create_subscription(Odometry, '/ego_racecar/opp_odom_ekf', self.estimate_callback, 20)
        self.control_topic = str(self.get_parameter('control_topic').value)
        self.create_subscription(AckermannDriveStamped, self.control_topic, self.control_callback, 20)
        self.create_subscription(Float64, '/ego_mpc/planning_time_ms', self.planning_callback, 20)
        self.create_subscription(Float64, '/ego_mpc/solve_time_ms', self.solve_callback, 20)
        self.create_subscription(Bool, '/ego_mpc/planner_valid', self.planner_valid_callback, 20)
        self.sample_timer = self.create_timer(1.0 / sample_rate, self.sample)
        self.summary_timer = self.create_timer(1.0, self.write_summary)

        self._write_metadata(run_id)
        self.get_logger().info(f'Baseline results: {self.output_dir}')

    def _open_csv(self, name, fieldnames) -> None:
        handle = open(os.path.join(self.output_dir, f'{name}.csv'), 'w', newline='')
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        handle.flush()
        self.files[name] = handle
        self.writers[name] = writer

    def _write_metadata(self, run_id: str) -> None:
        metadata = {
            'scenario': self.scenario,
            'run_id': run_id,
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'measurement_source': 'lidar',
            'ground_truth_topic': '/opp_racecar/odom',
            'ground_truth_role': 'evaluation_only',
            'estimated_target_topic': '/ego_racecar/opp_odom_ekf',
            'evaluated_control_topic': self.control_topic,
            'follow_distance_m': self.follow_distance,
        }
        with open(os.path.join(self.output_dir, 'metadata.json'), 'w') as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True)

    def elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    @staticmethod
    def speed(msg: Odometry) -> float:
        return math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def ego_callback(self, msg: Odometry) -> None:
        self.ego = msg

    def ground_truth_callback(self, msg: Odometry) -> None:
        self.ground_truth = msg

    def estimate_callback(self, msg: Odometry) -> None:
        now = self.get_clock().now()
        if self.target_lost and self.loss_start_time is not None:
            reacquisition = (now - self.loss_start_time).nanoseconds * 1e-9
            self.reacquisition_times.append(reacquisition)
            self.target_lost = False
            self.loss_start_time = None
        self.estimate = msg
        self.estimate_receive_time = now
        self.ever_had_estimate = True

    def control_callback(self, msg: AckermannDriveStamped) -> None:
        now = self.get_clock().now()
        steering = float(msg.drive.steering_angle)
        steering_rate = float('nan')
        if self.last_steering is not None and self.last_control_time is not None:
            dt = (now - self.last_control_time).nanoseconds * 1e-9
            if dt > 1e-4:
                steering_rate = (steering - self.last_steering) / dt
                if math.isfinite(steering_rate):
                    self.steering_rates.append(steering_rate)
        self.last_steering = steering
        self.last_control_time = now
        self.writers['control'].writerow({
            'time_s': self.elapsed(),
            'speed_command_mps': float(msg.drive.speed),
            'steering_angle_rad': steering,
            'steering_rate_radps': steering_rate,
        })

    def planning_callback(self, msg: Float64) -> None:
        self.latest_planning_ms = float(msg.data)
        if math.isfinite(self.latest_planning_ms):
            self.planning_times.append(self.latest_planning_ms)

    def solve_callback(self, msg: Float64) -> None:
        self.latest_solve_ms = float(msg.data)
        if math.isfinite(self.latest_solve_ms):
            self.solve_times.append(self.latest_solve_ms)

    def planner_valid_callback(self, msg: Bool) -> None:
        self.latest_planner_valid = bool(msg.data)
        self.planner_results.append(self.latest_planner_valid)

    def sample(self) -> None:
        now = self.get_clock().now()
        estimate_age = float('inf')
        if self.estimate_receive_time is not None:
            estimate_age = (now - self.estimate_receive_time).nanoseconds * 1e-9
        estimate_valid = self.estimate is not None and estimate_age <= self.estimate_timeout

        if self.ever_had_estimate and not estimate_valid and not self.target_lost:
            self.target_lost = True
            self.loss_count += 1
            self.loss_start_time = now

        if self.ground_truth is not None:
            gt = self.ground_truth
            row = {
                'time_s': self.elapsed(), 'estimate_valid': int(estimate_valid),
                'estimate_age_s': estimate_age,
                'estimated_x': '', 'estimated_y': '', 'estimated_vx': '', 'estimated_vy': '',
                'ground_truth_x': gt.pose.pose.position.x,
                'ground_truth_y': gt.pose.pose.position.y,
                'ground_truth_vx': gt.twist.twist.linear.x,
                'ground_truth_vy': gt.twist.twist.linear.y,
                'position_error_m': '', 'velocity_error_mps': '',
            }
            if estimate_valid:
                est = self.estimate
                position_error = math.hypot(
                    est.pose.pose.position.x - gt.pose.pose.position.x,
                    est.pose.pose.position.y - gt.pose.pose.position.y,
                )
                velocity_error = math.hypot(
                    est.twist.twist.linear.x - gt.twist.twist.linear.x,
                    est.twist.twist.linear.y - gt.twist.twist.linear.y,
                )
                self.position_error_sq.append(position_error ** 2)
                self.velocity_error_sq.append(velocity_error ** 2)
                row.update({
                    'estimated_x': est.pose.pose.position.x,
                    'estimated_y': est.pose.pose.position.y,
                    'estimated_vx': est.twist.twist.linear.x,
                    'estimated_vy': est.twist.twist.linear.y,
                    'position_error_m': position_error,
                    'velocity_error_mps': velocity_error,
                })
            self.writers['tracking'].writerow(row)

        if self.ego is not None and self.ground_truth is not None:
            lead_distance = math.hypot(
                self.ground_truth.pose.pose.position.x - self.ego.pose.pose.position.x,
                self.ground_truth.pose.pose.position.y - self.ego.pose.pose.position.y,
            )
            follow_error = lead_distance - self.follow_distance
            ego_speed = self.speed(self.ego)
            lead_speed = self.speed(self.ground_truth)
            speed_error = ego_speed - lead_speed
            self.follow_error_sq.append(follow_error ** 2)
            self.lead_distances.append(lead_distance)
            self.speed_error_sq.append(speed_error ** 2)
            self.writers['following'].writerow({
                'time_s': self.elapsed(), 'lead_distance_m': lead_distance,
                'follow_distance_error_m': follow_error,
                'ego_speed_mps': ego_speed, 'lead_speed_mps': lead_speed,
                'speed_error_mps': speed_error,
            })

        self.writers['runtime'].writerow({
            'time_s': self.elapsed(),
            'planning_time_ms': self.latest_planning_ms,
            'mpc_solve_time_ms': self.latest_solve_ms,
            'planner_valid': '' if self.latest_planner_valid is None else int(self.latest_planner_valid),
        })
        for handle in self.files.values():
            handle.flush()

    @staticmethod
    def rmse(squared_values):
        return math.sqrt(float(np.mean(squared_values))) if squared_values else None

    @staticmethod
    def stats(values):
        if not values:
            return {'mean': None, 'p95': None, 'max': None}
        array = np.asarray(values, dtype=float)
        return {
            'mean': float(np.mean(array)),
            'p95': float(np.percentile(array, 95)),
            'max': float(np.max(array)),
        }

    def write_summary(self) -> None:
        planner_attempts = len(self.planner_results)
        planner_successes = sum(self.planner_results)
        summary = {
            'scenario': self.scenario,
            'duration_s': self.elapsed(),
            'tracking': {
                'position_rmse_m': self.rmse(self.position_error_sq),
                'velocity_rmse_mps': self.rmse(self.velocity_error_sq),
                'target_loss_count': self.loss_count,
                'reacquisition_time_s': self.stats(self.reacquisition_times),
            },
            'following': {
                'distance_rmse_m': self.rmse(self.follow_error_sq),
                'minimum_lead_distance_m': min(self.lead_distances) if self.lead_distances else None,
                'speed_rmse_mps': self.rmse(self.speed_error_sq),
            },
            'control': {
                'steering_rate_rms_radps': (
                    math.sqrt(float(np.mean(np.square(self.steering_rates))))
                    if self.steering_rates else None
                ),
            },
            'runtime': {
                'planning_time_ms': self.stats(self.planning_times),
                'mpc_solve_time_ms': self.stats(self.solve_times),
                'planner_attempts': planner_attempts,
                'planner_successes': planner_successes,
                'planner_failures': planner_attempts - planner_successes,
                'planner_success_rate': (
                    planner_successes / planner_attempts if planner_attempts else None
                ),
            },
        }
        path = os.path.join(self.output_dir, 'summary.json')
        temp_path = path + '.tmp'
        with open(temp_path, 'w') as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
        os.replace(temp_path, path)

    def close(self) -> None:
        self.write_summary()
        for handle in self.files.values():
            handle.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaselineEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
