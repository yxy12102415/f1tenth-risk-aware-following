#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class RaceTrajectoryPlotter(Node):
    def __init__(self) -> None:
        super().__init__("race_trajectory_plotter")

        self.declare_parameter("ego_odom_topic", "/ego_racecar/odom")
        self.declare_parameter("opp_odom_topic", "/opp_racecar/odom")
        self.declare_parameter("target_odom_topic", "/ego_racecar/opp_odom_ekf")
        self.declare_parameter("output_directory", os.path.expanduser("~/f1tenth_plots"))
        self.declare_parameter("plot_filename", "")
        self.declare_parameter("csv_filename", "")
        self.declare_parameter("metrics_plot_filename", "")
        self.declare_parameter("debug_drive_topic", "/ego_mpc_debug_cmd")
        self.declare_parameter("follow_distance", 1.0)
        self.declare_parameter("show_zoom_inset", True)
        self.declare_parameter(
            "track_csv_path",
            "/root/F1-Tenth-Duke-local/Code/sim_ws/src/f1tenth_gym_ros/maps/Melbourne_map.csv",
        )

        ego_topic = self.get_parameter("ego_odom_topic").value
        opp_topic = self.get_parameter("opp_odom_topic").value
        target_topic = self.get_parameter("target_odom_topic").value
        debug_drive_topic = self.get_parameter("debug_drive_topic").value
        self.output_directory = self.get_parameter("output_directory").value
        self.plot_filename = self.get_parameter("plot_filename").value
        self.csv_filename = self.get_parameter("csv_filename").value
        self.metrics_plot_filename = self.get_parameter("metrics_plot_filename").value
        self.follow_distance = float(self.get_parameter("follow_distance").value)
        self.show_zoom_inset = bool(self.get_parameter("show_zoom_inset").value)
        self.track_csv_path = self.get_parameter("track_csv_path").value

        self.ego_traj = []
        self.opp_traj = []
        self.metric_rows = []
        self.latest_opp = None
        self.latest_target = None
        self.latest_drive = None
        self.start_time = None

        self.create_subscription(Odometry, ego_topic, self.ego_callback, 50)
        self.create_subscription(Odometry, opp_topic, self.opp_callback, 50)
        self.create_subscription(Odometry, target_topic, self.target_callback, 50)
        self.create_subscription(AckermannDriveStamped, debug_drive_topic, self.drive_callback, 50)

        self.get_logger().info(
            f"Recording trajectories from {ego_topic} and {opp_topic}. "
            f"Tracking reference metrics will use {target_topic}. "
            f"Tracking metrics will be computed using {debug_drive_topic}. "
            "Plots and CSV will be written on shutdown."
        )

    def ego_callback(self, msg: Odometry) -> None:
        stamp_sec = self._stamp_to_sec(msg)
        if self.start_time is None:
            self.start_time = stamp_sec

        self.ego_traj.append((
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        ))
        metric_target = self.latest_target if self.latest_target is not None else self.latest_opp
        if metric_target is not None:
            self._record_metrics(msg, metric_target, stamp_sec)

    def opp_callback(self, msg: Odometry) -> None:
        self.latest_opp = msg
        self.opp_traj.append((
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        ))

    def target_callback(self, msg: Odometry) -> None:
        self.latest_target = msg

    def drive_callback(self, msg: AckermannDriveStamped) -> None:
        self.latest_drive = msg

    def save_outputs(self) -> None:
        if not self.ego_traj and not self.opp_traj:
            self.get_logger().warning("No trajectory data received; nothing to save.")
            return

        os.makedirs(self.output_directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        plot_name = self.plot_filename or f"race_trajectories_{timestamp}.png"
        csv_name = self.csv_filename or f"race_trajectories_{timestamp}.csv"
        metrics_plot_name = self.metrics_plot_filename or f"tracking_metrics_{timestamp}.png"

        plot_path = os.path.join(self.output_directory, plot_name)
        csv_path = os.path.join(self.output_directory, csv_name)
        metrics_plot_path = os.path.join(self.output_directory, metrics_plot_name)

        self._save_plot(plot_path)
        self._save_metrics_plot(metrics_plot_path)
        self._save_csv(csv_path)

        self.get_logger().info(f"Saved trajectory plot to {plot_path}")
        self.get_logger().info(f"Saved tracking metrics plot to {metrics_plot_path}")
        self.get_logger().info(f"Saved trajectory CSV to {csv_path}")

    def _save_plot(self, plot_path: str) -> None:
        fig, ax = plt.subplots(figsize=(9, 9))

        left_boundary, right_boundary = self._load_track_boundaries(self.track_csv_path)
        if left_boundary is not None and right_boundary is not None:
            ax.plot(
                left_boundary[:, 0],
                left_boundary[:, 1],
                color="black",
                linewidth=3.0,
                alpha=0.85,
                label="Track boundary" if not self.ego_traj and not self.opp_traj else None,
                zorder=1,
            )
            ax.plot(
                right_boundary[:, 0],
                right_boundary[:, 1],
                color="black",
                linewidth=3.0,
                alpha=0.85,
                zorder=1,
            )

        if self.ego_traj:
            ego_x, ego_y = zip(*self.ego_traj)
            ego_line, = ax.plot(
                ego_x,
                ego_y,
                color="#1464F4",
                linewidth=4.5,
                linestyle="--",
                dash_capstyle="round",
                label="Ego",
                zorder=4,
            )
            ego_line.set_path_effects([pe.Stroke(linewidth=7.0, foreground="white"), pe.Normal()])
            ax.scatter([ego_x[0]], [ego_y[0]], color="#1464F4", edgecolors="white", linewidths=1.2, marker="o", s=80, zorder=5)
            ax.scatter([ego_x[-1]], [ego_y[-1]], color="#1464F4", marker="X", s=110, zorder=5)

        if self.opp_traj:
            opp_x, opp_y = zip(*self.opp_traj)
            opp_line, = ax.plot(
                opp_x,
                opp_y,
                color="#F28E1C",
                linewidth=4.8,
                linestyle="-",
                solid_capstyle="round",
                label="Opponent",
                zorder=3,
            )
            opp_line.set_path_effects([pe.Stroke(linewidth=7.5, foreground="white"), pe.Normal()])
            ax.scatter([opp_x[0]], [opp_y[0]], color="#F28E1C", edgecolors="white", linewidths=1.2, marker="o", s=80, zorder=5)
            ax.scatter([opp_x[-1]], [opp_y[-1]], color="#F28E1C", marker="X", s=110, zorder=5)

        if self.show_zoom_inset and self.ego_traj and self.opp_traj:
            self._add_zoom_inset(ax, left_boundary, right_boundary)

        ax.set_title("Ego and Opponent Trajectories with Track Boundaries")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True, alpha=0.2, linestyle="--")
        ax.legend(loc="best", framealpha=0.95)
        fig.tight_layout()
        fig.savefig(plot_path, dpi=200)
        plt.close(fig)

    def _add_zoom_inset(self, ax, left_boundary, right_boundary) -> None:
        n = min(len(self.ego_traj), len(self.opp_traj))
        if n < 5:
            return

        ego = np.array(self.ego_traj[:n])
        opp = np.array(self.opp_traj[:n])
        distances = np.linalg.norm(ego - opp, axis=1)
        idx = int(np.argmin(distances))

        center = 0.5 * (ego[idx] + opp[idx])
        window = slice(max(0, idx - 15), min(n, idx + 16))
        local_pts = np.vstack((ego[window], opp[window]))
        span = np.ptp(local_pts, axis=0)
        half_width = max(4.0, float(span[0]) * 0.8 + 1.5)
        half_height = max(4.0, float(span[1]) * 0.8 + 1.5)

        x1, x2 = center[0] - half_width, center[0] + half_width
        y1, y2 = center[1] - half_height, center[1] + half_height

        axins = inset_axes(ax, width="33%", height="33%", loc="center")
        if left_boundary is not None and right_boundary is not None:
            axins.plot(left_boundary[:, 0], left_boundary[:, 1], color="black", linewidth=2.0, alpha=0.8, zorder=1)
            axins.plot(right_boundary[:, 0], right_boundary[:, 1], color="black", linewidth=2.0, alpha=0.8, zorder=1)

        ego_line, = axins.plot(
            ego[:, 0], ego[:, 1], color="#1464F4", linewidth=3.6, linestyle="--", zorder=4
        )
        opp_line, = axins.plot(
            opp[:, 0], opp[:, 1], color="#F28E1C", linewidth=3.8, linestyle="-", zorder=3
        )
        ego_line.set_path_effects([pe.Stroke(linewidth=5.5, foreground="white"), pe.Normal()])
        opp_line.set_path_effects([pe.Stroke(linewidth=6.0, foreground="white"), pe.Normal()])

        axins.scatter([ego[idx, 0]], [ego[idx, 1]], color="#1464F4", marker="o", s=45, zorder=5)
        axins.scatter([opp[idx, 0]], [opp[idx, 1]], color="#F28E1C", marker="o", s=45, zorder=5)
        axins.set_xlim(x1, x2)
        axins.set_ylim(y1, y2)
        axins.set_xticks([])
        axins.set_yticks([])
        axins.set_title("Zoomed segment", fontsize=10)
        axins.grid(True, alpha=0.15, linestyle="--")
        mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec="0.35", lw=1.0)

    def _save_metrics_plot(self, plot_path: str) -> None:
        if not self.metric_rows:
            self.get_logger().warning("No tracking metrics collected; skipping metrics plot.")
            return

        t = np.array([row["time"] for row in self.metric_rows])
        dist_ref = np.array([row["distance_to_ref_error"] for row in self.metric_rows])
        lateral = np.array([row["lateral_error"] for row in self.metric_rows])
        rel_dist = np.array([row["relative_distance_error"] for row in self.metric_rows])
        steer = np.array([row["steering_cmd"] for row in self.metric_rows])
        accel = np.array([row["accel_cmd"] for row in self.metric_rows])
        steer_delta = np.abs(np.diff(steer, prepend=steer[0]))
        accel_delta = np.abs(np.diff(accel, prepend=accel[0]))

        fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)

        axes[0].plot(t, dist_ref, color="#7A1FA2", linewidth=2.6, label="Distance-to-reference")
        axes[0].plot(t, np.abs(lateral), color="#2A9D8F", linewidth=2.2, linestyle="--", label="|Lateral error|")
        axes[0].set_ylabel("Error [m]")
        axes[0].set_title("Tracking Error Metrics")
        axes[0].grid(True, alpha=0.25, linestyle="--")
        axes[0].legend(loc="best")

        axes[1].plot(t, rel_dist, color="#D62828", linewidth=2.6)
        axes[1].axhline(0.0, color="black", linewidth=1.0, linestyle=":")
        axes[1].set_ylabel("Error [m]")
        axes[1].set_title("Relative Distance Error")
        axes[1].grid(True, alpha=0.25, linestyle="--")

        axes[2].plot(t, steer, color="#1464F4", linewidth=2.4, label="Steering angle command")
        axes[2].plot(t, accel, color="#F28E1C", linewidth=2.4, label="Acceleration command")
        axes[2].set_ylabel("Command")
        axes[2].set_title("MPC Inputs")
        axes[2].grid(True, alpha=0.25, linestyle="--")
        axes[2].legend(loc="best")

        axes[3].plot(t, steer_delta, color="#1D4ED8", linewidth=2.2, label=r"$|\Delta \delta_k|$")
        axes[3].plot(t, accel_delta, color="#EA580C", linewidth=2.2, label=r"$|\Delta a_k|$")
        axes[3].set_xlabel("Time [s]")
        axes[3].set_ylabel("Variation")
        axes[3].set_title("Input Variation")
        axes[3].grid(True, alpha=0.25, linestyle="--")
        axes[3].legend(loc="best")

        fig.tight_layout()
        fig.savefig(plot_path, dpi=200)
        plt.close(fig)

    def _load_track_boundaries(self, csv_path: str):
        if not os.path.exists(csv_path):
            self.get_logger().warning(f"Track CSV not found: {csv_path}")
            return None, None

        center_x = []
        center_y = []
        width_right = []
        width_left = []

        with open(csv_path, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith("#"):
                    continue
                try:
                    x_m, y_m, w_r, w_l = map(float, row[:4])
                except ValueError:
                    continue
                center_x.append(x_m)
                center_y.append(y_m)
                width_right.append(w_r)
                width_left.append(w_l)

        if len(center_x) < 3:
            return None, None

        center = np.column_stack((np.array(center_x), np.array(center_y)))
        width_right = np.array(width_right)
        width_left = np.array(width_left)

        dx = np.gradient(center[:, 0])
        dy = np.gradient(center[:, 1])
        tangent_norm = np.hypot(dx, dy)
        tangent_norm[tangent_norm < 1e-6] = 1.0

        nx = -dy / tangent_norm
        ny = dx / tangent_norm
        normal = np.column_stack((nx, ny))

        left_boundary = center + normal * width_left[:, None]
        right_boundary = center - normal * width_right[:, None]
        return left_boundary, right_boundary

    def _save_csv(self, csv_path: str) -> None:
        max_len = max(len(self.ego_traj), len(self.opp_traj))
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "index",
                "ego_x",
                "ego_y",
                "opp_x",
                "opp_y",
                "time_s",
                "ref_x",
                "ref_y",
                "distance_to_ref_error",
                "lateral_error",
                "relative_distance_error",
                "steering_cmd",
                "accel_cmd",
                "steering_delta",
                "accel_delta",
            ])
            for i in range(max_len):
                ego = self.ego_traj[i] if i < len(self.ego_traj) else ("", "")
                opp = self.opp_traj[i] if i < len(self.opp_traj) else ("", "")
                metric = self.metric_rows[i] if i < len(self.metric_rows) else None
                writer.writerow([
                    i,
                    ego[0],
                    ego[1],
                    opp[0],
                    opp[1],
                    metric["time"] if metric else "",
                    metric["ref_x"] if metric else "",
                    metric["ref_y"] if metric else "",
                    metric["distance_to_ref_error"] if metric else "",
                    metric["lateral_error"] if metric else "",
                    metric["relative_distance_error"] if metric else "",
                    metric["steering_cmd"] if metric else "",
                    metric["accel_cmd"] if metric else "",
                    metric["steering_delta"] if metric else "",
                    metric["accel_delta"] if metric else "",
                ])

    def _record_metrics(self, ego_msg: Odometry, target_msg: Odometry, stamp_sec: float) -> None:
        ego_x = float(ego_msg.pose.pose.position.x)
        ego_y = float(ego_msg.pose.pose.position.y)
        target_x = float(target_msg.pose.pose.position.x)
        target_y = float(target_msg.pose.pose.position.y)

        target_vx = float(target_msg.twist.twist.linear.x)
        target_vy = float(target_msg.twist.twist.linear.y)
        target_speed = float(np.hypot(target_vx, target_vy))
        target_yaw = self._yaw_from_odom(target_msg)
        if target_speed > 1e-3:
            target_yaw = float(np.arctan2(target_vy, target_vx))

        ref_x = target_x - self.follow_distance * np.cos(target_yaw)
        ref_y = target_y - self.follow_distance * np.sin(target_yaw)

        err_x = ego_x - ref_x
        err_y = ego_y - ref_y
        tangent = np.array([np.cos(target_yaw), np.sin(target_yaw)])
        normal = np.array([-np.sin(target_yaw), np.cos(target_yaw)])
        lateral_error = float(np.dot(np.array([err_x, err_y]), normal))

        steering_cmd = 0.0
        accel_cmd = 0.0
        if self.latest_drive is not None:
            steering_cmd = float(self.latest_drive.drive.steering_angle)
            accel_cmd = float(self.latest_drive.drive.acceleration)
        prev_steer = self.metric_rows[-1]["steering_cmd"] if self.metric_rows else steering_cmd
        prev_accel = self.metric_rows[-1]["accel_cmd"] if self.metric_rows else accel_cmd

        self.metric_rows.append({
            "time": float(stamp_sec - self.start_time),
            "ref_x": float(ref_x),
            "ref_y": float(ref_y),
            "distance_to_ref_error": float(np.hypot(err_x, err_y)),
            "lateral_error": lateral_error,
            "relative_distance_error": float(np.hypot(target_x - ego_x, target_y - ego_y) - self.follow_distance),
            "steering_cmd": steering_cmd,
            "accel_cmd": accel_cmd,
            "steering_delta": float(abs(steering_cmd - prev_steer)),
            "accel_delta": float(abs(accel_cmd - prev_accel)),
        })

    def _stamp_to_sec(self, odom_msg: Odometry) -> float:
        stamp = odom_msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _yaw_from_odom(self, odom_msg: Odometry) -> float:
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(np.arctan2(siny_cosp, cosy_cosp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceTrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_outputs()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
