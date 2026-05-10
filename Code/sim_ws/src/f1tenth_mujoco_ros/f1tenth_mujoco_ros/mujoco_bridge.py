#!/usr/bin/env python3
import math
import os
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

try:
    import mujoco
except ImportError:  # pragma: no cover - reported at runtime with a ROS log.
    mujoco = None

try:
    import mujoco.viewer
except ImportError:  # pragma: no cover
    mujoco_viewer = None
else:
    mujoco_viewer = mujoco.viewer


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def cross2(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


@dataclass
class SimCar:
    name: str
    drive_topic: str
    odom_topic: str
    base_frame: str
    x: float
    y: float
    yaw: float
    qpos_addr: int
    qvel_addr: int
    speed: float = 0.0
    steer: float = 0.0
    target_speed: float = 0.0
    last_drive_time: Optional[object] = None
    odom_pub: Optional[object] = None


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        self.declare_parameter("model_path", "")
        self.declare_parameter("num_agent", 2)
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("base_frame", "ego_racecar/base_link")
        self.declare_parameter("opp_drive_topic", "/opp_drive")
        self.declare_parameter("opp_odom_topic", "/opp_racecar/odom")
        self.declare_parameter("ego_opp_odom_topic", "/ego_racecar/opp_odom")
        self.declare_parameter("opp_base_frame", "opp_racecar/base_link")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_csv_path", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("use_viewer", True)
        self.declare_parameter("follow_camera", False)
        self.declare_parameter("sx", -23.1983)
        self.declare_parameter("sy", 22.1783)
        self.declare_parameter("stheta", -0.787580)
        self.declare_parameter("sx1", -21.9370)
        self.declare_parameter("sy1", 20.9170)
        self.declare_parameter("stheta1", -0.782731)
        self.declare_parameter("wheelbase", 0.33)
        self.declare_parameter("max_speed", 5.0)
        self.declare_parameter("max_accel", 4.0)
        self.declare_parameter("max_steer", 0.4189)
        self.declare_parameter("control_timeout", 0.5)
        self.declare_parameter("sim_dt", 0.01)
        self.declare_parameter("publish_dt", 0.02)
        self.declare_parameter("scan_publish_dt", 0.12)
        self.declare_parameter("viewer_dt", 0.05)
        self.declare_parameter("scan_fov", 4.7)
        self.declare_parameter("scan_beams", 1080)
        self.declare_parameter("scan_range_min", 0.05)
        self.declare_parameter("scan_range_max", 30.0)
        self.declare_parameter("scan_distance_to_base_link", 0.275)
        self.declare_parameter("opponent_lidar_radius", 0.28)
        self.declare_parameter("track_segment_stride", 8)

        if mujoco is None:
            raise RuntimeError("Python package 'mujoco' is not installed. Install it with: pip install mujoco")

        model_path = self.get_parameter("model_path").value
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory("f1tenth_mujoco_ros"),
                "models",
                "f1tenth_scene.xml",
            )
        if not model_path:
            raise RuntimeError("Parameter 'model_path' must point to a MuJoCo XML file.")

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer: Optional[object] = None

        self.num_agent = int(self.get_parameter("num_agent").value)
        self.drive_topic = self.get_parameter("drive_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.opp_drive_topic = self.get_parameter("opp_drive_topic").value
        self.opp_odom_topic = self.get_parameter("opp_odom_topic").value
        self.ego_opp_odom_topic = self.get_parameter("ego_opp_odom_topic").value
        self.opp_base_frame = self.get_parameter("opp_base_frame").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.use_viewer = self.get_parameter("use_viewer").value
        self.follow_camera = self.get_parameter("follow_camera").value
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_accel = float(self.get_parameter("max_accel").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.control_timeout = float(self.get_parameter("control_timeout").value)
        self.sim_dt = float(self.get_parameter("sim_dt").value)
        self.viewer_dt = float(self.get_parameter("viewer_dt").value)
        self.scan_fov = float(self.get_parameter("scan_fov").value)
        self.scan_beams = int(self.get_parameter("scan_beams").value)
        self.scan_range_min = float(self.get_parameter("scan_range_min").value)
        self.scan_range_max = float(self.get_parameter("scan_range_max").value)
        self.scan_distance_to_base_link = float(self.get_parameter("scan_distance_to_base_link").value)
        self.opponent_lidar_radius = float(self.get_parameter("opponent_lidar_radius").value)
        self.track_segment_stride = max(1, int(self.get_parameter("track_segment_stride").value))
        self.angle_min = -0.5 * self.scan_fov
        self.angle_max = 0.5 * self.scan_fov
        self.angle_increment = self.scan_fov / max(self.scan_beams - 1, 1)
        self.track_segments = self._load_track_segments()
        self.last_viewer_sync_time = self.get_clock().now()

        self.cars = []
        self.ego_car = self._create_car(
            name="ego",
            joint_name="ego_root",
            drive_topic=self.drive_topic,
            odom_topic=self.odom_topic,
            base_frame=self.base_frame,
            sx_param="sx",
            sy_param="sy",
            stheta_param="stheta",
        )
        self.cars.append(self.ego_car)
        self.opp_car = None
        if self.num_agent >= 2:
            self.opp_car = self._create_car(
                name="opp",
                joint_name="opp_root",
                drive_topic=self.opp_drive_topic,
                odom_topic=self.opp_odom_topic,
                base_frame=self.opp_base_frame,
                sx_param="sx1",
                sy_param="sy1",
                stheta_param="stheta1",
            )
            self.cars.append(self.opp_car)

        now = self.get_clock().now()
        for car in self.cars:
            car.last_drive_time = now
            car.odom_pub = self.create_publisher(Odometry, car.odom_topic, 10)
            self.create_subscription(
                AckermannDriveStamped,
                car.drive_topic,
                lambda msg, sim_car=car: self.drive_callback(msg, sim_car),
                10,
            )
            self._write_car_state(car)

        self.ego_opp_odom_pub = (
            self.create_publisher(Odometry, self.ego_opp_odom_topic, 10)
            if self.opp_car is not None
            else None
        )
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.sim_timer = self.create_timer(self.sim_dt, self.sim_timer_callback)
        self.pub_timer = self.create_timer(float(self.get_parameter("publish_dt").value), self.publish_state)
        self.scan_timer = self.create_timer(float(self.get_parameter("scan_publish_dt").value), self.publish_scan)

        if self.use_viewer and mujoco_viewer is not None:
            try:
                self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
                self._set_viewer_camera()
            except Exception as exc:
                self.get_logger().warn(f"MuJoCo viewer disabled: {exc}")

        self.get_logger().info(
            "MuJoCo bridge running: "
            + ", ".join([f"{car.drive_topic}->{car.odom_topic}" for car in self.cars])
        )

    def _create_car(
        self,
        name: str,
        joint_name: str,
        drive_topic: str,
        odom_topic: str,
        base_frame: str,
        sx_param: str,
        sy_param: str,
        stheta_param: str,
    ) -> SimCar:
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo model is missing freejoint '{joint_name}'.")
        return SimCar(
            name=name,
            drive_topic=drive_topic,
            odom_topic=odom_topic,
            base_frame=base_frame,
            x=float(self.get_parameter(sx_param).value),
            y=float(self.get_parameter(sy_param).value),
            yaw=float(self.get_parameter(stheta_param).value),
            qpos_addr=int(self.model.jnt_qposadr[joint_id]),
            qvel_addr=int(self.model.jnt_dofadr[joint_id]),
        )

    def drive_callback(self, msg: AckermannDriveStamped, car: SimCar):
        car.target_speed = clamp(float(msg.drive.speed), -self.max_speed, self.max_speed)
        car.steer = clamp(float(msg.drive.steering_angle), -self.max_steer, self.max_steer)
        car.last_drive_time = self.get_clock().now()

    def sim_timer_callback(self):
        now = self.get_clock().now()
        for car in self.cars:
            age = (now - car.last_drive_time).nanoseconds * 1e-9
            target_speed = 0.0 if age > self.control_timeout else car.target_speed

            dv = clamp(target_speed - car.speed, -self.max_accel * self.sim_dt, self.max_accel * self.sim_dt)
            car.speed = clamp(car.speed + dv, -self.max_speed, self.max_speed)

            yaw_rate = car.speed * math.tan(car.steer) / self.wheelbase
            car.x += car.speed * math.cos(car.yaw) * self.sim_dt
            car.y += car.speed * math.sin(car.yaw) * self.sim_dt
            car.yaw = normalize_angle(car.yaw + yaw_rate * self.sim_dt)
            self._write_car_state(car)

        mujoco.mj_forward(self.model, self.data)
        if self.viewer is not None and self.viewer.is_running():
            elapsed = (now - self.last_viewer_sync_time).nanoseconds * 1e-9
            if elapsed >= self.viewer_dt:
                if self.follow_camera:
                    self._set_viewer_camera()
                self.viewer.sync()
                self.last_viewer_sync_time = now

    def _set_viewer_camera(self):
        if self.viewer is None:
            return
        self.viewer.cam.lookat[0] = self.ego_car.x
        self.viewer.cam.lookat[1] = self.ego_car.y
        self.viewer.cam.lookat[2] = 0.0
        self.viewer.cam.distance = 6.0
        self.viewer.cam.azimuth = 135.0
        self.viewer.cam.elevation = -45.0

    def _write_car_state(self, car: SimCar):
        qx, qy, qz, qw = yaw_to_quat(car.yaw)
        qpos = self.data.qpos
        qpos[car.qpos_addr + 0] = car.x
        qpos[car.qpos_addr + 1] = car.y
        qpos[car.qpos_addr + 2] = 0.09
        qpos[car.qpos_addr + 3] = qw
        qpos[car.qpos_addr + 4] = qx
        qpos[car.qpos_addr + 5] = qy
        qpos[car.qpos_addr + 6] = qz

        qvel = self.data.qvel
        qvel[car.qvel_addr + 0] = car.speed * math.cos(car.yaw)
        qvel[car.qvel_addr + 1] = car.speed * math.sin(car.yaw)
        qvel[car.qvel_addr + 2] = 0.0
        qvel[car.qvel_addr + 3] = 0.0
        qvel[car.qvel_addr + 4] = 0.0
        qvel[car.qvel_addr + 5] = car.speed * math.tan(car.steer) / self.wheelbase

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()
        odoms = {}
        for car in self.cars:
            odom = self._make_odom(car, stamp)
            odoms[car.name] = odom
            car.odom_pub.publish(odom)
            if self.tf_broadcaster is not None:
                self.tf_broadcaster.sendTransform(self._make_transform(car, stamp))

        if self.ego_opp_odom_pub is not None and "opp" in odoms:
            self.ego_opp_odom_pub.publish(odoms["opp"])

    def publish_scan(self) -> None:
        stamp = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "ego_racecar/laser"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        scan.ranges = self._simulate_scan()
        self.scan_pub.publish(scan)

    def _simulate_scan(self):
        ego = self.ego_car
        origin_x = ego.x + self.scan_distance_to_base_link * math.cos(ego.yaw)
        origin_y = ego.y + self.scan_distance_to_base_link * math.sin(ego.yaw)
        ranges = []
        for beam in range(self.scan_beams):
            rel_angle = self.angle_min + beam * self.angle_increment
            world_angle = ego.yaw + rel_angle
            dx = math.cos(world_angle)
            dy = math.sin(world_angle)
            best = self.scan_range_max
            for x0, y0, x1, y1 in self.track_segments:
                hit = self._ray_segment_distance(origin_x, origin_y, dx, dy, x0, y0, x1, y1)
                if hit is not None and self.scan_range_min <= hit < best:
                    best = hit
            if self.opp_car is not None:
                hit = self._ray_circle_distance(
                    origin_x,
                    origin_y,
                    dx,
                    dy,
                    self.opp_car.x,
                    self.opp_car.y,
                    self.opponent_lidar_radius,
                )
                if hit is not None and self.scan_range_min <= hit < best:
                    best = hit
            ranges.append(float(best))
        return ranges

    def _ray_segment_distance(self, ox, oy, dx, dy, x0, y0, x1, y1):
        sx = x1 - x0
        sy = y1 - y0
        denom = cross2(dx, dy, sx, sy)
        if abs(denom) < 1e-9:
            return None
        qpx = x0 - ox
        qpy = y0 - oy
        t = cross2(qpx, qpy, sx, sy) / denom
        u = cross2(qpx, qpy, dx, dy) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    def _ray_circle_distance(self, ox, oy, dx, dy, cx, cy, radius):
        fx = ox - cx
        fy = oy - cy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None
        root = math.sqrt(disc)
        t0 = (-b - root) * 0.5
        t1 = (-b + root) * 0.5
        if t0 >= 0.0:
            return t0
        if t1 >= 0.0:
            return t1
        return None

    def _load_track_segments(self):
        csv_path = self.get_parameter("map_csv_path").value
        if not csv_path:
            csv_path = os.path.join(
                get_package_share_directory("f1tenth_mujoco_ros"),
                "maps",
                "Melbourne_map.csv",
            )
        path = Path(csv_path)
        if not path.exists():
            self.get_logger().warn(f"Track CSV not found for lidar walls: {path}")
            return []

        centers = []
        widths_right = []
        widths_left = []
        with path.open() as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                values = [float(v) for v in line.split(",")[:4]]
                centers.append((values[0], values[1]))
                widths_right.append(values[2])
                widths_left.append(values[3])
        if len(centers) < 3:
            return []

        left = []
        right = []
        n = len(centers)
        for i, (x, y) in enumerate(centers):
            px, py = centers[(i - 2) % n]
            nx, ny = centers[(i + 2) % n]
            yaw = math.atan2(ny - py, nx - px)
            normal_x = -math.sin(yaw)
            normal_y = math.cos(yaw)
            left.append((x + widths_left[i] * normal_x, y + widths_left[i] * normal_y))
            right.append((x - widths_right[i] * normal_x, y - widths_right[i] * normal_y))

        segments = []
        if self.track_segment_stride > 1:
            left = left[::self.track_segment_stride]
            right = right[::self.track_segment_stride]

        for boundary in (left, right):
            for p0, p1 in zip(boundary, boundary[1:] + boundary[:1]):
                segments.append((p0[0], p0[1], p1[0], p1[1]))
        self.get_logger().info(f"Loaded {len(segments)} lidar wall segments from {path}")
        return segments

    def _make_odom(self, car: SimCar, stamp) -> Odometry:
        qx, qy, qz, qw = yaw_to_quat(car.yaw)
        yaw_rate = car.speed * math.tan(car.steer) / self.wheelbase

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = car.base_frame
        odom.pose.pose.position.x = car.x
        odom.pose.pose.position.y = car.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = car.speed * math.cos(car.yaw)
        odom.twist.twist.linear.y = car.speed * math.sin(car.yaw)
        odom.twist.twist.angular.z = yaw_rate
        return odom

    def _make_transform(self, car: SimCar, stamp) -> TransformStamped:
        qx, qy, qz, qw = yaw_to_quat(car.yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = car.base_frame
        transform.transform.translation.x = car.x
        transform.transform.translation.y = car.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridge()
    try:
        rclpy.spin(node)
    finally:
        if node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
