import math
from pathlib import Path
from typing import Tuple

import numpy as np
from scipy.interpolate import CubicSpline

SRC = Path("/root/F1-Tenth-Duke-local/Code/sim_ws/src/f1tenth_gym_ros/maps/Melbourne_map.csv")
DST = Path("/root/F1-Tenth-Duke-local/Code/sim_ws/src/f1tenth_mpc-main/mpc/waypoints/Melbourne_map_mpc.csv")

DS = 0.04
POINT_SMOOTH_ITERS = 3
POINT_SMOOTH_WINDOW = 7
YAW_SMOOTH_WINDOW = 21
SPEED_SMOOTH_WINDOW = 31

V_MIN = 1.2
V_MAX = 3.2
A_LAT_MAX = 2.6
KAPPA_FLOOR = 1e-3
MAX_DV_PER_STEP = 0.012


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def load_centerline(path: Path) -> np.ndarray:
    points = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            x, y = [float(v) for v in line.split(",")[:2]]
            points.append((x, y))
    if len(points) < 5:
        raise RuntimeError(f"Not enough points in {path}")
    return np.asarray(points, dtype=float)


def smooth_closed_points(points: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return points
    if window % 2 == 0:
        window += 1
    radius = window // 2
    out = np.zeros_like(points)
    n = len(points)
    for i in range(n):
        idx = [(i + k) % n for k in range(-radius, radius + 1)]
        out[i] = np.mean(points[idx], axis=0)
    return out


def cumulative_arc_length(points: np.ndarray) -> Tuple[np.ndarray, float]:
    closed = np.vstack([points, points[0]])
    diffs = np.diff(closed, axis=0)
    segment_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
    s = np.concatenate([[0.0], np.cumsum(segment_lengths[:-1])])
    return s, float(np.sum(segment_lengths))


def spline_resample(points: np.ndarray, ds: float):
    s, total_length = cumulative_arc_length(points)
    points_closed = np.vstack([points, points[0]])
    s_closed = np.append(s, total_length)

    spline_x = CubicSpline(s_closed, points_closed[:, 0], bc_type="periodic")
    spline_y = CubicSpline(s_closed, points_closed[:, 1], bc_type="periodic")

    sample_s = np.arange(0.0, total_length, ds)
    x = spline_x(sample_s)
    y = spline_y(sample_s)
    dx = spline_x(sample_s, 1)
    dy = spline_y(sample_s, 1)
    ddx = spline_x(sample_s, 2)
    ddy = spline_y(sample_s, 2)

    points_out = np.column_stack([x, y])
    yaw = np.arctan2(dy, dx)
    curvature = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
    return points_out, yaw, curvature


def smooth_angle_profile(yaw: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return yaw
    if window % 2 == 0:
        window += 1
    radius = window // 2
    out = np.zeros_like(yaw)
    n = len(yaw)
    for i in range(n):
        idx = [(i + k) % n for k in range(-radius, radius + 1)]
        out[i] = math.atan2(np.sin(yaw[idx]).mean(), np.cos(yaw[idx]).mean())
    return out


def smooth_scalar_closed(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values
    if window % 2 == 0:
        window += 1
    radius = window // 2
    out = np.zeros_like(values)
    n = len(values)
    for i in range(n):
        idx = [(i + k) % n for k in range(-radius, radius + 1)]
        out[i] = np.mean(values[idx])
    return out


def limit_speed_ramp(speed: np.ndarray, max_dv: float, passes: int = 5) -> np.ndarray:
    out = speed.copy()
    n = len(out)
    for _ in range(passes):
        for i in range(1, n):
            out[i] = min(out[i], out[i - 1] + max_dv)
        out[0] = min(out[0], out[-1] + max_dv)
        for i in range(n - 2, -1, -1):
            out[i] = min(out[i], out[i + 1] + max_dv)
        out[-1] = min(out[-1], out[0] + max_dv)
    return out


def speed_from_curvature(curvature: np.ndarray) -> np.ndarray:
    speed = np.sqrt(A_LAT_MAX / np.maximum(np.abs(curvature), KAPPA_FLOOR))
    speed = np.clip(speed, V_MIN, V_MAX)
    speed = smooth_scalar_closed(speed, SPEED_SMOOTH_WINDOW)
    speed = limit_speed_ramp(speed, MAX_DV_PER_STEP)
    return np.clip(speed, V_MIN, V_MAX)


def yaw_jump_stats(yaw: np.ndarray):
    jumps = np.abs([wrap_angle(yaw[(i + 1) % len(yaw)] - yaw[i]) for i in range(len(yaw))])
    return float(np.mean(jumps)), float(np.max(jumps)), int(np.sum(jumps > math.radians(5.0)))


def main():
    points = load_centerline(SRC)
    for _ in range(POINT_SMOOTH_ITERS):
        points = smooth_closed_points(points, POINT_SMOOTH_WINDOW)

    points, yaw, curvature = spline_resample(points, DS)
    yaw = smooth_angle_profile(yaw, YAW_SMOOTH_WINDOW)
    curvature = np.gradient(np.unwrap(yaw), DS)
    speed = speed_from_curvature(curvature)

    DST.parent.mkdir(parents=True, exist_ok=True)
    with DST.open("w") as f:
        for (x, y), psi, v in zip(points, yaw, speed):
            f.write(f"{x:.6f},{y:.6f},{wrap_angle(float(psi)):.6f},{float(v):.6f}\n")

    yaw_mean, yaw_max, over_5_deg = yaw_jump_stats(yaw)
    print(f"Saved to: {DST}")
    print(f"Num points: {len(points)}")
    print(f"Spacing: {DS:.3f} m")
    print(f"Speed range: min={float(np.min(speed)):.3f}, max={float(np.max(speed)):.3f}")
    print(f"Yaw jump: mean={math.degrees(yaw_mean):.3f} deg, max={math.degrees(yaw_max):.3f} deg")
    print(f"Yaw jumps over 5 deg: {over_5_deg}")
    print("First 10 lines:")
    with DST.open() as f:
        for _, line in zip(range(10), f):
            print(line.rstrip())


if __name__ == "__main__":
    main()
