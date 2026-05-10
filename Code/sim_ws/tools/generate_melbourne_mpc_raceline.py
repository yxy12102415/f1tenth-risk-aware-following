from typing import Tuple
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

SRC = Path('/root/F1-Tenth-Duke-local/Code/sim_ws/src/f1tenth_gym_ros/maps/Melbourne_map.csv')
DST = Path('/root/F1-Tenth-Duke-local/Code/sim_ws/src/f1tenth_mpc-main/mpc/waypoints/Melbourne_map_mpc.csv')

DS = 0.05
SMOOTH_ITERS = 2

V_MIN = 1.0
V_MAX = 2.2
A_LAT_MAX = 2.8
KAPPA_FLOOR = 1e-3
SPEED_SMOOTH_WINDOW = 15
YAW_SMOOTH_WINDOW = 11
MAX_DV_PER_STEP = 0.01


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def smooth_closed(arr: np.ndarray, window: int = 5) -> np.ndarray:
    assert window % 2 == 1
    radius = window // 2
    out = np.zeros_like(arr)
    n = len(arr)
    for i in range(n):
        idx = [(i + k) % n for k in range(-radius, radius + 1)]
        out[i] = np.mean(arr[idx], axis=0)
    return out


def load_centerline(path: Path) -> np.ndarray:
    pts = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            x, y = [float(v) for v in line.split(',')[:2]]
            pts.append((x, y))
    data = np.asarray(pts, dtype=float)
    if len(data) < 5:
        raise RuntimeError('Not enough centerline points')
    return data


def cumulative_arc_length(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    diffs = np.diff(np.vstack([points, points[0]]), axis=0)
    seg = np.hypot(diffs[:, 0], diffs[:, 1])
    s = np.concatenate([[0.0], np.cumsum(seg[:-1])])
    return s, seg, float(np.sum(seg))


def periodic_spline_resample(points: np.ndarray, ds: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    s, _, total_len = cumulative_arc_length(points)
    points_closed = np.vstack([points, points[0]])
    s_closed = np.append(s, total_len)

    cs_x = CubicSpline(s_closed, points_closed[:, 0], bc_type='periodic')
    cs_y = CubicSpline(s_closed, points_closed[:, 1], bc_type='periodic')

    sample_s = np.arange(0.0, total_len, ds)
    x = cs_x(sample_s)
    y = cs_y(sample_s)
    dx = cs_x(sample_s, 1)
    dy = cs_y(sample_s, 1)
    ddx = cs_x(sample_s, 2)
    ddy = cs_y(sample_s, 2)

    pts = np.column_stack([x, y])
    yaw = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
    return pts, yaw, kappa


def smooth_speed_profile(speed: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return speed
    if window % 2 == 0:
        window += 1
    radius = window // 2
    out = np.zeros_like(speed)
    n = len(speed)
    for i in range(n):
        idx = [(i + k) % n for k in range(-radius, radius + 1)]
        out[i] = np.mean(speed[idx])
    return out


def smooth_angle_profile(yaw: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return yaw
    if window % 2 == 0:
        window += 1
    radius = window // 2
    n = len(yaw)
    out = np.zeros_like(yaw)
    for i in range(n):
        idx = [(i + k) % n for k in range(-radius, radius + 1)]
        s = np.sin(yaw[idx]).mean()
        c = np.cos(yaw[idx]).mean()
        out[i] = math.atan2(s, c)
    return out


def limit_speed_ramp(speed: np.ndarray, max_dv: float) -> np.ndarray:
    out = speed.copy()
    n = len(out)
    for _ in range(3):
        for i in range(1, n):
            out[i] = min(out[i], out[i - 1] + max_dv)
        out[0] = min(out[0], out[-1] + max_dv)
        for i in range(n - 2, -1, -1):
            out[i] = min(out[i], out[i + 1] + max_dv)
        out[-1] = min(out[-1], out[0] + max_dv)
    return out


def main() -> None:
    points = load_centerline(SRC)
    for _ in range(SMOOTH_ITERS):
        points = smooth_closed(points, window=5)

    pts, yaw, _ = periodic_spline_resample(points, DS)
    yaw = smooth_angle_profile(yaw, YAW_SMOOTH_WINDOW)

    yaw_unwrapped = np.unwrap(yaw)
    kappa = np.gradient(yaw_unwrapped, DS)

    speed = np.sqrt(A_LAT_MAX / np.maximum(np.abs(kappa), KAPPA_FLOOR))
    speed = np.clip(speed, V_MIN, V_MAX)
    speed = smooth_speed_profile(speed, SPEED_SMOOTH_WINDOW)
    speed = limit_speed_ramp(speed, MAX_DV_PER_STEP)
    speed = np.clip(speed, V_MIN, V_MAX)

    DST.parent.mkdir(parents=True, exist_ok=True)
    with DST.open('w') as f:
        for (x, y), psi, v in zip(pts, yaw, speed):
            f.write(f'{x:.6f},{y:.6f},{wrap_angle(float(psi)):.6f},{float(v):.6f}\n')

    print(f'Saved to: {DST}')
    print(f'Num points: {len(pts)}')
    print(f'Speed range: min={float(np.min(speed)):.3f}, max={float(np.max(speed)):.3f}')
    print('First 10 lines:')
    for (x, y), psi, v in list(zip(pts, yaw, speed))[:10]:
        print(f'{x:.6f},{y:.6f},{wrap_angle(float(psi)):.6f},{float(v):.6f}')


if __name__ == '__main__':
    main()
