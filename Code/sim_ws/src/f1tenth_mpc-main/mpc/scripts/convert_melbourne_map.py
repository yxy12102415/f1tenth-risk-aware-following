#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import numpy as np


def resolve_default_paths():
    repo_root = Path(__file__).resolve().parents[6]
    input_path = repo_root / "map" / "Melbourne_map.csv"
    output_path = (
        repo_root
        / "Code"
        / "sim_ws"
        / "src"
        / "f1tenth_mpc-main"
        / "mpc"
        / "waypoints"
        / "Melbourne_map_mpc.csv"
    )
    return input_path, output_path


def load_centerline(csv_path: Path):
    points = []
    with csv_path.open("r", newline="") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row:
                continue
            if row[0].strip().startswith("#"):
                continue
            points.append((float(row[0]), float(row[1])))
    if len(points) < 2:
        raise ValueError(f"Need at least 2 points in {csv_path}")
    return points


def resample_points(points, spacing: float):
    pts = np.asarray(points, dtype=float)
    diffs = np.diff(pts, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = cumulative[-1]
    if total_length <= 0.0:
        raise ValueError("Track length must be positive")

    sample_s = np.arange(0.0, total_length, spacing)
    if sample_s[-1] != total_length:
        sample_s = np.append(sample_s, total_length)

    x_sampled = np.interp(sample_s, cumulative, pts[:, 0])
    y_sampled = np.interp(sample_s, cumulative, pts[:, 1])
    return np.column_stack((x_sampled, y_sampled))


def smooth_points(points, window_size: int):
    if window_size <= 1:
        return points
    padded = np.pad(points, ((window_size // 2, window_size // 2), (0, 0)), mode="edge")
    kernel = np.ones(window_size) / window_size
    x_smooth = np.convolve(padded[:, 0], kernel, mode="valid")
    y_smooth = np.convolve(padded[:, 1], kernel, mode="valid")
    return np.column_stack((x_smooth, y_smooth))


def compute_yaws(points):
    yaws = []
    for index, (x, y) in enumerate(points):
        if index < len(points) - 1:
            next_x, next_y = points[index + 1]
            dx = next_x - x
            dy = next_y - y
        else:
            prev_x, prev_y = points[index - 1]
            dx = x - prev_x
            dy = y - prev_y
        yaws.append(math.atan2(dy, dx))
    return yaws


def write_mpc_waypoints(points, yaws, output_path: Path, speed: float):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        for (x, y), yaw in zip(points, yaws):
            writer.writerow([f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}", f"{speed:.6f}"])


def main():
    default_input, default_output = resolve_default_paths()
    parser = argparse.ArgumentParser(
        description="Convert Melbourne_map.csv centerline into MPC waypoints."
    )
    parser.add_argument("--input", type=Path, default=default_input)
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument("--speed", type=float, default=0.35)
    parser.add_argument("--spacing", type=float, default=0.4)
    parser.add_argument("--smooth-window", type=int, default=9)
    args = parser.parse_args()

    points = load_centerline(args.input)
    points = resample_points(points, args.spacing)
    points = smooth_points(points, args.smooth_window)
    yaws = compute_yaws(points)
    write_mpc_waypoints(points, yaws, args.output, args.speed)
    print(f"Wrote {len(points)} waypoints to {args.output}")


if __name__ == "__main__":
    main()
