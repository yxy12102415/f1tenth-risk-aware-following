# Gym Pure-LiDAR Baseline Evaluation

Run a complete timed trial from `Code/sim_ws`:

```bash
./run_baseline_trial.sh constant_speed 60
```

Arguments are `scenario`, `duration_seconds`, optional `run_id`, and optional
`map_name`. Supported lead scenarios are `constant_speed`, `accel_brake`,
`turning`, and `stop_go`.

Each run writes `results/<scenario>/<run_id>/` with `tracking.csv`,
`following.csv`, `control.csv`, `runtime.csv`, `metadata.json`, and
`summary.json`. The evaluator compares the LiDAR EKF output against Gym ground
truth; ground truth is not subscribed to by the tracker, planner, or MPC.

Generate the deterministic obstacle maps once (or regenerate them identically):

```bash
./generate_baseline_maps.py
```

Available maps are `Melbourne_map_obstacle_center`,
`Melbourne_map_obstacle_left`, and `Melbourne_map_obstacle_right`. For example:

```bash
./run_baseline_trial.sh constant_speed 60 obstacle_center_01 \
  Melbourne_map_obstacle_center
```

These maps only prepare the next evaluation phase. No obstacle-avoidance tuning
is part of this baseline.
