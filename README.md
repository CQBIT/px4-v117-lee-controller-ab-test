# PX4 v1.17 + Gazebo x500 + ROS 2 — Lee Torque vs Body-Rate A/B Test

This repository runs a native cloud PX4 v1.17 SITL + Gazebo `x500` + ROS 2 experiment comparing two low-level offboard control architectures on the same vehicle, trajectory, and outer loop.

## Compared architectures

- **PX4 body-rate path**: Lee/geometric outer loop generates body-rate and thrust setpoints, then PX4 closes the rate loop and performs control allocation.
- **Direct-torque path**: Lee/geometric controller generates thrust and body torque setpoints directly, bypassing the PX4 body-rate controller while retaining PX4 control allocation.

The experiment intentionally compares the two interfaces inside the real PX4 SITL stack rather than replacing PX4 with a standalone vehicle model.

## Test matrix

The suite runs four scenarios for 25 seconds each in both modes:

- `hover`
- `circle`
- `figure8`
- `aggressive`

This gives eight independent PX4 SITL cases. Each case must reach **ARMED + OFFBOARD**, produce controller telemetry, and produce a non-empty PX4 ULog before the result set is accepted.

## Main program locations

- Lee ROS 2 controller: `ros2_ws/src/lee_ab_controller/src/lee_ab_controller.cpp`
- Single-case PX4/Gazebo runner: `scripts/run_case_native.sh`
- Paired A/B suite runner: `scripts/run_suite_native.sh`
- Analysis, figures, and GIF generation: `scripts/analyze_results.py`
- GitHub Actions workflow: `.github/workflows/px4-ab-test.yml`
- Cloud environment setup: `codex/setup_environment.sh`

## Result policy

Validated lightweight experiment outputs are retained in `results/` so the A/B result can be inspected directly from the repository. These include controller CSV/log files, summary CSV/JSON, `RESULTS.md`, static plots, and animated trajectory GIFs.

Large files are deliberately excluded from Git history:

- PX4 `*.ulg`
- `px4_gz.log`

The large files remain available in the corresponding GitHub Actions artifact, which is uploaded before the lightweight repository copy is prepared.

## Visualization outputs

The analysis produces global A/B metric plots:

- `results/position_rmse.png`
- `results/attitude_rmse.png`
- `results/rate_tracking_rmse.png`
- `results/motor_saturation.png`

Each scenario also receives a directory under `results/plots/<scenario>/` containing:

- `trajectory_xy.png`
- `trajectory_3d.png`
- `position_tracking.png`
- `position_error.png`
- `attitude_error.png`
- `body_rate_tracking.png`
- `controller_commands.png`
- `motor_outputs.png`
- `trajectory.gif`

The trajectory animations and static tracking plots are generated directly from the real ROS 2 controller CSV data produced during PX4 SITL. The motor-output plots and saturation metrics are derived from the PX4 ULog before the large ULog is excluded from the Git copy.

## Local/cloud execution

On a supported Ubuntu host, the same suite entry point used by CI is:

```bash
bash codex/setup_environment.sh
bash scripts/run_suite_native.sh
```

The GitHub Actions workflow performs the same setup and then uploads the complete result tree as an artifact. A successful same-repository PR run additionally commits the validated lightweight result set back to the PR branch with a `[skip ci]` result-only commit.
