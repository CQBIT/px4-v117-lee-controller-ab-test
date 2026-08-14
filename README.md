# PX4 v1.17 + Gazebo x500 + ROS 2 — Lee Torque vs Body-Rate A/B Test

This repository runs a **native PX4 v1.17 SITL + Gazebo `x500` + ROS 2 experiment** comparing two low-level PX4 offboard control architectures on the same vehicle, trajectory, and outer loop.

## Compared architectures

### A. Geometric body-rate interface

`position/velocity geometric outer loop -> SO(3) attitude error -> body-rate setpoint -> VehicleRatesSetpoint -> PX4 mc_rate_control -> PX4 control_allocator -> Gazebo x500`

### B. Full Lee direct-torque interface

`position/velocity geometric outer loop -> full Lee SO(3) moment law -> VehicleThrustSetpoint + VehicleTorqueSetpoint -> PX4 control_allocator -> Gazebo x500`

The experiment measures how much performance is gained or lost when the Lee moment law replaces PX4's onboard multicopter rate loop. It uses the real PX4 SITL stack; standalone or estimated vehicle simulations are not accepted as final results.

## Fixed software targets

- PX4-Autopilot: `v1.17.0`
- Vehicle: Gazebo `gz_x500`
- `px4_msgs`: `release/1.17`
- Micro XRCE-DDS Agent: `v2.4.3`
- ROS 2: Humble on Ubuntu 22.04 or Jazzy on Ubuntu 24.04
- Simulation: headless native PX4 SITL + Gazebo

## Repository layout

```text
AGENTS.md
CODEX_TASK.md
codex/
  setup_environment.sh
ros2_ws/src/lee_ab_controller/
  CMakeLists.txt
  package.xml
  src/lee_ab_controller.cpp
scripts/
  run_case_native.sh
  run_suite_native.sh
  analyze_results.py
results/
```

The executable parts are:

- Lee ROS 2 controller: `ros2_ws/src/lee_ab_controller/src/lee_ab_controller.cpp`
- Single-case PX4/Gazebo runner: `scripts/run_case_native.sh`
- Paired A/B suite runner: `scripts/run_suite_native.sh`
- Analysis, static plots and GIF generation: `scripts/analyze_results.py`
- Cloud setup: `codex/setup_environment.sh`
- GitHub Actions workflow: `.github/workflows/px4-ab-test.yml`

## Cloud/local execution

For a fresh supported Ubuntu environment:

```bash
bash codex/setup_environment.sh
bash scripts/run_suite_native.sh
```

GitHub Actions runs the same native setup and experiment automatically for PR validation.

## Experiment matrix

Both control modes run the same four 25-second scenarios:

1. `hover`
2. `circle`
3. `figure8`
4. `aggressive`

This produces eight independent real PX4 SITL cases. A case is accepted only when the controller log confirms **ARMED + OFFBOARD**, the requested experiment duration is reached, controller telemetry is non-empty, and a non-empty PX4 ULog is produced.

## Main metrics

- position RMSE and maximum position error
- velocity RMSE
- SO(3) attitude RMSE and maximum attitude error
- body-rate tracking error
- PX4/internal or commanded torque effort
- motor/actuator saturation percentage
- failure and recovery behavior

## Visualization outputs

Global A/B comparison figures:

- `results/position_rmse.png`
- `results/attitude_rmse.png`
- `results/rate_tracking_rmse.png`
- `results/motor_saturation.png`

Each scenario also has `results/plots/<scenario>/` containing:

- `trajectory_xy.png`
- `trajectory_3d.png`
- `position_tracking.png`
- `position_error.png`
- `attitude_error.png`
- `body_rate_tracking.png`
- `controller_commands.png`
- `motor_outputs.png`
- `trajectory.gif`

Trajectory plots/animations are generated directly from the real controller CSV recorded during PX4 SITL. Motor-output plots and saturation metrics are derived from the PX4 ULog before the large ULog is removed from the Git copy.

## Result storage policy

The complete GitHub Actions artifact keeps everything needed for forensic analysis, including PX4 ULogs and the large PX4/Gazebo console logs.

Validated lightweight outputs are also retained directly in `results/` in Git so the experiment can be inspected without downloading the artifact. These include:

```text
results/RESULTS.md
results/summary.csv
results/summary.json
results/*.png
results/plots/**/*.png
results/plots/**/*.gif
results/<scenario>_<mode>/controller.csv
results/<scenario>_<mode>/controller.log
results/<scenario>_<mode>/xrce.log
results/<scenario>_<mode>/gcs_heartbeat.log
results/<scenario>_<mode>/vehicle_odometry_probe*.txt
results/<scenario>_<mode>/vehicle_odometry_topic_info.txt
```

The following large files are intentionally excluded from Git history and remain in the Actions artifact only:

```text
results/<scenario>_<mode>/*.ulg
results/<scenario>_<mode>/px4_gz.log
```

A successful same-repository PR run uploads the complete artifact first, validates the full figure/GIF set, then commits only the lightweight result files back to the PR branch with a `[skip ci]` result-only commit.

## Important normalization requirement

`VehicleTorqueSetpoint.xyz` is a PX4-normalized torque demand, while the Lee law naturally produces SI moments in N*m. The controller scaling must be derived or empirically calibrated against the actual PX4 v1.17 `x500` actuator effectiveness and Gazebo motor model before claiming a direct-torque performance benefit. This requirement is retained in `AGENTS.md` and `CODEX_TASK.md`.

## Final deliverables

A validated run must produce:

```text
results/RESULTS.md
results/summary.csv
results/summary.json
results/*.png
results/plots/<scenario>/*.png
results/plots/<scenario>/trajectory.gif
results/<scenario>_<mode>/controller.csv
results/<scenario>_<mode>/*.ulg        # artifact only
results/<scenario>_<mode>/*.log        # px4_gz.log artifact only
```

`RESULTS.md` quantitatively compares direct Lee torque against `VehicleRatesSetpoint` on PX4 v1.17 and records the exact Actions run and tested source commit.
