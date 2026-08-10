# PX4 v1.17 + Gazebo x500 + ROS 2 — Lee Torque vs Body-Rate A/B Test

This repository is prepared for a **native Codex Cloud Ubuntu experiment** comparing two low-level PX4 offboard control architectures on the same vehicle, trajectory and outer loop.

## Compared architectures

### A. Geometric body-rate interface

`position/velocity geometric outer loop -> SO(3) attitude error -> body-rate setpoint -> VehicleRatesSetpoint -> PX4 mc_rate_control -> PX4 control_allocator -> Gazebo x500`

### B. Full Lee direct torque interface

`position/velocity geometric outer loop -> full Lee SO(3) moment law -> VehicleThrustSetpoint + VehicleTorqueSetpoint -> PX4 control_allocator -> Gazebo x500`

The purpose is to measure how much performance is gained or lost when the Lee moment law replaces PX4's onboard multicopter rate loop.

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

## Codex Cloud usage

Create/open a Codex cloud environment for this repository with internet access enabled. For a fresh environment run:

```bash
bash codex/setup_environment.sh
```

Then give Codex this instruction:

```text
Read AGENTS.md and CODEX_TASK.md completely, then execute the full experiment autonomously. Do not stop after compilation or code review. Actually run PX4 v1.17 + Gazebo gz_x500 + ROS 2, debug failures, run the paired rate/torque experiments, calibrate the direct-torque normalization, and produce RESULTS.md plus quantitative files under results/.
```

For a manually prepared environment the full base suite is:

```bash
bash scripts/run_suite_native.sh
```

## Base experiments

Both control modes run the same scenarios:

1. hover
2. circle
3. figure-8
4. aggressive figure-8

`CODEX_TASK.md` additionally asks Codex to add an identical robustness/disturbance experiment when feasible.

## Main metrics

- position RMSE and maximum position error
- velocity RMSE
- SO(3) attitude RMSE and maximum attitude error
- body-rate tracking error
- PX4/internal or commanded torque effort
- motor/actuator saturation percentage
- failure and recovery behavior

## Important normalization requirement

`VehicleTorqueSetpoint.xyz` is a PX4-normalized torque demand, while the Lee law naturally produces SI moments in N*m. The initial controller contains provisional scaling parameters only to make the interface explicit. **Do not use those provisional values as final experimental calibration.** Before accepting the direct-torque results, derive or empirically calibrate the N*m -> PX4 normalized torque mapping from the actual PX4 v1.17 x500 actuator effectiveness and Gazebo motor model. This requirement is enforced in `AGENTS.md` and `CODEX_TASK.md`.

## Expected final deliverables

Codex should leave:

```text
RESULTS.md
results/summary.csv
results/summary.json
results/*.png
results/<scenario>_<mode>/controller.csv
results/<scenario>_<mode>/*.ulg
results/<scenario>_<mode>/*.log
```

`RESULTS.md` must quantitatively answer how much direct Lee torque improves or degrades control performance relative to `VehicleRatesSetpoint` on PX4 v1.17, and recommend a practical implementation/update rate for real hardware.
