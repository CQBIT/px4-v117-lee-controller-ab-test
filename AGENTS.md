# Repository instructions for Codex

This repository is an executable PX4 v1.17 simulation experiment, not a static code-review task.

1. Read `CODEX_TASK.md` completely before changing code.
2. Use `bash codex/setup_environment.sh` for a fresh cloud Ubuntu environment.
3. Actually run native PX4 v1.17 SITL + Gazebo `gz_x500` + ROS 2 through uXRCE-DDS.
4. Keep the A/B comparison fair: `rate` uses `VehicleRatesSetpoint`; `torque` uses `VehicleThrustSetpoint` + `VehicleTorqueSetpoint`. Do not silently move either branch to a different low-level interface.
5. Calibrate the SI N*m to PX4 normalized torque mapping from the real PX4 v1.17 x500 allocation/model before accepting direct-torque results.
6. Keep debugging build, DDS, frame/sign, arming, normalization, Gazebo and logging failures until the paired experiments run or a genuine cloud-infrastructure limitation is proven by terminal logs.
7. Write quantitative results to `results/` and the final technical report to `RESULTS.md`.
