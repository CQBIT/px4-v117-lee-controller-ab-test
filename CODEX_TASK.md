# Codex Cloud task: PX4 v1.17 Lee direct-torque vs PX4 body-rate A/B test

Work autonomously in this repository and **actually execute the simulation**, not just inspect code.

## Objective

Quantitatively compare two controller paths on the same PX4 v1.17 + Gazebo x500 + ROS 2 simulation:

1. `rate`: the shared geometric outer loop produces body-rate + thrust setpoints; publish `VehicleRatesSetpoint`; PX4 `mc_rate_control` produces torque and PX4 control allocation drives the x500.
2. `torque`: the same geometric position outer loop plus the full Lee SO(3) moment law produces thrust + body torque; publish `VehicleThrustSetpoint` + `VehicleTorqueSetpoint`; bypass PX4 attitude/rate control and retain PX4 control allocation.

## Required procedure

1. Run `bash codex/setup_environment.sh`. If the environment is already prepared, source `codex/runtime_env.sh` instead.
2. Verify exact versions before testing:
   - `git -C "$PX4_DIR" describe --tags --always` must resolve to PX4 `v1.17.0`.
   - `px4_msgs` must be branch `release/1.17`.
   - Micro XRCE-DDS Agent must be v2.4.3 / v2.x, not v3.x.
   - Record Ubuntu, ROS distro, Gazebo version and compiler version.
3. Inspect `lee_ab_controller.cpp` for frame/sign correctness. PX4 odometry is NED world + FRD body. `VehicleRatesSetpoint` body rates are FRD and multicopter collective thrust is negative body-z. Do not change the A/B concept.
4. **Do not accept the current direct-torque normalization blindly.** Derive or empirically calibrate the SI moment-to-PX4-normalized torque mapping from PX4 v1.17 x500 actuator effectiveness and Gazebo motor model. Document the derivation. The comparison must not be biased by arbitrary torque scaling.
5. Ensure both branches share identical position/velocity outer-loop gains, trajectory/reference generator, state feedback, control update rate, thrust computation/limits and initial condition. Only the attitude-to-low-level interface may differ.
6. Start MicroXRCEAgent on UDP 8888 and run headless `gz_x500` using native PX4 v1.17 SITL.
7. Run at least the following paired tests using `bash scripts/run_case_native.sh`, fixing scripts/code as required until they actually work:
   - hover
   - circle
   - figure8
   - aggressive figure8
8. Add one robustness experiment if feasible without changing the compared control laws: state-feedback delay/update-rate reduction, wind/wrench disturbance, or modest inertial mismatch. Apply exactly the same perturbation to both branches.
9. Collect controller CSV and PX4 ULog for every case. If internal torque/motor signals are not already available in the ULog, use ULog topics rather than modifying PX4 control laws.
10. Run/repair `scripts/analyze_results.py` and produce `results/summary.csv`, XY trajectory overlays, position-error histories, SO(3) attitude-error histories, body-rate tracking error where applicable, torque/actuator demand comparison, and motor saturation percentage.
11. For every paired scenario report position RMSE/max, attitude RMSE/max, rate error, settling/overshoot where meaningful, actuator saturation and a control-effort metric.
12. Check whether the direct-torque branch's `Omega_d`/`dot(Omega_d)` numerical differentiation becomes noisy at lower external update rates. If so, quantify it; do not silently filter only one branch.
13. Do not claim a successful PX4/Gazebo test unless terminal logs prove PX4 v1.17, Gazebo x500 and ROS 2 were all running.

## Deliverables

At completion, leave generated plots/data under `results/` and write `RESULTS.md` with:

- exact environment/version table;
- exact control equations/interfaces used;
- normalization derivation;
- table of A/B metrics;
- observed failure/saturation cases;
- a conclusion answering: *how much control performance is gained/lost by direct Lee torque vs VehicleRatesSetpoint on PX4 v1.17?*
- recommendation for a real PX4 v1.17 aircraft, including the minimum suggested controller update rate if direct torque is used.

If an environment limitation prevents Gazebo/PX4 from running, show the actual failing command/log, then implement the closest valid headless workaround inside the Codex cloud environment. Do not substitute a custom toy dynamics simulation and call it PX4 SITL.
