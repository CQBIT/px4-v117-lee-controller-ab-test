# Codex Cloud task: PX4 v1.17 Lee direct-torque vs PX4 body-rate A/B test

Work autonomously in this repository and **actually execute the simulation**, not just inspect code.

The required validation target is a real PX4 v1.17 SITL + Gazebo `x500` + ROS 2 experiment comparing the Lee body-rate interface against direct thrust/torque offboard control. Standalone or estimated simulation output must not be substituted for PX4 SITL data.

The final validated run must produce all eight cases (`hover`, `circle`, `figure8`, `aggressive` × `rate`, `torque`), controller CSV/logs, non-empty PX4 ULogs, `summary.csv`, `summary.json`, `RESULTS.md`, global metric plots, per-scenario trajectory/error/body-rate/controller/actuator plots, and animated `trajectory.gif` files.

After the complete artifact is uploaded, retain lightweight results in the repository while leaving the large `*.ulg` and `px4_gz.log` files in the GitHub Actions artifact only.
