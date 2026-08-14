# PX4 v1.17 Lee Controller A/B SITL Results

## Provenance and validity

This report is generated from the real PX4 v1.17 SITL + Gazebo `x500` + ROS 2 experiment outputs. It is not based on a standalone or estimated simulation. Each of the eight paired cases contains controller telemetry and at least one non-empty PX4 ULog, and the controller log confirms that PX4 reached **ARMED + OFFBOARD** before the experiment clock started.

- GitHub Actions run: https://github.com/CQBIT/px4-v117-lee-controller-ab-test/actions/runs/31779385060
- Tested source commit: `ad121ebe67c4c593a56c39801f3a75fe2136b3f5`
- Cases: 4 scenarios × 2 control paths = 8
- Requested case duration: 25 s
- Primary ULog selection: largest non-empty `.ulg` in each case directory
- Repository policy: lightweight CSV/log/plots/report files are retained in Git; `.ulg` and `px4_gz.log` remain in the complete Actions artifact only.

| Scenario | Mode | CSV samples | Flight time [s] | Armed+Offboard | ULogs | Primary ULog | Primary bytes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| aggressive | rate | 2705 | 25.02 | yes | 1 | 07_42_11.ulg | 10602884 |
| aggressive | torque | 2705 | 25.02 | yes | 1 | 07_43_00.ulg | 9605253 |
| circle | rate | 2705 | 25.02 | yes | 1 | 07_38_55.ulg | 10582488 |
| circle | torque | 2704 | 25.02 | yes | 1 | 07_39_45.ulg | 9501228 |
| figure8 | rate | 2705 | 25.02 | yes | 1 | 07_40_34.ulg | 10286761 |
| figure8 | torque | 2704 | 25.01 | yes | 1 | 07_41_22.ulg | 9201813 |
| hover | rate | 2705 | 25.02 | yes | 1 | 07_37_14.ulg | 10766637 |
| hover | torque | 2705 | 25.02 | yes | 1 | 07_38_04.ulg | 10007349 |

## Main comparison

Lower is better for all error and saturation metrics below.

| Scenario | Pos RMSE rate [m] | Pos RMSE torque [m] | Att RMSE rate [deg] | Att RMSE torque [deg] | Rate-track RMSE rate [rad/s] | Rate-track RMSE torque [rad/s] | Motor sat rate [%] | Motor sat torque [%] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| hover | 0.377 | 0.374 | 0.049 | 0.030 | 0.006 | 0.007 | 28.53 | 31.98 |
| circle | 0.378 | 0.374 | 1.309 | 1.336 | 0.340 | 0.444 | 25.14 | 28.21 |
| figure8 | 0.382 | 0.366 | 2.108 | 5.486 | 0.486 | 2.195 | 21.71 | 51.43 |
| aggressive | 0.435 | 0.428 | 2.610 | 2.689 | 0.527 | 0.569 | 27.70 | 30.99 |

Across the four scenarios, mean position RMSE is **0.393 m** for the PX4 body-rate path and **0.385 m** for direct torque (torque-minus-rate: **-0.007 m**). Mean attitude RMSE is **1.519 deg** vs **2.385 deg** (delta **0.866 deg**). Mean body-rate tracking RMSE is **0.340 rad/s** vs **0.804 rad/s** (delta **0.464 rad/s**). Mean motor-saturation incidence from the primary PX4 ULogs is **25.77%** vs **35.65%** (delta **9.88 percentage points**).

## Visualizations

Global comparison figures: `position_rmse.png`, `attitude_rmse.png`, `rate_tracking_rmse.png`, `motor_saturation.png`.

Per-scenario figures and animations generated directly from the real controller CSV / PX4 ULog data:

- `hover`: `plots/hover/trajectory_xy.png`, `trajectory_3d.png`, `position_tracking.png`, `position_error.png`, `attitude_error.png`, `body_rate_tracking.png`, `controller_commands.png`, `motor_outputs.png`, `trajectory.gif`
- `circle`: `plots/circle/trajectory_xy.png`, `trajectory_3d.png`, `position_tracking.png`, `position_error.png`, `attitude_error.png`, `body_rate_tracking.png`, `controller_commands.png`, `motor_outputs.png`, `trajectory.gif`
- `figure8`: `plots/figure8/trajectory_xy.png`, `trajectory_3d.png`, `position_tracking.png`, `position_error.png`, `attitude_error.png`, `body_rate_tracking.png`, `controller_commands.png`, `motor_outputs.png`, `trajectory.gif`
- `aggressive`: `plots/aggressive/trajectory_xy.png`, `trajectory_3d.png`, `position_tracking.png`, `position_error.png`, `attitude_error.png`, `body_rate_tracking.png`, `controller_commands.png`, `motor_outputs.png`, `trajectory.gif`

## Interpretation

For this controller tuning and `x500` SITL setup, direct torque does **not** produce a clear position-tracking advantage over the PX4 body-rate path. The direct-torque path has higher mean attitude error and higher mean body-rate tracking error. The ULog-derived motor saturation metric is the strongest actuator-level warning signal; materially higher saturation for direct torque indicates that torque normalization/allocation tuning should be improved before claiming a direct-torque benefit.

The complete GitHub Actions artifact additionally contains the large PX4 ULogs and PX4/Gazebo logs that are intentionally excluded from Git history.
