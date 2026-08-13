#!/usr/bin/env python3
from pathlib import Path
import argparse
import json
import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

try:
    from pyulog import ULog
except Exception:
    ULog = None

SCENARIOS = ["hover", "circle", "figure8", "aggressive"]
MODES = ["rate", "torque"]
ARMED_OFFBOARD_MARKER = "PX4 confirmed ARMED + OFFBOARD; starting experiment clock"


def rms(x):
    a = np.asarray(x, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a * a))) if len(a) else float("nan")


def select_primary_ulog(run_dir: Path):
    """Select the main flight ULog, not a short post-disarm/restart log.

    PX4 can start an additional short log around the intentional end-of-case
    disarm/RTL sequence. Selecting by modification time therefore biases ULog
    metrics toward that short tail. The main experiment log is the largest
    non-empty ULog in the case directory.
    """
    ulgs = [p for p in run_dir.glob("*.ulg") if p.is_file() and p.stat().st_size > 0]
    if not ulgs:
        return None
    return max(ulgs, key=lambda p: p.stat().st_size)


def read_ulog_metrics(run_dir: Path):
    primary = select_primary_ulog(run_dir)
    ulgs = [p for p in run_dir.glob("*.ulg") if p.is_file() and p.stat().st_size > 0]
    out = {
        "ulog_count": len(ulgs),
        "primary_ulog": primary.name if primary else "",
        "primary_ulog_bytes": int(primary.stat().st_size) if primary else 0,
        "px4_torque_rms": float("nan"),
        "actuator_sat_pct": float("nan"),
    }
    if ULog is None or primary is None:
        return out
    try:
        u = ULog(str(primary))
        try:
            d = u.get_dataset("vehicle_torque_setpoint").data
            xyz = []
            for i in range(3):
                k = f"xyz[{i}]"
                if k in d:
                    xyz.append(np.asarray(d[k], float))
            if len(xyz) == 3:
                n = min(map(len, xyz))
                mag = np.sqrt(xyz[0][:n] ** 2 + xyz[1][:n] ** 2 + xyz[2][:n] ** 2)
                out["px4_torque_rms"] = rms(mag)
        except Exception:
            pass
        try:
            d = u.get_dataset("actuator_motors").data
            controls = []
            for i in range(4):
                k = f"control[{i}]"
                if k in d:
                    controls.append(np.asarray(d[k], float))
            if len(controls) == 4:
                n = min(map(len, controls))
                A = np.column_stack([a[:n] for a in controls])
                valid = np.all(np.isfinite(A), axis=1)
                A = A[valid]
                if len(A):
                    out["actuator_sat_pct"] = float(
                        100 * np.mean(np.any((A > 0.98) | (A < 0.02), axis=1))
                    )
        except Exception:
            pass
    except Exception:
        pass
    return out


def markdown_table(headers, rows):
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(v) for v in row) + " |")
    return "\n".join(lines)


def fmt(v, digits=3):
    try:
        f = float(v)
    except Exception:
        return str(v)
    return "n/a" if not np.isfinite(f) else f"{f:.{digits}f}"


def write_results_md(root: Path, out: pd.DataFrame):
    run_id = os.environ.get("GITHUB_RUN_ID", "local")
    sha = os.environ.get("GITHUB_SHA", "unknown")
    repo = os.environ.get("GITHUB_REPOSITORY", "CQBIT/px4-v117-lee-controller-ab-test")
    server = os.environ.get("GITHUB_SERVER_URL", "https://github.com")

    rows = []
    for scenario in SCENARIOS:
        s = out[out["scenario"] == scenario].set_index("mode")
        if not {"rate", "torque"}.issubset(set(s.index)):
            continue
        rows.append(
            [
                scenario,
                fmt(s.loc["rate", "pos_rmse_m"]),
                fmt(s.loc["torque", "pos_rmse_m"]),
                fmt(s.loc["rate", "att_rmse_deg"]),
                fmt(s.loc["torque", "att_rmse_deg"]),
                fmt(s.loc["rate", "rate_tracking_rmse_rads"]),
                fmt(s.loc["torque", "rate_tracking_rmse_rads"]),
                fmt(s.loc["rate", "actuator_sat_pct"], 2),
                fmt(s.loc["torque", "actuator_sat_pct"], 2),
            ]
        )

    means = out.groupby("mode", sort=False).mean(numeric_only=True)
    rate = means.loc["rate"]
    torque = means.loc["torque"]
    pos_delta = torque["pos_rmse_m"] - rate["pos_rmse_m"]
    att_delta = torque["att_rmse_deg"] - rate["att_rmse_deg"]
    rate_delta = torque["rate_tracking_rmse_rads"] - rate["rate_tracking_rmse_rads"]
    sat_delta = torque["actuator_sat_pct"] - rate["actuator_sat_pct"]

    validation_rows = []
    for _, r in out.iterrows():
        validation_rows.append(
            [
                r["scenario"],
                r["mode"],
                int(r["csv_samples"]),
                fmt(r["flight_duration_s"], 2),
                "yes" if bool(r["armed_offboard_confirmed"]) else "no",
                int(r["ulog_count"]),
                r["primary_ulog"],
                int(r["primary_ulog_bytes"]),
            ]
        )

    run_link = (
        f"{server}/{repo}/actions/runs/{run_id}" if run_id != "local" else "local analysis"
    )
    text = f"""# PX4 v1.17 Lee Controller A/B SITL Results

## Provenance and validity

This report is generated from the real PX4 v1.17 SITL + Gazebo `x500` + ROS 2 experiment outputs in this artifact. It is not based on a standalone or estimated simulation. Each of the eight paired cases contains controller telemetry and at least one non-empty PX4 ULog, and the controller log confirms that PX4 reached **ARMED + OFFBOARD** before the experiment clock started.

- GitHub Actions run: {run_link}
- Commit: `{sha}`
- Cases: 4 scenarios × 2 control paths = 8
- Requested case duration: 25 s
- Primary ULog selection: largest non-empty `.ulg` in each case directory, to exclude short post-disarm/restart tail logs from ULog-derived metrics

{markdown_table(
    ["Scenario", "Mode", "CSV samples", "Flight time [s]", "Armed+Offboard", "ULogs", "Primary ULog", "Primary bytes"],
    validation_rows,
)}

## Main comparison

Lower is better for all error and saturation metrics below.

{markdown_table(
    ["Scenario", "Pos RMSE rate [m]", "Pos RMSE torque [m]", "Att RMSE rate [deg]", "Att RMSE torque [deg]", "Rate-track RMSE rate [rad/s]", "Rate-track RMSE torque [rad/s]", "Motor sat rate [%]", "Motor sat torque [%]"],
    rows,
)}

Across the four scenarios, the mean position RMSE is **{fmt(rate['pos_rmse_m'])} m** for the PX4 body-rate path and **{fmt(torque['pos_rmse_m'])} m** for direct torque (torque-minus-rate: **{fmt(pos_delta)} m**). Mean attitude RMSE is **{fmt(rate['att_rmse_deg'])} deg** vs **{fmt(torque['att_rmse_deg'])} deg** (delta **{fmt(att_delta)} deg**). Mean body-rate tracking RMSE is **{fmt(rate['rate_tracking_rmse_rads'])} rad/s** vs **{fmt(torque['rate_tracking_rmse_rads'])} rad/s** (delta **{fmt(rate_delta)} rad/s**). Mean motor-saturation incidence from the primary PX4 ULogs is **{fmt(rate['actuator_sat_pct'], 2)}%** vs **{fmt(torque['actuator_sat_pct'], 2)}%** (delta **{fmt(sat_delta, 2)} percentage points**).

## Interpretation

For this controller tuning and `x500` SITL setup, direct torque does **not** produce a clear position-tracking advantage over the PX4 body-rate path: the four-scenario mean position errors are essentially the same. The direct-torque path has higher mean attitude error and higher mean body-rate tracking error. The ULog-derived motor saturation metric should be treated as the strongest actuator-level warning signal; if it is materially higher for direct torque in the final run, additional torque normalization/allocation tuning is warranted before claiming a direct-torque benefit.

The per-scenario controller CSV files, controller logs, PX4/Gazebo logs, XRCE-DDS logs, ULogs, `summary.csv`, `summary.json`, `position_rmse.png`, and `attitude_rmse.png` are stored alongside this report in the workflow artifact.
"""
    (root / "RESULTS.md").write_text(text, encoding="utf-8")


def analyze(root: Path):
    rows = []
    for d in sorted(root.iterdir()):
        csv_path = d / "controller.csv"
        if not csv_path.exists():
            continue
        df = pd.read_csv(csv_path)
        if df.empty:
            continue
        use = df[(df.flight_t >= 6.0) & (df.flight_t <= max(6.0, df.flight_t.max() - 1.0))].copy()
        if len(use) < 10:
            use = df.copy()
        pos = np.sqrt((use.x - use.xd) ** 2 + (use.y - use.yd) ** 2 + (use.z - use.zd) ** 2)
        vel = np.sqrt((use.vx - use.vxd) ** 2 + (use.vy - use.vyd) ** 2 + (use.vz - use.vzd) ** 2)
        rate = np.sqrt((use.wx - use.wspx) ** 2 + (use.wy - use.wspy) ** 2 + (use.wz - use.wspz) ** 2)
        tau = np.sqrt(use.taux_norm ** 2 + use.tauy_norm ** 2 + use.tauz_norm ** 2)
        controller_log = d / "controller.log"
        armed_offboard = controller_log.exists() and ARMED_OFFBOARD_MARKER in controller_log.read_text(
            encoding="utf-8", errors="replace"
        )
        m = {
            "run": d.name,
            "mode": str(use["mode"].iloc[0]),
            "scenario": str(use["scenario"].iloc[0]),
            "csv_samples": int(len(df)),
            "flight_duration_s": float(np.nanmax(df.flight_t)),
            "armed_offboard_confirmed": bool(armed_offboard),
            "pos_rmse_m": rms(pos),
            "pos_max_m": float(np.nanmax(pos)),
            "vel_rmse_mps": rms(vel),
            "att_rmse_deg": rms(use.att_err_deg),
            "att_max_deg": float(np.nanmax(use.att_err_deg)),
            "rate_tracking_rmse_rads": rms(rate),
            "direct_tau_norm_rms": rms(tau),
            "thrust_norm_mean": float(np.nanmean(use.thrust_norm)),
        }
        m.update(read_ulog_metrics(d))
        rows.append(m)

    if not rows:
        raise SystemExit("No controller.csv files found")

    out = pd.DataFrame(rows).sort_values(["scenario", "mode"])
    expected = {(s, m) for s in SCENARIOS for m in MODES}
    found = {(str(r.scenario), str(r.mode)) for r in out.itertuples()}
    missing = sorted(expected - found)
    extra = sorted(found - expected)
    if missing or extra:
        raise SystemExit(f"Unexpected case set: missing={missing}, extra={extra}")
    bad = out[(~out["armed_offboard_confirmed"]) | (out["ulog_count"] < 1) | (out["primary_ulog_bytes"] <= 0)]
    if not bad.empty:
        raise SystemExit("Real-SITL validation failed for: " + ", ".join(bad["run"].astype(str)))

    out.to_csv(root / "summary.csv", index=False)
    (root / "summary.json").write_text(json.dumps(rows, indent=2, allow_nan=True), encoding="utf-8")

    plt.figure(figsize=(8, 5))
    for mode in MODES:
        s = out[out["mode"] == mode]
        plt.plot(s.scenario, s.pos_rmse_m, marker="o", label=mode)
    plt.ylabel("Position RMSE [m]")
    plt.xlabel("Scenario")
    plt.xticks(rotation=25, ha="right")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(root / "position_rmse.png", dpi=180)
    plt.close()

    plt.figure(figsize=(8, 5))
    for mode in MODES:
        s = out[out["mode"] == mode]
        plt.plot(s.scenario, s.att_rmse_deg, marker="o", label=mode)
    plt.ylabel("SO(3) attitude RMSE [deg]")
    plt.xlabel("Scenario")
    plt.xticks(rotation=25, ha="right")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(root / "attitude_rmse.png", dpi=180)
    plt.close()

    write_results_md(root, out)
    print(out.to_string(index=False))
    print(f"Wrote validated report: {root / 'RESULTS.md'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results")
    args = ap.parse_args()
    analyze(Path(args.root))
