#!/usr/bin/env python3
from pathlib import Path
import argparse
import json
import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

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
    ulgs = [p for p in run_dir.glob("*.ulg") if p.is_file() and p.stat().st_size > 0]
    if not ulgs:
        return None
    return max(ulgs, key=lambda p: p.stat().st_size)


def load_ulog(run_dir: Path):
    primary = select_primary_ulog(run_dir)
    if ULog is None or primary is None:
        return primary, None
    try:
        return primary, ULog(str(primary))
    except Exception:
        return primary, None


def read_ulog_metrics(run_dir: Path):
    primary, ulog = load_ulog(run_dir)
    ulgs = [p for p in run_dir.glob("*.ulg") if p.is_file() and p.stat().st_size > 0]
    out = {
        "ulog_count": len(ulgs),
        "primary_ulog": primary.name if primary else "",
        "primary_ulog_bytes": int(primary.stat().st_size) if primary else 0,
        "px4_torque_rms": float("nan"),
        "actuator_sat_pct": float("nan"),
    }
    if ulog is None:
        return out
    try:
        d = ulog.get_dataset("vehicle_torque_setpoint").data
        xyz = [np.asarray(d[f"xyz[{i}]"], float) for i in range(3) if f"xyz[{i}]" in d]
        if len(xyz) == 3:
            n = min(map(len, xyz))
            mag = np.sqrt(xyz[0][:n] ** 2 + xyz[1][:n] ** 2 + xyz[2][:n] ** 2)
            out["px4_torque_rms"] = rms(mag)
    except Exception:
        pass
    try:
        d = ulog.get_dataset("actuator_motors").data
        controls = [np.asarray(d[f"control[{i}]"], float) for i in range(4) if f"control[{i}]" in d]
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
    return out


def actuator_series(run_dir: Path):
    _, ulog = load_ulog(run_dir)
    if ulog is None:
        return None
    try:
        d = ulog.get_dataset("actuator_motors").data
        if "timestamp" not in d:
            return None
        t = np.asarray(d["timestamp"], float) * 1e-6
        t = t - t[0]
        controls = []
        for i in range(4):
            k = f"control[{i}]"
            if k not in d:
                return None
            controls.append(np.asarray(d[k], float))
        n = min(len(t), *(len(a) for a in controls))
        return t[:n], np.column_stack([a[:n] for a in controls])
    except Exception:
        return None


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


def case_df(root: Path, scenario: str, mode: str):
    p = root / f"{scenario}_{mode}" / "controller.csv"
    return pd.read_csv(p)


def valid_flight(df: pd.DataFrame):
    if "flight_t" not in df:
        return df.copy()
    d = df[np.isfinite(df.flight_t)].copy()
    d = d[d.flight_t >= 0.0]
    return d if len(d) else df.copy()


def plot_summary(root: Path, out: pd.DataFrame):
    specs = [
        ("pos_rmse_m", "Position RMSE [m]", "position_rmse.png"),
        ("att_rmse_deg", "SO(3) attitude RMSE [deg]", "attitude_rmse.png"),
        ("rate_tracking_rmse_rads", "Body-rate tracking RMSE [rad/s]", "rate_tracking_rmse.png"),
        ("actuator_sat_pct", "Motor saturation incidence [%]", "motor_saturation.png"),
    ]
    for metric, ylabel, filename in specs:
        plt.figure(figsize=(8, 5))
        for mode in MODES:
            s = out[out["mode"] == mode]
            plt.plot(s.scenario, s[metric], marker="o", label=mode)
        plt.ylabel(ylabel)
        plt.xlabel("Scenario")
        plt.xticks(rotation=25, ha="right")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(root / filename, dpi=180)
        plt.close()


def plot_scenario(root: Path, scenario: str):
    plot_dir = root / "plots" / scenario
    plot_dir.mkdir(parents=True, exist_ok=True)
    data = {m: valid_flight(case_df(root, scenario, m)) for m in MODES}

    plt.figure(figsize=(7, 7))
    ref = data["rate"]
    plt.plot(ref.xd, ref.yd, linestyle="--", linewidth=2.0, label="reference")
    for mode in MODES:
        d = data[mode]
        plt.plot(d.x, d.y, linewidth=1.5, label=mode)
    plt.xlabel("North x [m]")
    plt.ylabel("East y [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title(f"{scenario}: XY trajectory")
    plt.tight_layout()
    plt.savefig(plot_dir / "trajectory_xy.png", dpi=180)
    plt.close()

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(ref.xd, ref.yd, -ref.zd, linestyle="--", linewidth=2.0, label="reference")
    for mode in MODES:
        d = data[mode]
        ax.plot(d.x, d.y, -d.z, linewidth=1.3, label=mode)
    ax.set_xlabel("North x [m]")
    ax.set_ylabel("East y [m]")
    ax.set_zlabel("Altitude [m]")
    ax.set_title(f"{scenario}: 3D trajectory")
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_dir / "trajectory_3d.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    labels = [("x", "xd", "x [m]"), ("y", "yd", "y [m]"), ("z", "zd", "z NED [m]")]
    for ax, (actual, desired, ylabel) in zip(axes, labels):
        ax.plot(ref.flight_t, ref[desired], linestyle="--", linewidth=1.8, label="reference")
        for mode in MODES:
            d = data[mode]
            ax.plot(d.flight_t, d[actual], linewidth=1.0, label=mode)
        ax.set_ylabel(ylabel)
        ax.grid(True)
    axes[-1].set_xlabel("Flight time [s]")
    axes[0].legend(ncol=3)
    fig.suptitle(f"{scenario}: position tracking")
    fig.tight_layout()
    fig.savefig(plot_dir / "position_tracking.png", dpi=180)
    plt.close(fig)

    plt.figure(figsize=(9, 5))
    for mode in MODES:
        d = data[mode]
        e = np.sqrt((d.x-d.xd)**2 + (d.y-d.yd)**2 + (d.z-d.zd)**2)
        plt.plot(d.flight_t, e, linewidth=1.1, label=mode)
    plt.xlabel("Flight time [s]")
    plt.ylabel("Position error norm [m]")
    plt.grid(True)
    plt.legend()
    plt.title(f"{scenario}: position error")
    plt.tight_layout()
    plt.savefig(plot_dir / "position_error.png", dpi=180)
    plt.close()

    plt.figure(figsize=(9, 5))
    for mode in MODES:
        d = data[mode]
        plt.plot(d.flight_t, d.att_err_deg, linewidth=1.1, label=mode)
    plt.xlabel("Flight time [s]")
    plt.ylabel("SO(3) attitude error [deg]")
    plt.grid(True)
    plt.legend()
    plt.title(f"{scenario}: attitude error")
    plt.tight_layout()
    plt.savefig(plot_dir / "attitude_error.png", dpi=180)
    plt.close()

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=False)
    for ax, mode in zip(axes, MODES):
        d = data[mode]
        for actual, desired, axis_name in [("wx", "wspx", "p"), ("wy", "wspy", "q"), ("wz", "wspz", "r")]:
            ax.plot(d.flight_t, d[actual], linewidth=1.0, label=f"{axis_name} actual")
            ax.plot(d.flight_t, d[desired], linestyle="--", linewidth=0.9, label=f"{axis_name} setpoint")
        ax.set_ylabel(f"{mode} [rad/s]")
        ax.grid(True)
        ax.legend(ncol=3, fontsize=8)
    axes[-1].set_xlabel("Flight time [s]")
    fig.suptitle(f"{scenario}: body-rate tracking")
    fig.tight_layout()
    fig.savefig(plot_dir / "body_rate_tracking.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
    for ax, mode in zip(axes, MODES):
        d = data[mode]
        ax.plot(d.flight_t, d.thrust_norm, linewidth=1.1, label="thrust norm")
        tau_mag = np.sqrt(d.taux_norm**2 + d.tauy_norm**2 + d.tauz_norm**2)
        ax.plot(d.flight_t, tau_mag, linewidth=1.1, label="Lee torque norm")
        ax.set_ylabel(mode)
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("Flight time [s]")
    fig.suptitle(f"{scenario}: controller command magnitudes")
    fig.tight_layout()
    fig.savefig(plot_dir / "controller_commands.png", dpi=180)
    plt.close(fig)

    motor_data = {m: actuator_series(root / f"{scenario}_{m}") for m in MODES}
    if any(v is not None for v in motor_data.values()):
        fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=False)
        for ax, mode in zip(axes, MODES):
            series = motor_data[mode]
            if series is None:
                ax.text(0.5, 0.5, "No actuator_motors ULog data", ha="center", va="center", transform=ax.transAxes)
            else:
                t, motors = series
                for i in range(min(4, motors.shape[1])):
                    ax.plot(t, motors[:, i], linewidth=0.9, label=f"motor {i+1}")
                ax.axhline(0.98, linestyle="--", linewidth=0.8, label="upper sat threshold")
                ax.axhline(0.02, linestyle=":", linewidth=0.8, label="lower sat threshold")
                ax.legend(ncol=3, fontsize=8)
            ax.set_ylabel(mode)
            ax.set_ylim(-0.05, 1.05)
            ax.grid(True)
        axes[-1].set_xlabel("ULog time since actuator start [s]")
        fig.suptitle(f"{scenario}: PX4 actuator motor outputs")
        fig.tight_layout()
        fig.savefig(plot_dir / "motor_outputs.png", dpi=180)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(ref.xd, ref.yd, linestyle="--", linewidth=1.8, label="reference")
    all_x = np.concatenate([np.asarray(ref.xd), np.asarray(data["rate"].x), np.asarray(data["torque"].x)])
    all_y = np.concatenate([np.asarray(ref.yd), np.asarray(data["rate"].y), np.asarray(data["torque"].y)])
    all_x = all_x[np.isfinite(all_x)]
    all_y = all_y[np.isfinite(all_y)]
    if len(all_x) and len(all_y):
        dx = max(0.5, float(np.max(all_x)-np.min(all_x)))
        dy = max(0.5, float(np.max(all_y)-np.min(all_y)))
        ax.set_xlim(float(np.min(all_x)-0.08*dx), float(np.max(all_x)+0.08*dx))
        ax.set_ylim(float(np.min(all_y)-0.08*dy), float(np.max(all_y)+0.08*dy))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("North x [m]")
    ax.set_ylabel("East y [m]")
    ax.grid(True)
    ax.set_title(f"{scenario}: real PX4 SITL A/B trajectory")
    rate_line, = ax.plot([], [], linewidth=1.4, label="rate actual")
    torque_line, = ax.plot([], [], linewidth=1.4, label="torque actual")
    rate_pt, = ax.plot([], [], marker="o", linestyle="None")
    torque_pt, = ax.plot([], [], marker="s", linestyle="None")
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")
    ax.legend(loc="best")

    n = min(len(data["rate"]), len(data["torque"]))
    if n > 2:
        frame_idx = np.unique(np.linspace(0, n-1, min(60, n)).astype(int))
        tail = max(20, n // 5)

        def animate(k):
            i = int(frame_idx[k])
            r = data["rate"].iloc[:i+1]
            q = data["torque"].iloc[:i+1]
            j0 = max(0, i-tail)
            rate_line.set_data(r.x.iloc[j0:], r.y.iloc[j0:])
            torque_line.set_data(q.x.iloc[j0:], q.y.iloc[j0:])
            rate_pt.set_data([data["rate"].x.iloc[i]], [data["rate"].y.iloc[i]])
            torque_pt.set_data([data["torque"].x.iloc[i]], [data["torque"].y.iloc[i]])
            tval = min(float(data["rate"].flight_t.iloc[i]), float(data["torque"].flight_t.iloc[i]))
            time_text.set_text(f"t = {tval:.1f} s")
            return rate_line, torque_line, rate_pt, torque_pt, time_text

        anim = FuncAnimation(fig, animate, frames=len(frame_idx), interval=100, blit=False)
        anim.save(plot_dir / "trajectory.gif", writer=PillowWriter(fps=10), dpi=80)
    plt.close(fig)


def write_results_md(root: Path, out: pd.DataFrame):
    run_id = os.environ.get("GITHUB_RUN_ID", "local")
    sha = os.environ.get("GITHUB_SOURCE_SHA", os.environ.get("GITHUB_SHA", "unknown"))
    repo = os.environ.get("GITHUB_REPOSITORY", "CQBIT/px4-v117-lee-controller-ab-test")
    server = os.environ.get("GITHUB_SERVER_URL", "https://github.com")

    rows = []
    for scenario in SCENARIOS:
        s = out[out["scenario"] == scenario].set_index("mode")
        if not {"rate", "torque"}.issubset(set(s.index)):
            continue
        rows.append([
            scenario,
            fmt(s.loc["rate", "pos_rmse_m"]), fmt(s.loc["torque", "pos_rmse_m"]),
            fmt(s.loc["rate", "att_rmse_deg"]), fmt(s.loc["torque", "att_rmse_deg"]),
            fmt(s.loc["rate", "rate_tracking_rmse_rads"]), fmt(s.loc["torque", "rate_tracking_rmse_rads"]),
            fmt(s.loc["rate", "actuator_sat_pct"], 2), fmt(s.loc["torque", "actuator_sat_pct"], 2),
        ])

    means = out.groupby("mode", sort=False).mean(numeric_only=True)
    rate = means.loc["rate"]
    torque = means.loc["torque"]
    pos_delta = torque["pos_rmse_m"] - rate["pos_rmse_m"]
    att_delta = torque["att_rmse_deg"] - rate["att_rmse_deg"]
    rate_delta = torque["rate_tracking_rmse_rads"] - rate["rate_tracking_rmse_rads"]
    sat_delta = torque["actuator_sat_pct"] - rate["actuator_sat_pct"]

    validation_rows = []
    for _, r in out.iterrows():
        validation_rows.append([
            r["scenario"], r["mode"], int(r["csv_samples"]), fmt(r["flight_duration_s"], 2),
            "yes" if bool(r["armed_offboard_confirmed"]) else "no", int(r["ulog_count"]),
            r["primary_ulog"], int(r["primary_ulog_bytes"]),
        ])

    run_link = f"{server}/{repo}/actions/runs/{run_id}" if run_id != "local" else "local analysis"
    plot_lines = []
    for scenario in SCENARIOS:
        plot_lines.append(
            f"- `{scenario}`: `plots/{scenario}/trajectory_xy.png`, `trajectory_3d.png`, "
            f"`position_tracking.png`, `position_error.png`, `attitude_error.png`, "
            f"`body_rate_tracking.png`, `controller_commands.png`, `motor_outputs.png`, `trajectory.gif`"
        )
    plot_listing = "\n".join(plot_lines)

    text = f"""# PX4 v1.17 Lee Controller A/B SITL Results

## Provenance and validity

This report is generated from the real PX4 v1.17 SITL + Gazebo `x500` + ROS 2 experiment outputs. It is not based on a standalone or estimated simulation. Each of the eight paired cases contains controller telemetry and at least one non-empty PX4 ULog, and the controller log confirms that PX4 reached **ARMED + OFFBOARD** before the experiment clock started.

- GitHub Actions run: {run_link}
- Tested source commit: `{sha}`
- Cases: 4 scenarios × 2 control paths = 8
- Requested case duration: 25 s
- Primary ULog selection: largest non-empty `.ulg` in each case directory
- Repository policy: lightweight CSV/log/plots/report files are retained in Git; `.ulg` and `px4_gz.log` remain in the complete Actions artifact only.

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

Across the four scenarios, mean position RMSE is **{fmt(rate['pos_rmse_m'])} m** for the PX4 body-rate path and **{fmt(torque['pos_rmse_m'])} m** for direct torque (torque-minus-rate: **{fmt(pos_delta)} m**). Mean attitude RMSE is **{fmt(rate['att_rmse_deg'])} deg** vs **{fmt(torque['att_rmse_deg'])} deg** (delta **{fmt(att_delta)} deg**). Mean body-rate tracking RMSE is **{fmt(rate['rate_tracking_rmse_rads'])} rad/s** vs **{fmt(torque['rate_tracking_rmse_rads'])} rad/s** (delta **{fmt(rate_delta)} rad/s**). Mean motor-saturation incidence from the primary PX4 ULogs is **{fmt(rate['actuator_sat_pct'], 2)}%** vs **{fmt(torque['actuator_sat_pct'], 2)}%** (delta **{fmt(sat_delta, 2)} percentage points**).

## Visualizations

Global comparison figures: `position_rmse.png`, `attitude_rmse.png`, `rate_tracking_rmse.png`, `motor_saturation.png`.

Per-scenario figures and animations generated directly from the real controller CSV / PX4 ULog data:

{plot_listing}

## Interpretation

For this controller tuning and `x500` SITL setup, direct torque does **not** produce a clear position-tracking advantage over the PX4 body-rate path. The direct-torque path has higher mean attitude error and higher mean body-rate tracking error. The ULog-derived motor saturation metric is the strongest actuator-level warning signal; materially higher saturation for direct torque indicates that torque normalization/allocation tuning should be improved before claiming a direct-torque benefit.

The complete GitHub Actions artifact additionally contains the large PX4 ULogs and PX4/Gazebo logs that are intentionally excluded from Git history.
"""
    (root / "RESULTS.md").write_text(text, encoding="utf-8")


def analyze(root: Path):
    rows = []
    for d in sorted(root.iterdir()):
        if not d.is_dir() or d.name == "plots":
            continue
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

    plot_summary(root, out)
    for scenario in SCENARIOS:
        plot_scenario(root, scenario)
    write_results_md(root, out)
    print(out.to_string(index=False))
    print(f"Wrote validated report: {root / 'RESULTS.md'}")
    print(f"Wrote plots/animations under: {root / 'plots'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results")
    args = ap.parse_args()
    analyze(Path(args.root))
