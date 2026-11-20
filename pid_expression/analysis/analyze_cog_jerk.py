#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DRAGON CoG XY: compute RMS jerk during a completed +X 1 m step.

Place this file next to judge_convergence.py in pid_expression/analysis/.
The command selection and motion-end decision use judge_convergence.py.

Examples (ROS1 environment):
  python3 analyze_cog_jerk.py data/run01.bag
  python3 analyze_cog_jerk.py data/*.bag
  python3 analyze_cog_jerk.py data/run01.bag --kp 0.5 --ki 0.03 --kd 0.2
  python3 analyze_cog_jerk.py data/run01.bag --command-index 1
  python3 analyze_cog_jerk.py data/run01.bag --window 61

PID gains are NOT parsed from filenames. Supply --kp/--ki/--kd manually,
or leave these CSV columns blank and enter them later in the spreadsheet.

Output in --output (destination directory):
  metrics.csv                 One row per bag, aligned with the Google Sheet.
  timeseries_XX_*.csv         Time series for inspection (when calculable).
  jerk_XX_*.png               Position error, velocity, and jerk plots.
  jerk_comparison.png         Overlay plot when 2+ trials are calculable.

All positions, velocities, and jerk are treated as WORLD-frame XY values.
No rotation by the CoG quaternion is applied. This follows DRAGON's
state_estimation.cpp implementation; verify consistency with your rosbag.
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter

from judge_convergence import (
    ARRIVAL_LIMIT,
    DURATION,
    HOLD,
    MAX_GAP,
    NAV_TOPIC,
    ODOM_TOPIC,
    POSITION_TOL,
    find_command,
    judge,
    read_bag,
)

OUTPUT_COLUMNS = [
    "試行番号", "Kp", "Ki", "Kd", "終了判定",
    "Ts [s]", "RMS Jerk [m/s³]", "rosbag名",
]


def read_world_velocity(path):
    """Read CoG velocity with the same ROS bag timestamps as read_bag()."""
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("ROS1のrosbagが見つかりません。ROS環境をsourceしてください") from exc

    samples = []
    first_time = None
    pose_frames, twist_frames = set(), set()
    with rosbag.Bag(str(path), "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=[NAV_TOPIC, ODOM_TOPIC]):
            t = float(stamp.to_sec())
            if first_time is None:
                first_time = t
            if topic != ODOM_TOPIC:
                continue
            v = msg.twist.twist.linear
            samples.append((t, float(v.x), float(v.y)))
            pose_frames.add(str(msg.header.frame_id))
            twist_frames.add(str(msg.child_frame_id))

    if len(samples) < 2:
        raise ValueError("CoGのOdometry速度データが不足しています")
    if len(pose_frames) != 1 or len(twist_frames) != 1:
        raise ValueError("Odometryのframe_id / child_frame_idが途中で変化しています")

    data = np.asarray(samples, dtype=float)
    if not np.isfinite(data).all() or np.any(np.diff(data[:, 0]) < 0):
        raise ValueError("Odometry速度にNaN/Infまたは時刻の逆行があります")

    # Match judge_convergence.read_bag(): keep the FIRST duplicate timestamp.
    data = data[np.r_[True, np.diff(data[:, 0]) > 0]]
    return data, first_time, next(iter(pose_frames)), next(iter(twist_frames))


def select_command(odom, commands, index, command_time, bag_start):
    """Select a FlightNav command, reusing judge_convergence.find_command()."""
    if command_time is not None:
        # A manually selected time still needs the matching target from FlightNav.
        t0 = bag_start + command_time
        nearest = min(commands, key=lambda c: abs(c[0] - t0))
        if abs(nearest[0] - t0) > MAX_GAP:
            raise ValueError(
                "--command-time付近に位置指令がありません。"
                "目標座標を推測しないため解析を中止します"
            )
        return (t0, nearest[1], nearest[2])

    # Drop repeated publications of the same target. The judge module's
    # find_command() validates each candidate against recent CoG position.
    distinct = []
    previous_target = None
    for command in commands:
        target = np.asarray(command[1:3], dtype=float)
        if previous_target is not None and np.linalg.norm(target - previous_target) < 1e-6:
            continue
        distinct.append(command)
        previous_target = target

    matched = []
    for command in distinct:
        try:
            matched.append(find_command(odom, [command]))
        except ValueError:
            continue

    if index >= len(matched):
        raise ValueError(
            f"+X約1mの指令は{len(matched)}件です（指定index={index}、0始まり）"
        )
    return matched[index]


def prepare_series(odom, velocity, command, rate, window, polyorder, padding):
    """Resample to equal time steps and derive acceleration/jerk from velocity."""
    t0, target_x, target_y = command
    if len(odom) != len(velocity) or not np.array_equal(odom[:, 0], velocity[:, 0]):
        raise ValueError("位置と速度のタイムスタンプが一致しません")
    if window % 2 != 1 or window <= polyorder or polyorder < 2:
        raise ValueError("SG窓長は次数より大きい奇数、次数は2以上にしてください")

    dt = 1.0 / rate
    total_count = int(round(DURATION * rate))
    if not np.isclose(total_count * dt, DURATION, atol=1e-8):
        raise ValueError("30秒×リサンプリング周波数が整数になる必要があります")

    time_abs = odom[:, 0]
    n_pre = min(int(padding * rate), max(0, int((t0 - time_abs[0]) * rate)))
    n_post = min(int(padding * rate), max(0, int((time_abs[-1] - t0 - DURATION) * rate)))
    relative = np.arange(-n_pre, total_count + n_post + 1) * dt
    query = t0 + relative

    if query[0] < time_abs[0] - 1e-7 or query[-1] > time_abs[-1] + 1e-7:
        raise ValueError("ジャーク計算区間の端でOdometryが不足しています")

    # No interpolation across long gaps. This may be stricter than judge().
    relevant = (time_abs >= query[0] - MAX_GAP) & (time_abs <= query[-1] + MAX_GAP)
    if np.count_nonzero(relevant) < 3 or np.max(np.diff(time_abs[relevant])) > MAX_GAP + 1e-9:
        raise ValueError("Odometryに0.2秒を超える欠損があります")
    if len(query) < window:
        raise ValueError("ジャーク計算用のサンプル数がSG窓長より少ないです")

    x = np.interp(query, time_abs, odom[:, 1])
    y = np.interp(query, time_abs, odom[:, 2])
    vx = np.interp(query, time_abs, velocity[:, 1])  # Already WORLD frame
    vy = np.interp(query, time_abs, velocity[:, 2])

    smooth_vx = savgol_filter(vx, window, polyorder, deriv=0, delta=dt)
    smooth_vy = savgol_filter(vy, window, polyorder, deriv=0, delta=dt)
    jx = savgol_filter(vx, window, polyorder, deriv=2, delta=dt)
    jy = savgol_filter(vy, window, polyorder, deriv=2, delta=dt)

    # Diagnostic only: the derivative of WORLD position should resemble WORLD
    # velocity. Never silently rotate the measured velocity to force a match.
    pos_vx = savgol_filter(x, window, polyorder, deriv=1, delta=dt)
    pos_vy = savgol_filter(y, window, polyorder, deriv=1, delta=dt)
    mask = (relative >= -1e-9) & (relative <= DURATION + 1e-9)
    velocity_error_rms = float(np.sqrt(np.mean(
        (pos_vx[mask] - smooth_vx[mask])**2 +
        (pos_vy[mask] - smooth_vy[mask])**2
    )))

    series = {
        "t_s": relative[mask],
        "error_x_m": x[mask] - target_x,
        "error_y_m": y[mask] - target_y,
        "vx_world_m_s": vx[mask],
        "vy_world_m_s": vy[mask],
        "jx_world_m_s3": jx[mask],
        "jy_world_m_s3": jy[mask],
        "jerk_xy_m_s3": np.hypot(jx[mask], jy[mask]),
    }
    return series, velocity_error_rms


def rms_jerk(series, ts):
    """Time-weighted RMS on the EXACT [0, Ts] interval (end interpolated)."""
    if ts is None or ts <= 0:
        raise ValueError("Tsが0以下、または未定義です")
    t = series["t_s"]
    if ts > t[-1] + 1e-9:
        raise ValueError("Tsを含むJerkの時系列がありません")
    inside = t[(t > 0) & (t < ts)]
    sample_t = np.r_[0.0, inside, ts]
    jx = np.interp(sample_t, t, series["jx_world_m_s3"])
    jy = np.interp(sample_t, t, series["jy_world_m_s3"])
    squared = jx**2 + jy**2
    integral = np.trapezoid(squared, sample_t) if hasattr(np, "trapezoid") \
        else np.trapz(squared, sample_t)
    return float(np.sqrt(integral / ts))


def write_timeseries(path, series):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(series.keys())
        for values in zip(*series.values()):
            writer.writerow(values)


def plot_trial(path, name, series, status, ts, rms):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = series["t_s"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8.5), sharex=True,
                             constrained_layout=True)

    # 1. Position tracking and the ±0.15 m completion band.
    ax = axes[0]
    ax.axhspan(-POSITION_TOL, POSITION_TOL, color="tab:green", alpha=0.10)
    ax.plot(t, series["error_x_m"], label="X position error", lw=1.3)
    ax.plot(t, series["error_y_m"], label="Y position error", lw=1.3)
    ax.set_ylabel("Position error [m]")
    ax.legend(loc="upper right", fontsize=8)

    # 2. WORLD-frame velocities (no body-frame rotation).
    ax = axes[1]
    ax.plot(t, series["vx_world_m_s"], label="vx (world)", lw=1.3)
    ax.plot(t, series["vy_world_m_s"], label="vy (world)", lw=1.3)
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper right", fontsize=8)

    # 3. XY jerk magnitude and the completed-motion RMS.
    ax = axes[2]
    ax.plot(t, series["jerk_xy_m_s3"], lw=1.1, color="tab:purple",
            label="XY jerk magnitude")
    if rms is not None:
        ax.axhline(rms, color="tab:orange", ls="--", lw=1.2,
                   label=f"RMS (0-Ts): {rms:.4g} m/s³")
    ax.set_ylabel("Jerk [m/s³]")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("Time from command [s]")

    for ax in axes:
        ax.axvline(0, color="black", lw=0.9, ls=":")
        if ts is not None:
            ax.axvspan(0, ts, color="tab:green", alpha=0.06)
            ax.axvline(ts, color="tab:green", ls="--", lw=1.3)
        ax.grid(alpha=0.25)
        ax.set_xlim(0, DURATION)
    axes[0].axvline(ARRIVAL_LIMIT, color="tab:orange", ls=":", lw=1)
    if ts is not None:
        axes[0].axvspan(ts, ts + HOLD, color="tab:green", alpha=0.05)

    status_en = {"時間内終了": "Finished", "時間超過": "Timeout",
                 "判定不能": "Unavailable"}.get(status, status)
    fig.suptitle(f"DRAGON CoG XY jerk | {name} | {status_en}"
                 + (f" | Ts={ts:.2f} s" if ts is not None else ""))
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_comparison(path, trials):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    for label, series, ts, rms in trials:
        mask = series["t_s"] <= ts + 1e-9
        ax.plot(series["t_s"][mask], series["jerk_xy_m_s3"][mask],
                lw=1.1, label=f"{label} (RMS={rms:.3g}, Ts={ts:.2f}s)")
    ax.set(xlabel="Time from command [s]", ylabel="XY jerk magnitude [m/s³]",
           title="DRAGON CoG XY jerk: completed motion only")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def write_metrics(path, rows):
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+", type=Path, help="ROS1 rosbag files")
    ap.add_argument("--output", dest="output_dir", type=Path,
                    default=Path.home() / "ros/human_robot_interaction_ws/data/jerk_results")
    ap.add_argument("--command-index", type=int, default=0,
                    help="0-based index among distinct +X 1m commands")
    ap.add_argument("--command-time", type=float,
                    help="manual t0 [s] from first NAV/ODOM bag message; matching NAV required")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--window", type=int, default=45)
    ap.add_argument("--polyorder", type=int, default=3)
    ap.add_argument("--padding", type=float, default=0.5)
    ap.add_argument("--kp", type=float, help="PID P gain for CSV (manual; blank if omitted)")
    ap.add_argument("--ki", type=float, help="PID I gain for CSV (manual; blank if omitted)")
    ap.add_argument("--kd", type=float, help="PID D gain for CSV (manual; blank if omitted)")
    args = ap.parse_args(argv)
    if args.command_index < 0 or args.rate <= 0 or args.padding < 0:
        ap.error("command-index >= 0, rate > 0, padding >= 0 が必要です")
    if args.window % 2 != 1 or args.window <= args.polyorder or args.polyorder < 2:
        ap.error("SG窓長は次数より大きい奇数で、次数は2以上にしてください")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows, completed_trials = [], []
    for index, given_path in enumerate(args.bags, start=1):
        path = given_path.expanduser()
        if not path.exists() and not path.is_absolute() and (Path("data") / path).exists():
            path = Path("data") / path
        label = f"{index:02d}_{path.stem}"
        # Gains are supplied only through CLI flags; never inferred from filenames.
        gains = {"kp": args.kp, "ki": args.ki, "kd": args.kd}

        status, ts, rms, reason = "判定不能", None, None, ""
        warnings = []
        try:
            if not path.is_file():
                raise FileNotFoundError(f"bagファイルがありません: {path}")

            # Use the same start/end criteria as judge_convergence.py.
            odom, commands = read_bag(path)
            velocity, bag_start, pose_frame, twist_frame = read_world_velocity(path)
            command = select_command(odom, commands, args.command_index,
                                     args.command_time, bag_start)
            status, ts, reason = judge(odom, command)

            if status != "判定不能":
                series, velocity_error = prepare_series(
                    odom, velocity, command, args.rate,
                    args.window, args.polyorder, args.padding)
                if status == "時間内終了":
                    rms = rms_jerk(series, ts)
                    completed_trials.append((label, series, ts, rms))
                write_timeseries(args.output_dir / f"timeseries_{label}.csv", series)
                plot_trial(args.output_dir / f"jerk_{label}.png",
                           path.stem, series, status, ts, rms)
                if velocity_error > 0.10:
                    warnings.append(
                        f"位置微分とCoG速度のRMS差={velocity_error:.3f} m/s。"
                        "座標系・推定値を確認"
                    )
        except Exception as exc:
            status, ts, rms, reason = "判定不能", None, None, str(exc)

        rows.append({
            "試行番号": index,
            "Kp": "" if gains["kp"] is None else gains["kp"],
            "Ki": "" if gains["ki"] is None else gains["ki"],
            "Kd": "" if gains["kd"] is None else gains["kd"],
            "終了判定": status,
            "Ts [s]": "" if ts is None else round(ts, 4),
            "RMS Jerk [m/s³]": "" if rms is None else round(rms, 8),
            "rosbag名": path.name,
        })
        # Separate trials with consistent, easy-to-scan log blocks.
        line = "=" * 60
        print(f"\n{line}")
        print(f"[{index}/{len(args.bags)}] {path.name}")
        print("-" * 60)
        print(f"判定     : {status}")
        if status == "時間内終了":
            print(f"Ts       : {ts:.2f} s")
            print(f"RMS Jerk : {rms:.4f} m/s³")
        elif status == "判定不能" and reason:
            print(f"原因     : {reason}")
        for warning in warnings:
            print(f"[WARN]   : {warning}")
        print(line)

    write_metrics(args.output_dir / "metrics.csv", rows)
    if len(completed_trials) > 1:
        plot_comparison(args.output_dir / "jerk_comparison.png", completed_trials)
    counts = {key: sum(row["終了判定"] == key for row in rows)
              for key in ("時間内終了", "時間超過", "判定不能")}
    print("\n" + "=" * 60)
    print(f"解析終了 : 全{len(rows)}試行 | 時間内終了 {counts['時間内終了']} | "
          f"時間超過 {counts['時間超過']} | 判定不能 {counts['判定不能']}")
    print(f"保存先   : {args.output_dir.resolve()}")
    print("=" * 60)
    return 0 if all(row["終了判定"] != "判定不能" for row in rows) else 2


if __name__ == "__main__":
    sys.exit(main())
