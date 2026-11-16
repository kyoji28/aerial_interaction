#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DRAGON CoG XY jerkiness / activity from ROS1 rosbag.

Based on Notion '20261114メモ' (2026-11-16 addendum), and
'20261116メモ': default 100 Hz, Savitzky-Golay (45 samples, degree 3).

Usage (run in a ROS1 environment where 'rosbag' is available):
  python3 analyze_cog_jerk.py data/kp4_ki0_03_kd3_2.bag
  python3 analyze_cog_jerk.py data/*.bag --command-index 0
  python3 analyze_cog_jerk.py data/*.bag --command-time 8.5

--command-index is ZERO-BASED, as in compare_position_xy.py.
--command-time is seconds from first rosbag message, and overrides detection.
The script never guesses t=0 from the beginning of a bag.
"""

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

NAV_TOPIC = "/dragon/uav/nav"
ODOM_TOPIC = "/dragon/uav/cog/odom"


@dataclass
class BagData:
    name: str
    bag_start: float
    odom_t: np.ndarray
    position: np.ndarray       # (N,3), represented in odom.header.frame_id
    velocity: np.ndarray       # (N,3), represented in odom.child_frame_id
    quaternion: np.ndarray     # (N,4): xyzw, maps child to header frame
    nav_t: np.ndarray
    nav_xy: np.ndarray         # (M,2) target XY; NaN for unsupported messages
    pose_frame: str
    twist_frame: str


def nav_target_xy(msg):
    """Read the position target from JSK aerial_robot_msgs/FlightNav, or a Pose."""
    if hasattr(msg, "target_pos_x") and hasattr(msg, "target_pos_y"):
        return float(msg.target_pos_x), float(msg.target_pos_y)
    if hasattr(msg, "target_position") and hasattr(msg.target_position, "x"):
        return float(msg.target_position.x), float(msg.target_position.y)
    if hasattr(msg, "pose"):
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        if hasattr(pose, "position"):
            return float(pose.position.x), float(pose.position.y)
    if hasattr(msg, "point"):
        return float(msg.point.x), float(msg.point.y)
    if hasattr(msg, "position"):
        return float(msg.position.x), float(msg.position.y)
    return None


def load_bag(path):
    # Import here so that mathematical routines can be unit-tested without ROS.
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError(
            "ROS1 'rosbag' が見つかりません。ROS環境をsourceして実行してください。"
        ) from exc

    times, xyz, vel, quat, nav_t, nav_xy = [], [], [], [], [], []
    pose_frames, twist_frames = set(), set()
    first_timestamp = None
    with rosbag.Bag(str(path), "r") as bag:
        for topic, msg, bag_stamp in bag.read_messages(topics=[NAV_TOPIC, ODOM_TOPIC]):
            # A single time source is used for BOTH topics to avoid clock offsets.
            stamp = float(bag_stamp.to_sec())
            if first_timestamp is None:
                first_timestamp = stamp
            if topic == NAV_TOPIC:
                target = nav_target_xy(msg)
                nav_t.append(stamp)
                nav_xy.append(target if target is not None else (np.nan, np.nan))
            elif topic == ODOM_TOPIC:
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                q = msg.pose.pose.orientation
                times.append(stamp)
                xyz.append((p.x, p.y, p.z))
                vel.append((v.x, v.y, v.z))
                quat.append((q.x, q.y, q.z, q.w))
                pose_frames.add(str(msg.header.frame_id))
                twist_frames.add(str(msg.child_frame_id))

    if not times:
        raise ValueError(f"{path}: {ODOM_TOPIC} がありません")
    if len(pose_frames) != 1 or len(twist_frames) != 1:
        raise ValueError(f"{path}: frame_idが試行中に変化しています: {pose_frames}, {twist_frames}")
    if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(vel)):
        raise ValueError(f"{path}: CoG位置または速度にNaN/Infがあります")

    t = np.array(times, dtype=float)
    if np.any(np.diff(t) < 0):
        raise ValueError(f"{path}: 時刻が逆行しています。シミュレーションのリセットに注意")
    # ROS bags may contain duplicate timestamps; retain their last sample.
    ids = np.r_[np.where(np.diff(t) != 0)[0], len(t) - 1]
    t, p, v, q = t[ids], np.asarray(xyz)[ids], np.asarray(vel)[ids], np.asarray(quat)[ids]
    if len(t) < 10:
        raise ValueError(f"{path}: Odometryデータが少なすぎます")
    if not np.all(np.isfinite(q)) or np.any(np.linalg.norm(q, axis=1) < 1e-8):
        raise ValueError(f"{path}: 無効な姿勢クォータニオンがあります")

    return BagData(path.stem, float(first_timestamp), t, p, v, q,
                   np.array(nav_t, dtype=float), np.array(nav_xy, dtype=float).reshape((-1, 2)),
                   next(iter(pose_frames)), next(iter(twist_frames)))


def find_step_commands(data, step_m=1.0, step_tolerance=0.05):
    """Find +X step targets, either from previous nav target or recent CoG pose.

    If only one command exists and no recent pre-command odometry can validate
    the 1-m step, the candidate is rejected; use --command-time instead.
    """
    out = []
    previous = None
    for t, target in zip(data.nav_t, data.nav_xy):
        if not np.all(np.isfinite(target)):
            continue
        if previous is not None and np.linalg.norm(target - previous) < 1e-6:
            continue  # repeatedly published unchanged targets are not commands
        if previous is not None:
            dx, dy = target - previous
        else:
            at = np.searchsorted(data.odom_t, t, side="left") - 1
            if at < 0 or not (0.0 <= t - data.odom_t[at] <= 1.0):
                previous = target.copy()
                continue
            dx, dy = target - data.position[at, :2]
        if abs(dx - step_m) <= step_tolerance and abs(dy) <= step_tolerance:
            out.append(float(t))
        previous = target.copy()
    return out


def rotate_velocity_to_pose_frame(vel, quaternion, mode):
    if mode == "world":
        return vel.copy()
    # nav_msgs/Odometry: twist is conventionally in 'child_frame_id'.
    # Pose quaternion represents the child-frame orientation in header.frame_id.
    norms = np.linalg.norm(quaternion, axis=1)
    return Rotation.from_quat(quaternion / norms[:, None]).apply(vel)


def rms_xy(t, x, y, lo, hi):
    select = (t >= lo - 1e-9) & (t <= hi + 1e-9)
    if np.count_nonzero(select) < 2:
        raise ValueError(f"{lo}～{hi}sにサンプルがありません")
    squared = x[select]**2 + y[select]**2
    integral = np.trapezoid(squared, t[select]) if hasattr(np, "trapezoid") \
        else np.trapz(squared, t[select])
    return float(np.sqrt(integral / (hi - lo)))


def analyze(data, args):
    if args.command_time is not None:
        t0 = data.bag_start + args.command_time
        command_method = f"manual: {args.command_time:g}s after bag start"
    else:
        commands = find_step_commands(data, args.step_m, args.step_tolerance)
        if args.command_index >= len(commands):
            raise ValueError(
                f"+X {args.step_m:g}mの指令が{len(commands)}件しか検出できません"
                f"（要求index={args.command_index}, 0始まり）。"
                "navに目標値が存在するか確認するか、--command-time を指定してください。"
            )
        t0 = commands[args.command_index]
        command_method = f"nav auto-detected, index={args.command_index} (0-based)"

    t_end = t0 + args.duration
    tt = data.odom_t
    dt_native = np.diff(tt)
    # Extend the filter input by 0.5s if odometry before/after the requested
    # interval is available; this reduces edge bias at 0 and 30 seconds.
    dt = 1.0 / args.rate
    n = int(round(args.duration / dt))
    if abs(n * dt - args.duration) > 1e-8:
        raise ValueError("--duration * --rate は整数になるよう指定してください")
    n_pre = min(int(args.padding / dt), max(0, int((t0 - tt[0]) / dt)))
    n_post = min(int(args.padding / dt), max(0, int((tt[-1] - t_end) / dt)))
    relative = np.arange(-n_pre, n + n_post + 1, dtype=float) * dt
    t_query = t0 + relative

    edge0 = max(0, tt[0] - t0)
    edge1 = max(0, t_end - tt[-1])
    if max(edge0, edge1) > args.edge_tolerance:
        raise ValueError(
            f"評価区間の端で記録が不足しています：開始差={edge0:.3f}s,"
            f"終了差={edge1:.3f}s（許容={args.edge_tolerance}s）"
        )
    # Check gaps only in the interval we will interpolate (plus filtering pad).
    near = (tt >= t_query[0] - args.max_gap) & (tt <= t_query[-1] + args.max_gap)
    if np.count_nonzero(near) < 3:
        raise ValueError("評価区間のOdometryデータが不足しています")
    gaps = np.diff(tt[near])
    if len(gaps) and float(np.max(gaps)) > args.max_gap + 1e-9:
        raise ValueError(
            f"Odometryに最大{np.max(gaps):.3f}sの欠損があります"
            f"（上限{args.max_gap}s）。ジャークの推定を中止します。"
        )

    velocity_mode = args.velocity_frame
    if velocity_mode == "auto":
        # If pose and twist share a frame, do not rotate. Otherwise use the
        # Odometry standard: twist in child_frame_id, pose in header.frame_id.
        velocity_mode = "world" if data.pose_frame == data.twist_frame else "body"
    velocity_world = rotate_velocity_to_pose_frame(data.velocity, data.quaternion, velocity_mode)
    vx = np.interp(t_query, tt, velocity_world[:, 0])
    vy = np.interp(t_query, tt, velocity_world[:, 1])
    x = np.interp(t_query, tt, data.position[:, 0])
    y = np.interp(t_query, tt, data.position[:, 1])

    window = args.window
    if window % 2 != 1 or window <= args.polyorder or len(t_query) < window:
        raise ValueError("SG窓長は次数より大きい奇数であり、サンプル数以下にしてください")
    smooth_vx = savgol_filter(vx, window, args.polyorder, deriv=0, delta=dt, mode="interp")
    smooth_vy = savgol_filter(vy, window, args.polyorder, deriv=0, delta=dt, mode="interp")
    ax = savgol_filter(vx, window, args.polyorder, deriv=1, delta=dt, mode="interp")
    ay = savgol_filter(vy, window, args.polyorder, deriv=1, delta=dt, mode="interp")
    jx = savgol_filter(vx, window, args.polyorder, deriv=2, delta=dt, mode="interp")
    jy = savgol_filter(vy, window, args.polyorder, deriv=2, delta=dt, mode="interp")

    sel = (relative >= -1e-9) & (relative <= args.duration + 1e-9)
    series = dict(t=relative[sel], x=x[sel], y=y[sel],
                  vx=vx[sel], vy=vy[sel],
                  vx_smooth=smooth_vx[sel], vy_smooth=smooth_vy[sel],
                  ax=ax[sel], ay=ay[sel], jx=jx[sel], jy=jy[sel])
    series["jerk_xy"] = np.hypot(series["jx"], series["jy"])
    metrics = {
        "bag": data.name,
        "command_time_from_bag_s": t0 - data.bag_start,
        "command_selection": command_method,
        "frame_id": data.pose_frame,
        "child_frame_id": data.twist_frame,
        "velocity_frame_mode": velocity_mode,
        "native_median_dt_s": float(np.median(dt_native)),
        "native_max_gap_in_window_s": float(np.max(gaps)) if len(gaps) else 0,
        "rate_hz": args.rate,
        "sg_window_samples": window,
        "sg_window_nominal_s": window / args.rate,
        "sg_polyorder": args.polyorder,
    }
    for label, lo, hi in [("0_30", 0, args.duration),
                          ("0_10", 0, min(10, args.duration)),
                          ("20_30", 20, args.duration)]:
        if hi <= lo:
            continue
        metrics[f"jerk_rms_{label}_m_s3"] = rms_xy(series["t"], series["jx"], series["jy"], lo, hi)
        # Activity is calculated using raw interpolated (not smoothed) velocities.
        metrics[f"activity_rms_{label}_m_s"] = rms_xy(series["t"], series["vx"], series["vy"], lo, hi)
    return series, metrics


def write_csv(path, cols):
    keys = list(cols.keys())
    size = len(cols[keys[0]])
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(keys)
        for i in range(size):
            writer.writerow([cols[k][i] for k in keys])


def plot_one(path, name, s, rms):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    for ax, key, title in zip(axes, ("jx", "jy", "jerk_xy"),
                              ("CoG X jerk", "CoG Y jerk", "CoG XY jerk magnitude")):
        ax.plot(s["t"], s[key], lw=1.15, label=key)
        ax.set_ylabel("m/s³")
        ax.set_title(title)
        ax.grid(True, alpha=0.25)
    axes[2].axhline(rms, ls="--", lw=1.5, label=f"RMS (0–30 s) = {rms:.4g} m/s³")
    axes[2].legend(loc="upper right")
    axes[2].set_xlabel("Time after command [s]")
    fig.suptitle(f"Jerkiness from CoG XY velocity: {name}")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_overlay(path, trials):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 5.5), constrained_layout=True)
    for label, s, m in trials:
        ax.plot(s["t"], s["jerk_xy"], lw=1.1,
                label=f"{label}: RMS {m['jerk_rms_0_30_m_s3']:.3g} m/s³")
    ax.set(xlabel="Time after command [s]", ylabel="CoG XY jerk magnitude [m/s³]",
           title="CoG XY jerk comparison (same preprocessing)")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+", type=Path, help="ROS1 .bag files (multiple accepted)")
    ap.add_argument("--output-dir", type=Path,
                    default=Path.home() / "ros/human_robot_interaction_ws/data/jerk_results",
                    help="directory for plots and CSVs (default: ~/ros/human_robot_interaction_ws/data/jerk_results)")
    ap.add_argument("--command-index", type=int, default=0, help="0-based +X step command index")
    ap.add_argument("--command-time", type=float, default=None,
                    help="override autodetection: seconds from first rosbag message")
    ap.add_argument("--duration", type=float, default=30.0, help="evaluation duration [s]")
    ap.add_argument("--rate", type=float, default=100.0, help="resampling frequency [Hz]")
    ap.add_argument("--window", type=int, default=45, help="SG window length (odd samples)")
    ap.add_argument("--polyorder", type=int, default=3, help="SG polynomial degree")
    ap.add_argument("--padding", type=float, default=0.5, help="max SG context on each side [s]")
    ap.add_argument("--max-gap", type=float, default=0.2, help="max odom sample interval [s]")
    ap.add_argument("--edge-tolerance", type=float, default=0.05, help="max missing endpoint coverage [s]")
    ap.add_argument("--step-m", type=float, default=1.0, help="expected +X step distance [m]")
    ap.add_argument("--step-tolerance", type=float, default=0.05, help="tolerance in X/Y step detection [m]")
    ap.add_argument("--velocity-frame", choices=("auto", "body", "world"), default="auto",
                    help="auto: infer from frame IDs; body: rotate twist; world: no rotation")
    args = ap.parse_args(argv)
    if args.command_index < 0 or args.rate <= 0 or args.duration <= 0 or args.max_gap <= 0:
        ap.error("command-index >=0、rate/duration/max-gap >0 が必要です")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    trials, rows = [], []
    for i, path in enumerate(args.bags):
        if not path.exists() and not path.is_absolute() and (Path("data") / path).exists():
            path = Path("data") / path
        if not path.exists():
            print(f"[ERROR] bagファイルがありません: {path}", file=sys.stderr)
            continue
        try:
            data = load_bag(path)
            series, metrics = analyze(data, args)
            # Include an index to avoid collisions when different folders have same stem.
            label = f"{i+1:02d}_{data.name}"
            write_csv(args.output_dir / f"timeseries_{label}.csv", series)
            plot_one(args.output_dir / f"jerk_{label}.png", data.name,
                     series, metrics["jerk_rms_0_30_m_s3"])
            trials.append((data.name, series, metrics))
            rows.append(metrics)
            print(f"{data.name}: J_XY RMS = {metrics['jerk_rms_0_30_m_s3']:.6f} m/s³; "
                  f"Activity RMS = {metrics['activity_rms_0_30_m_s']:.6f} m/s; "
                  f"t0={metrics['command_time_from_bag_s']:.3f}s")
            print(f"  pose frame: {metrics['frame_id']!r}, "
                  f"twist frame: {metrics['child_frame_id']!r}, "
                  f"velocity mode: {metrics['velocity_frame_mode']}")
            if metrics["native_median_dt_s"] > 0.05:
                print("  [WARN] Odometryの中央値サンプリング周期が50ms超です。微分精度に注意")
        except Exception as exc:
            print(f"[ERROR] {path}: {exc}", file=sys.stderr)

    if not rows:
        print("有効な試行がありません。", file=sys.stderr)
        return 1
    summary_path = args.output_dir / "metrics.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    if len(trials) > 1:
        plot_overlay(args.output_dir / "jerk_comparison.png", trials)
    print(f"結果を保存しました: {args.output_dir.resolve()}")
    if len(rows) != len(args.bags):
        print("一部のbagの処理に失敗しました（ログを確認してください）。", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
