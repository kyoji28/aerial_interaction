#!/usr/bin/env python3
"""DRAGONのCoG XYステップ応答と新しい動作終了判定を比較表示する。"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from judge_convergence import (
    ARRIVAL_LIMIT,
    DURATION,
    HOLD,
    POSITION_TOL,
    find_command,
    judge,
    read_bag,
)


def find_data_dir():
    """ワークスペース内のdata/フォルダを見つける。"""
    for parent in Path(__file__).resolve().parents:
        folder = parent / "data"
        if folder.is_dir():
            return folder
    raise FileNotFoundError("ワークスペースのdata/が見つかりません")


def load_result(path):
    """rosbagを判定し、位置誤差の時系列を返す。"""
    odom, commands = read_bag(path)
    command = find_command(odom, commands)
    status, ts, reason = judge(odom, command)

    t0, target_x, target_y = command
    time = odom[:, 0] - t0
    error_x = odom[:, 1] - target_x
    error_y = odom[:, 2] - target_y

    print(f"{path.name}: {status}" + (f"  Ts={ts:.2f} s" if ts is not None else ""))
    if status == "判定不能":
        print(f"  理由: {reason}")

    return time, error_x, error_y, status, ts


def plot_results(paths, labels, before, output):
    """X・Y位置誤差、20秒の到達期限、10秒維持区間を表示する。"""
    fig, axes = plt.subplots(
        2, 1, figsize=(11, 6), sharex=True, layout="constrained"
    )

    # X・Yの位置許容範囲（速度は終了判定に使用しない）
    for ax in axes:
        ax.axhspan(-POSITION_TOL, POSITION_TOL, color="seagreen", alpha=0.12)
        ax.axhline(0, color="gray", linewidth=0.8)
        ax.axhline(POSITION_TOL, color="seagreen", linestyle="--", linewidth=0.9)
        ax.axhline(-POSITION_TOL, color="seagreen", linestyle="--", linewidth=0.9)

    for path, label in zip(paths, labels):
        time, ex, ey, status, ts = load_result(path)
        mask = (time >= -before) & (time <= DURATION)

        if status == "時間内終了" and ts is not None:
            outcome = f"Finished: Ts={ts:.2f} s"
        elif status == "時間超過":
            outcome = "Timeout"
        else:
            outcome = "Unavailable"

        line, = axes[0].plot(
            time[mask], ex[mask], linewidth=1.5, label=f"{label} / {outcome}"
        )
        color = line.get_color()
        axes[1].plot(time[mask], ey[mask], color=color, linewidth=1.5)

        # Tsから10秒間が位置許容範囲内に収まったことを示す
        if ts is not None:
            for ax in axes:
                ax.axvline(ts, color=color, linestyle=":", linewidth=1.5)
                ax.axvspan(ts, ts + HOLD, color=color, alpha=0.07)

    # 指令時刻、到達期限、10秒維持確認の最終期限
    for ax in axes:
        ax.axvline(0, color="black", linestyle=":", linewidth=1)
        ax.axvline(
            ARRIVAL_LIMIT, color="darkorange", linestyle="--", linewidth=1.2
        )
        ax.axvline(DURATION, color="firebrick", linestyle="--", linewidth=1.2)
        ax.grid(alpha=0.25)

    axes[0].set_ylabel("X position error [m]")
    axes[1].set_ylabel("Y position error [m]")
    axes[1].set_xlabel("Time after movement command [s]")
    axes[1].set_xlim(-before, DURATION + 0.2)
    axes[0].legend(loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=9)

    fig.suptitle(
        "DRAGON CoG XY: Motion Completion "
        f"(position +/-{POSITION_TOL:.2f} m, hold {HOLD:.0f} s)"
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")
    print(f"Saved: {output}")
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path, help="比較するrosbagファイル")
    parser.add_argument("--labels", nargs="+", help="各bagの凡例名")
    parser.add_argument(
        "--before", type=float, default=1.0, help="指令前の表示時間 [s]"
    )
    parser.add_argument(
        "--output", type=Path, default=Path("plot/compare_convergence.png"),
        help="グラフの保存先"
    )
    parser.add_argument(
        "--no-show", action="store_true", help="画面表示せずに保存する"
    )
    args = parser.parse_args()

    if args.labels is not None and len(args.labels) != len(args.bags):
        parser.error("--labelsはbagの数と同じ数を指定してください")
    if args.before < 0:
        parser.error("--beforeは0以上にしてください")

    data_dir = find_data_dir()
    paths = [
        p.expanduser() if p.is_absolute() else data_dir / p
        for p in args.bags
    ]
    for path in paths:
        if not path.is_file():
            parser.error(f"bagが見つかりません: {path}")

    output = (
        args.output.expanduser() if args.output.is_absolute()
        else data_dir / args.output
    )
    labels = args.labels or [p.stem for p in paths]

    fig = plot_results(paths, labels, args.before, output)
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
