#!/usr/bin/env python3
"""複数のDRAGON rosbagを比較し、動作終了条件と終了時刻を可視化する。"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

from judge_convergence import (
    DURATION, HOLD, POSITION_TOL, RATE, SPEED_TOL,
    find_command, judge, read_bag,
)


def find_data_dir():
    """ワークスペース内のdata/フォルダを見つける。"""
    for parent in Path(__file__).resolve().parents:
        folder = parent / "data"
        if folder.is_dir():
            return folder
    raise FileNotFoundError("ワークスペースのdata/が見つかりません")


def load_result(path):
    """動作終了判定とプロット用の時系列データを取得する。"""
    odom, commands = read_bag(path)
    command = find_command(odom, commands)
    status, ts, reason = judge(odom, command)

    t0, target_x, target_y = command

    time = odom[:, 0] - t0
    error_x = odom[:, 1] - target_x
    error_y = odom[:, 2] - target_y

    rms_time, rms = None, None

    if status != "判定不能":
        # 100 Hzでリサンプリング
        grid = np.arange(int(DURATION * RATE) + 1) / RATE

        vx = np.interp(grid, time, odom[:, 3])
        vy = np.interp(grid, time, odom[:, 4])

        # 速度の平滑化
        vx = savgol_filter(vx, 45, 3)
        vy = savgol_filter(vy, 45, 3)

        # XY速度の二乗
        speed_sq = vx**2 + vy**2

        # 台形積分による累積積分
        area = np.r_[
            0,
            np.cumsum(
                (speed_sq[:-1] + speed_sq[1:]) / (2 * RATE)
            ),
        ]

        # 各時刻から先の2秒間の速度RMS
        width = int(HOLD * RATE)

        rms = np.sqrt(
            np.maximum(
                0,
                (area[width:] - area[:-width]) / HOLD,
            )
        )

        rms_time = grid[:len(rms)]

    print(
        f"{path.name}: {status}"
        + (f"  Ts={ts:.2f} s" if ts is not None else "")
    )

    if status == "判定不能":
        print(f"  理由: {reason}")

    return time, error_x, error_y, rms_time, rms, status, ts


def plot_results(paths, labels, before, output):
    """X位置誤差・Y位置誤差・XY速度RMSを比較表示する。"""
    fig, axes = plt.subplots(
        3, 1,
        figsize=(11, 8),
        sharex=True,
        layout="constrained",
    )

    # X・Y位置誤差の許容範囲
    for ax in axes[:2]:
        ax.axhspan(
            -POSITION_TOL,
            POSITION_TOL,
            color="seagreen",
            alpha=0.12,
        )
        ax.axhline(
            0,
            color="gray",
            linewidth=0.8,
        )

    # XY速度RMSの許容範囲
    axes[2].axhspan(
        0,
        SPEED_TOL,
        color="seagreen",
        alpha=0.12,
    )
    axes[2].axhline(
        SPEED_TOL,
        color="seagreen",
        linestyle="--",
        linewidth=1.2,
    )

    for path, label in zip(paths, labels):
        t, ex, ey, tr, rms, status, ts = load_result(path)

        mask = (t >= -before) & (t <= DURATION)

        # 判定結果を凡例に表示
        # ts=Noneの場合に数値フォーマットを使わない
        if status == "時間内終了" and ts is not None:
            status_en = f"Finished: Ts={ts:.2f} s"
        elif status == "時間超過":
            status_en = "Timeout"
        else:
            status_en = "Unavailable"

        # X位置誤差
        line, = axes[0].plot(
            t[mask],
            ex[mask],
            linewidth=1.6,
            label=f"{label} / {status_en}",
        )

        color = line.get_color()

        # Y位置誤差
        axes[1].plot(
            t[mask],
            ey[mask],
            color=color,
            linewidth=1.6,
        )

        # XY速度RMS
        if rms is not None:
            axes[2].plot(
                tr,
                rms,
                color=color,
                linewidth=1.6,
            )

        # 動作終了時刻と2秒間の維持確認区間
        if ts is not None:
            for ax in axes:
                ax.axvline(
                    ts,
                    color=color,
                    linestyle=":",
                    linewidth=1.4,
                )
                ax.axvspan(
                    ts,
                    ts + HOLD,
                    color=color,
                    alpha=0.045,
                )

    # 指令時刻と30秒の制限時間
    for ax in axes:
        ax.axvline(
            0,
            color="black",
            linestyle=":",
            linewidth=1,
        )
        ax.axvline(
            DURATION,
            color="firebrick",
            linestyle="--",
            linewidth=1.2,
        )
        ax.grid(alpha=0.25)

    # 軸ラベル
    axes[0].set_ylabel("X position error [m]")
    axes[1].set_ylabel("Y position error [m]")
    axes[2].set_ylabel(
        "XY speed RMS [m/s]\n(window: t to t+2 s)"
    )
    axes[2].set_xlabel("Time after movement command [s]")
    axes[2].set_xlim(-before, DURATION + 0.2)

    # 凡例
    axes[0].legend(
        loc="upper left",
        bbox_to_anchor=(1.01, 1),
        fontsize=9,
    )

    fig.suptitle(
        "DRAGON CoG XY: Motion Completion (30 s deadline)"
    )

    # グラフを保存
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(
        output,
        dpi=180,
        bbox_inches="tight",
    )

    print(f"Saved: {output}")

    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        "bags",
        nargs="+",
        type=Path,
        help="比較するrosbagファイル",
    )
    parser.add_argument(
        "--labels",
        nargs="+",
        help="各bagの凡例名",
    )
    parser.add_argument(
        "--before",
        type=float,
        default=1.0,
        help="指令前の表示時間 [s]",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("plot/compare_convergence.png"),
        help="グラフの保存先",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="画面表示せずに保存する",
    )

    args = parser.parse_args()

    if args.labels is not None and len(args.labels) != len(args.bags):
        parser.error("--labelsはbagの数と同じ数を指定してください")

    if args.before < 0:
        parser.error("--beforeは0以上にしてください")

    # data/ディレクトリを取得
    data_dir = find_data_dir()

    # bagのパスを解決
    paths = [
        p.expanduser() if p.is_absolute()
        else data_dir / p
        for p in args.bags
    ]

    for path in paths:
        if not path.is_file():
            parser.error(f"bagが見つかりません: {path}")

    # 出力パスを解決
    output = (
        args.output.expanduser()
        if args.output.is_absolute()
        else data_dir / args.output
    )

    labels = args.labels or [p.stem for p in paths]

    # プロット
    fig = plot_results(
        paths,
        labels,
        args.before,
        output,
    )

    if not args.no_show:
        plt.show()

    plt.close(fig)


if __name__ == "__main__":
    main()
