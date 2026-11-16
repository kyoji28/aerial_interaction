#!/usr/bin/env python3
"""複数のROS1 rosbagの移動指令時刻を揃え、DRAGONの重心位置を軸ごとに比較する。"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag


NAV_TOPIC = "/dragon/uav/nav"
ODOM_TOPIC = "/dragon/uav/cog/odom"
AXES = ("x", "y", "z")

# グラフ上に表示する許容位置誤差（各軸の目標値±この値）
TOLERANCE_M = 0.2
SETTLE_LIMIT_S = 20.0
DURATION_S = 30.0


def find_data_dir():
    """スクリプトの親階層にあるワークスペースのdata/を探す。"""
    for parent in Path(__file__).resolve().parents:
        data_dir = parent / "data"
        if data_dir.is_dir():
            return data_dir
    raise FileNotFoundError("ワークスペースのdata/ディレクトリが見つかりません")


def resolve_bag_path(path, data_dir):
    """相対パスはdata/以下、絶対パスはそのまま使用する。"""
    path = path.expanduser()
    bag_path = path if path.is_absolute() else data_dir / path
    if not bag_path.is_file():
        raise FileNotFoundError(f"bagファイルが見つかりません: {bag_path}")
    return bag_path


def resolve_output_path(path, data_dir):
    """相対パスはdata/以下として扱い、保存先フォルダを作る。"""
    path = path.expanduser()
    output_path = path if path.is_absolute() else data_dir / path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def load_bag(path, command_index):
    """指定した移動指令をt=0として、重心位置と目標位置を読み込む。"""
    commands = []
    samples = []

    with rosbag.Bag(str(path), "r") as bag:
        bag_end_time = bag.get_end_time()
        for topic, msg, bag_time in bag.read_messages(topics=[NAV_TOPIC, ODOM_TOPIC]):
            t = bag_time.to_sec()
            if topic == NAV_TOPIC:
                commands.append((t, msg))
            elif topic == ODOM_TOPIC:
                p = msg.pose.pose.position
                samples.append((t, float(p.x), float(p.y), float(p.z)))

    if not commands:
        raise ValueError(f"{path}: {NAV_TOPIC} が見つかりません")
    if not samples:
        raise ValueError(f"{path}: {ODOM_TOPIC} が見つかりません")
    if command_index >= len(commands):
        raise ValueError(
            f"{path}: 指令は{len(commands)}件です。"
            f"--command-index は0～{len(commands) - 1}で指定してください"
        )

    command_time, command = commands[command_index]
    data = np.asarray(samples, dtype=float)
    data = data[np.argsort(data[:, 0])]
    times = data[:, 0] - command_time
    positions = data[:, 1:4]

    # 指令直前の最後の位置を変位の基準にする。
    before = np.flatnonzero(times <= 0.0)
    base_index = before[-1] if len(before) else np.argmin(np.abs(times))
    baseline = positions[base_index].copy()

    target = np.array(
        [command.target_pos_x, command.target_pos_y, command.target_pos_z],
        dtype=float,
    )
    has_position_target = (
        command.pos_xy_nav_mode in (2, 4),
        command.pos_xy_nav_mode in (2, 4),
        command.pos_z_nav_mode in (2, 4),
    )

    return {
        "time": times,
        "position": positions,
        "baseline": baseline,
        "target": target,
        "has_position_target": has_position_target,
        "command_time": command_time,
        "num_commands": len(commands),
        "end_time": bag_end_time - command_time,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bags", nargs="+", type=Path,
        help="比較するbagファイル（相対パスはdata/基準）",
    )
    parser.add_argument("--labels", nargs="+", help="凡例（bagと同じ数だけ指定）")
    parser.add_argument(
        "--axis", choices=("xy", "x", "y", "z", "xyz"), default="xy",
        help="表示する軸（デフォルト: xy＝X・Yの2段表示）",
    )
    parser.add_argument(
        "--before", type=float, default=1.0,
        help="指令前の表示時間[s]（デフォルト: 1）",
    )
    parser.add_argument(
        "--after", type=float, default=None,
        help="指令後の表示時間[s]（省略時: 最長のbagに揃える）",
    )
    parser.add_argument(
        "--command-index", type=int, default=0,
        help="基準にする指令の番号（0から開始）",
    )
    parser.add_argument(
        "--absolute", action="store_true",
        help="初期位置を引かずに絶対位置を表示",
    )
    parser.add_argument(
        "--no-target", action="store_true",
        help="目標位置線と許容帯を非表示",
    )
    parser.add_argument(
        "--tolerance", type=float, default=TOLERANCE_M,
        help=f"各軸の目標値に対する許容幅[m]（デフォルト: {TOLERANCE_M}）",
    )
    parser.add_argument(
        "--output", type=Path, default=Path("plot/compare_position.png"),
        help="画像の保存先（相対パスはdata/基準）",
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="グラフのウィンドウを開かない",
    )
    args = parser.parse_args()

    if args.labels is not None and len(args.labels) != len(args.bags):
        parser.error("--labelsの数はbagファイル数と一致させてください")
    if args.before < 0:
        parser.error("--beforeは0以上にしてください")
    if args.after is not None and args.after <= 0:
        parser.error("--afterは0より大きくしてください")
    if args.command_index < 0:
        parser.error("--command-indexは0以上にしてください")
    if args.tolerance < 0:
        parser.error("--toleranceは0以上にしてください")

    try:
        data_dir = find_data_dir()
        bag_paths = [resolve_bag_path(path, data_dir) for path in args.bags]
        results = [load_bag(path, args.command_index) for path in bag_paths]
        output_path = resolve_output_path(args.output, data_dir)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    after = max(r["end_time"] for r in results) if args.after is None else args.after
    if after <= 0:
        parser.error("移動指令より後にbagの記録がありません")

    labels = args.labels or [path.stem for path in args.bags]
    components = {"xy": ("x", "y"), "xyz": AXES}.get(args.axis, (args.axis,))

    fig, axes = plt.subplots(
        len(components), 1,
        figsize=(10, 3.1 * len(components) + 1.0),
        sharex=True, squeeze=False,
    )
    axes = axes[:, 0]

    # 灰色の帯は、20～30秒の収束維持判定区間を示す。
    if after > SETTLE_LIMIT_S:
        for ax in axes:
            ax.axvspan(
                SETTLE_LIMIT_S, min(DURATION_S, after),
                color="gray", alpha=0.12,
                label=f"Evaluation: {SETTLE_LIMIT_S:g}-{DURATION_S:g} s",
            )

    for label, result in zip(labels, results):
        t = result["time"]
        position = result["position"].copy()
        target = result["target"].copy()
        baseline = result["baseline"]
        mask = (t >= -args.before) & (t <= after)

        if not args.absolute:
            position -= baseline
            target -= baseline

        print(
            f"{label}: 指令時刻={result['command_time']:.3f}s, "
            f"指令後の記録時間={result['end_time']:.3f}s, "
            f"指令件数={result['num_commands']}, "
            f"初期位置={baseline.round(4)}m"
        )
        if not np.any(mask):
            print(f"WARNING: {label} の表示範囲に位置データがありません")
            continue

        for ax, component in zip(axes, components):
            index = AXES.index(component)

            (line,) = ax.plot(
                t[mask], position[mask, index],
                linewidth=1.8, label=label,
            )

            if args.no_target or not result["has_position_target"][index]:
                continue

            target_value = target[index]
            color = line.get_color()

            ax.axhline(
                target_value, color=color, linestyle="--",
                linewidth=1.2, alpha=0.7,
                label=f"{label} target",
            )

            if component in ("x", "y"):
                ax.axhspan(
                    target_value - args.tolerance,
                    target_value + args.tolerance,
                    color=color, alpha=0.08,
                    label=f"{label} ±{args.tolerance:g} m",
                )

    for ax, component in zip(axes, components):
        ax.axvline(
            0, color="black", linestyle=":", linewidth=1.2,
            label="Move command (t=0)",
        )

        ylabel = "Position" if args.absolute else "Displacement"
        ax.set_ylabel(f"{ylabel} {component.upper()} [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=9)

    axes[-1].set_xlabel("Time from movement command [s]")
    axes[-1].set_xlim(-args.before, after)

    fig.suptitle("DRAGON CoG Step Response Comparison")
    fig.tight_layout()

    fig.savefig(output_path, dpi=200, bbox_inches="tight")

    print(f"表示区間: {-args.before:.2f} ～ {after:.2f} s")
    print(f"Saved: {output_path}")

    if not args.no_show:
        plt.show()

    plt.close(fig)


if __name__ == "__main__":
    main()
