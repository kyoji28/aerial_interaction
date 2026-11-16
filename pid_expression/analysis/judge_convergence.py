#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DRAGONのXYステップ応答をROS1 rosbagからOK / NG / 未判定に分類する。

デフォルトのOK条件:
  1. 指令後0～30秒の位置データが取得できる。
  2. 指令後20～30秒の間、X・Yそれぞれの位置誤差が常に±0.2 m以内。
  3. 指令後0～30秒の最大オーバーシュートが200%以内。

使い方:
  python3 judge_convergence.py experiment.bag
  python3 judge_convergence.py experiment.bag --tolerance 0.1
  python3 judge_convergence.py experiment.bag --json

位置誤差はXY合成距離ではなく、各軸の目標位置との差の絶対値で判定する。
"""

import argparse
import bisect
import json
import math
import sys
from pathlib import Path


NAV_TOPIC = "/dragon/uav/nav"
ODOM_TOPIC = "/dragon/uav/cog/odom"


def interpolate_position(samples, t):
    """時系列(t, x, y)から、時刻tの位置を線形補間する。"""
    times = [row[0] for row in samples]
    i = bisect.bisect_left(times, t)

    if i < len(samples) and samples[i][0] == t:
        return samples[i][1:]
    if i == 0 or i == len(samples):
        return None

    t0, x0, y0 = samples[i - 1]
    t1, x1, y1 = samples[i]
    if t1 <= t0:
        return None

    ratio = (t - t0) / (t1 - t0)
    return x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)


def prepare_samples(odom, command_time, settle_limit, duration, max_gap):
    """指令後0～duration秒の位置データを取り出し、必要な時刻を補間する。"""
    relative = sorted((t - command_time, x, y) for t, x, y in odom)
    if not relative or relative[0][0] > 0 or relative[-1][0] < duration:
        return None, f"指令前または指令後{duration:g}秒までの位置データが不足しています"

    start = interpolate_position(relative, 0.0)
    middle = interpolate_position(relative, settle_limit)
    end = interpolate_position(relative, duration)
    if start is None or middle is None or end is None:
        return None, "必要な時刻の位置を補間できません"

    points = [(0.0, *start)]
    points += [(t, x, y) for t, x, y in relative if 0 < t < duration]
    points.append((duration, *end))

    # サンプル間の大きな欠落を、許容範囲内の連続維持と誤認しない。
    if any(b[0] - a[0] > max_gap + 1e-9 for a, b in zip(points, points[1:])):
        return None, f"位置データの間隔が{max_gap:g}秒を超えています"

    points.append((settle_limit, *middle))
    points.sort(key=lambda row: row[0])
    return points, None


def evaluate(odom, command, args):
    """位置系列を判定する。ROSライブラリには依存しない。"""
    command_time, target_x, target_y = command
    result = {
        "判定": "未判定",
        "判定理由": [],
        "目標位置_m": [target_x, target_y],
        "解析区間_s": [0.0, args.duration],
        "収束判定区間_s": [args.settle_limit, args.duration],
    }

    points, reason = prepare_samples(
        odom, command_time, args.settle_limit, args.duration, args.max_gap
    )
    if points is None:
        result["判定理由"] = [reason]
        return result

    if not all(math.isfinite(v) for _, x, y in points for v in (x, y)):
        result["判定理由"] = ["位置データにNaNまたは無限大があります"]
        return result

    start_x, start_y = points[0][1:]
    dx, dy = target_x - start_x, target_y - start_y
    distance = math.hypot(dx, dy)
    result["指令移動量_m"] = distance

    if (
        not math.isfinite(distance)
        or distance == 0
        or abs(distance - args.distance) > args.distance_tolerance
    ):
        result["判定理由"] = [
            f"指令移動量{distance:.3f}mが想定{args.distance:g}±"
            f"{args.distance_tolerance:g}mから外れています"
        ]
        return result

    # XとYを独立に評価（XY合成誤差は使用しない）。
    x_errors = [abs(x - target_x) for _, x, y in points]
    y_errors = [abs(y - target_y) for _, x, y in points]

    # 観測区間の終わりまで、両軸とも許容範囲内に留まる最初の時刻。
    last_outside = max(
        (
            i for i, (ex, ey) in enumerate(zip(x_errors, y_errors))
            if ex > args.tolerance + 1e-12
            or ey > args.tolerance + 1e-12
        ),
        default=-1,
    )
    settle_time = (
        round(points[last_outside + 1][0], 3)
        if last_outside + 1 < len(points) else None
    )

    # オーバーシュートは従来どおり、移動方向への射影で測る。
    ux, uy = dx / distance, dy / distance
    overshoot_m = max(
        0.0,
        max(
            (x - target_x) * ux + (y - target_y) * uy
            for _, x, y in points
        ),
    )
    overshoot_pct = 100 * overshoot_m / args.distance

    # 20～30秒のX・Yそれぞれの最大誤差を計算する。
    window_indices = [
        i for i, (t, _, _) in enumerate(points)
        if t >= args.settle_limit
    ]
    max_x = max(x_errors[i] for i in window_indices)
    max_y = max(y_errors[i] for i in window_indices)

    result.update({
        "各軸の許容位置誤差_m": args.tolerance,
        "収束時刻_s": settle_time,
        "判定区間の最大X誤差_m": max_x,
        "判定区間の最大Y誤差_m": max_y,
        "最終X誤差_m": x_errors[-1],
        "最終Y誤差_m": y_errors[-1],
        "最大オーバーシュート_pct": overshoot_pct,
    })

    problems = []

    if max_x > args.tolerance + 1e-12:
        problems.append(
            f"X方向の最大誤差{max_x:.3f}mが"
            f"許容幅{args.tolerance:.3f}mを超えました"
        )

    if max_y > args.tolerance + 1e-12:
        problems.append(
            f"Y方向の最大誤差{max_y:.3f}mが"
            f"許容幅{args.tolerance:.3f}mを超えました"
        )

    if overshoot_pct > args.overshoot + 1e-12:
        problems.append(
            f"オーバーシュート{overshoot_pct:.2f}%が"
            f"上限{args.overshoot:g}%を超えました"
        )

    result["判定"] = "NG" if problems else "OK"
    result["判定理由"] = problems or [
        "X・Yそれぞれの誤差とオーバーシュートの条件を満たしました"
    ]
    return result


def load_rosbag(path, nav_topic, odom_topic, command_index):
    """ROS1 rosbagからCoG位置と位置指令を取得する。"""
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError(
            "ROS1環境で実行してください（import rosbagが必要）"
        ) from exc

    odom = []
    commands = []

    with rosbag.Bag(str(path), "r") as bag:
        for topic, msg, stamp in bag.read_messages(
            topics=[nav_topic, odom_topic]
        ):
            t = stamp.to_sec()

            if topic == odom_topic:
                p = msg.pose.pose.position
                odom.append((t, float(p.x), float(p.y)))

            elif topic == nav_topic:
                # XY位置指令かつCoGが対象のものだけを採用する。
                if msg.pos_xy_nav_mode in (2, 4) and msg.target == 1:
                    commands.append((
                        t,
                        float(msg.target_pos_x),
                        float(msg.target_pos_y),
                    ))

    if not odom:
        raise RuntimeError(f"位置データがありません: {odom_topic}")
    if not commands:
        raise RuntimeError(f"CoGの位置指令がありません: {nav_topic}")
    if not 1 <= command_index <= len(commands):
        raise RuntimeError(
            f"--command-indexは1～{len(commands)}で指定してください"
        )

    return odom, commands[command_index - 1], len(commands)


def main():
    parser = argparse.ArgumentParser(
        description="DRAGON XY PID応答判定（各軸独立）"
    )

    parser.add_argument(
        "bag", type=Path, help="解析するrosbagファイル"
    )
    parser.add_argument(
        "--duration", type=float, default=30.0,
        help="解析終了時刻 [s]"
    )
    parser.add_argument(
        "--settle-limit", type=float, default=20.0,
        help="これ以降、解析終了まで各軸の許容範囲を維持する [s]"
    )
    parser.add_argument(
        "--distance", type=float, default=1.0,
        help="想定移動距離 [m]"
    )
    parser.add_argument(
        "--distance-tolerance", type=float, default=0.05,
        help="想定移動距離の許容差 [m]"
    )
    parser.add_argument(
        "--tolerance", type=float, default=0.2,
        help="X・Y各軸の許容位置誤差 [m]（既定±0.2 m）"
    )
    parser.add_argument(
        "--overshoot", type=float, default=200.0,
        help="許容オーバーシュート [%%]"
    )
    parser.add_argument(
        "--max-gap", type=float, default=0.2,
        help="位置データの最大許容間隔 [s]"
    )
    parser.add_argument(
        "--command-index", type=int, default=1,
        help="解析対象の位置指令の番号（1始まり）"
    )
    parser.add_argument("--nav-topic", default=NAV_TOPIC)
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument(
        "--json", action="store_true",
        help="JSON形式で出力"
    )

    args = parser.parse_args()

    if not args.bag.is_file():
        parser.error(f"ファイルが見つかりません: {args.bag}")
    if not 0 <= args.settle_limit < args.duration:
        parser.error("0 <= settle-limit < durationにしてください")
    if args.distance <= 0 or args.max_gap <= 0:
        parser.error("distanceとmax-gapは正の数にしてください")
    if min(args.distance_tolerance, args.tolerance, args.overshoot) < 0:
        parser.error("各許容値は0以上にしてください")

    try:
        odom, command, count = load_rosbag(
            args.bag, args.nav_topic, args.odom_topic, args.command_index
        )
        result = evaluate(odom, command, args)
    except Exception as exc:
        print(f"解析できません: {exc}", file=sys.stderr)
        return 2

    result["bag"] = args.bag.name
    result["対象位置指令"] = f"{args.command_index} / {count}"

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print("\n=== DRAGON XY PID 判定（X・Y独立） ===")
        print(f"bag: {args.bag.name}")
        print(f"位置指令: {result['対象位置指令']}")

        if "収束時刻_s" in result:
            print(f"各軸の許容位置誤差: ±{args.tolerance:.3f} m")
            print(f"観測上の収束時刻: {result['収束時刻_s']} 秒")
            print(
                f"{args.settle_limit:g}～{args.duration:g}秒の最大X誤差: "
                f"{result['判定区間の最大X誤差_m']:.3f} m"
            )
            print(
                f"{args.settle_limit:g}～{args.duration:g}秒の最大Y誤差: "
                f"{result['判定区間の最大Y誤差_m']:.3f} m"
            )
            print(
                f"最大オーバーシュート: "
                f"{result['最大オーバーシュート_pct']:.2f}%"
            )

        print(f"判定: {result['判定']}")
        for reason in result["判定理由"]:
            print(f"  - {reason}")

    return {"OK": 0, "NG": 1, "未判定": 2}[result["判定"]]


if __name__ == "__main__":
    sys.exit(main())
