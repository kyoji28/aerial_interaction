#!/usr/bin/env python3
"""DRAGONのCoG XYステップ応答の動作終了判定（ROS1 rosbag）。"""

import argparse
from pathlib import Path

import numpy as np

NAV_TOPIC = "/dragon/uav/nav"
ODOM_TOPIC = "/dragon/uav/cog/odom"

POSITION_TOL = 0.15     # X・Y各軸の許容位置誤差 [m]
ARRIVAL_LIMIT = 20.0    # 許容域への到達期限 [s]
HOLD = 10.0             # 位置条件の連続維持時間 [s]
DURATION = 30.0         # 指令後の記録確認時間 [s]
RATE = 100              # 補間周波数 [Hz]
MAX_GAP = 0.20          # Odometryの最大許容欠損 [s]


def read_bag(path):
    """CoGのXY位置と位置指令をrosbagから読み込む。"""
    import rosbag

    odom, commands = [], []

    with rosbag.Bag(str(path), "r") as bag:
        for topic, msg, stamp in bag.read_messages(
            topics=[NAV_TOPIC, ODOM_TOPIC]
        ):
            t = stamp.to_sec()

            if topic == ODOM_TOPIC:
                p = msg.pose.pose.position
                odom.append((t, p.x, p.y))

            elif msg.pos_xy_nav_mode in (2, 4) and msg.target == 1:
                commands.append((t, msg.target_pos_x, msg.target_pos_y))

    if len(odom) < 2 or not commands:
        raise ValueError("OdometryまたはCoGのXY位置指令が不足しています")

    odom = np.asarray(odom, dtype=float)

    if not np.isfinite(odom).all():
        raise ValueError("OdometryにNaN/Infが含まれています")

    if np.any(np.diff(odom[:, 0]) < 0):
        raise ValueError("Odometryの記録時刻が逆行しています")

    # 重複時刻は最初のサンプルだけ残す
    odom = odom[np.r_[True, np.diff(odom[:, 0]) > 0]]

    return odom, commands


def find_command(odom, commands):
    """初期位置から+X方向に約1 m移動する最初の指令を選ぶ。"""
    times = odom[:, 0]

    for t, x_target, y_target in commands:
        i = np.searchsorted(times, t, side="right") - 1

        if i < 0 or t - times[i] > MAX_GAP:
            continue

        x0, y0 = odom[i, 1:3]

        if (
            abs(x_target - x0 - 1.0) <= 0.05
            and abs(y_target - y0) <= 0.05
        ):
            return t, x_target, y_target

    raise ValueError("+X方向約1 mのCoG位置指令が見つかりません")


def judge(odom, command):
    """(判定, 終了時刻Ts, 理由)を返す。Tsは指令からの経過秒数。"""
    t0, x_target, y_target = command
    times = odom[:, 0]
    end = t0 + DURATION

    # 1. 指令後30秒間の記録とデータ欠損を確認
    if times[0] > t0 or times[-1] < end:
        return "判定不能", None, "指令後30秒までの記録がありません"

    start = np.searchsorted(times, t0, side="right") - 1
    stop = np.searchsorted(times, end, side="left") + 1

    if np.any(np.diff(times[start:stop]) > MAX_GAP):
        return "判定不能", None, "Odometryに0.2秒を超える欠損があります"

    # 2. 指令時刻を0秒としてXY位置を100 Hzで補間
    t = np.arange(int(DURATION * RATE) + 1) / RATE
    x = np.interp(t0 + t, times, odom[:, 1])
    y = np.interp(t0 + t, times, odom[:, 2])

    position_ok = (
        (np.abs(x - x_target) <= POSITION_TOL)
        & (np.abs(y - y_target) <= POSITION_TOL)
    )

    # 3. 各候補時刻から10秒間、位置条件を一度も破らないか確認
    # 累積和により、各10秒窓内の条件違反サンプル数を求める
    failed = (~position_ok).astype(int)
    cumulative = np.r_[0, np.cumsum(failed)]
    window_size = int(HOLD * RATE) + 1  # 始点・終点を含む
    violations = cumulative[window_size:] - cumulative[:-window_size]

    # 4. 開始時刻が20秒以内にある最初の有効窓を採用
    last_start = int(ARRIVAL_LIMIT * RATE)
    candidates = np.flatnonzero(violations[:last_start + 1] == 0)

    if len(candidates) == 0:
        return (
            "時間超過", None,
            "20秒以内に到達して10秒間維持する条件を満たしませんでした"
        )

    return "時間内終了", float(t[candidates[0]]), "位置条件を10秒間維持しました"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bags", nargs="+", type=Path,
        help="解析するbagファイル（複数指定可）"
    )
    args = parser.parse_args()

    for path in args.bags:
        try:
            odom, commands = read_bag(path)
            command = find_command(odom, commands)
            status, ts, reason = judge(odom, command)
        except Exception as exc:
            status, ts, reason = "判定不能", None, str(exc)

        print(f"\n=== {path.name} ===")
        print(f"判定: {status}")
        if ts is not None:
            print(f"動作終了時刻 Ts: {ts:.2f} s")
        print(f"理由: {reason}")


if __name__ == "__main__":
    main()
