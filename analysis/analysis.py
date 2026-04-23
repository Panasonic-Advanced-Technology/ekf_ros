"""
analysis.py

`ros2 launch ekf_ros gnss_vs_gnss_and_imu.launch.py` の結果 rosbag (rosbag/result)
を読み込み、位置推定の軌跡を PNG で出力する。

- GNSS の ENU 観測 ( /enu_gnss_pose )
- GNSS+IMU 融合の EKF 推定 ( /estimated_pose_fusion )
- GNSS 単独 (IMU入力=0) の EKF 推定 ( /estimated_pose_gnss_only )

使い方:
    python3 analysis.py
    python3 analysis.py --bag /path/to/bagdir --out traj.png
"""

import argparse
import math
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ROS 2 bag 読み込み
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


DEFAULT_BAG_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'rosbag', 'result'
)

TOPIC_FUSION = '/estimated_pose_fusion'
TOPIC_GNSS_ONLY = '/estimated_pose_gnss_only'
TOPIC_ENU_GNSS = '/enu_gnss_pose'


def _quat_to_yaw(q):
    """Quaternion (x,y,z,w) -> yaw[rad] (2D想定)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _open_reader(bag_dir):
    """bag reader を open。metadata.yaml から storage_identifier を読む。"""
    metadata_path = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.exists(metadata_path):
        raise FileNotFoundError(f'metadata.yaml not found in {bag_dir}')

    storage_id = 'mcap'
    with open(metadata_path, 'r') as f:
        for line in f:
            if 'storage_identifier' in line:
                storage_id = line.split(':', 1)[1].strip()
                break

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def read_bag(bag_dir, wanted_topics):
    reader = _open_reader(bag_dir)

    topic_types = reader.get_all_topics_and_types()
    type_map = {}
    for t in topic_types:
        if t.name in wanted_topics:
            type_map[t.name] = get_message(t.type)

    missing = set(wanted_topics) - set(type_map.keys())
    if missing:
        print(f'[warn] topics not found in bag: {missing}', file=sys.stderr)

    storage_filter = rosbag2_py.StorageFilter(topics=list(type_map.keys()))
    reader.set_filter(storage_filter)

    out = {name: [] for name in type_map}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in type_map:
            continue
        msg = deserialize_message(data, type_map[topic])
        out[topic].append((t_ns * 1e-9, msg))
    return out


def pose_records_to_df(records, has_covariance):
    rows = []
    for t_bag, msg in records:
        pose = msg.pose.pose if has_covariance else msg.pose
        stamp = msg.header.stamp
        t_header = stamp.sec + stamp.nanosec * 1e-9
        rows.append({
            'timestamp_s': t_header if t_header > 0 else t_bag,
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': _quat_to_yaw(pose.orientation),
        })
    if not rows:
        return pd.DataFrame(columns=['timestamp_s', 'x', 'y', 'yaw'])
    return pd.DataFrame(rows).sort_values('timestamp_s').reset_index(drop=True)


def plot_trajectory(df_fusion, df_gnss_only, df_enu_gnss, output_png):
    """位置軌跡を matplotlib で描画して PNG 保存"""
    fig, ax = plt.subplots(figsize=(8, 8))

    if len(df_enu_gnss) > 0:
        ax.scatter(df_enu_gnss['x'], df_enu_gnss['y'],
                   s=30, c="#1f77b4", alpha=0.75,
                   label="GNSSの観測位置")
    if len(df_fusion) > 0:
        ax.scatter(df_fusion['x'], df_fusion['y'],
                   s=6, c="#d62728", alpha=0.95,
                   label="GNSS & IMU")
    if len(df_gnss_only) > 0:
        ax.scatter(df_gnss_only['x'], df_gnss_only['y'],
                   s=6, c="green", alpha=0.95,
                   label="GNSS単独")

    ax.set_title("GNSS 単独 vs GNSS & IMU")
    ax.set_xlabel("東 (m)")
    ax.set_ylabel("北 (m)")
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left')

    fig.tight_layout()
    fig.savefig(output_png, dpi=150)
    return fig


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default=DEFAULT_BAG_DIR,
                    help='rosbag2 ディレクトリ (default: ../rosbag/result)')
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'trajectory.png'),
        help='出力PNGファイル')
    ap.add_argument('--no-show', action='store_true',
                    help='保存のみで画像を表示しない')
    return ap.parse_args()


def _try_open_image(path):
    """OS に応じて画像を表示する。失敗してもエラーにしない。"""
    import subprocess
    for cmd in (['xdg-open', path], ['eog', path], ['display', path]):
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            return
        except FileNotFoundError:
            continue


def main():
    args = parse_args()
    bag_dir = os.path.abspath(args.bag)
    print(f'Reading bag: {bag_dir}')

    data = read_bag(bag_dir, [TOPIC_FUSION, TOPIC_GNSS_ONLY, TOPIC_ENU_GNSS])

    df_fusion = pose_records_to_df(
        data.get(TOPIC_FUSION, []), has_covariance=True)
    df_gnss_only = pose_records_to_df(
        data.get(TOPIC_GNSS_ONLY, []), has_covariance=True)
    df_enu_gnss = pose_records_to_df(
        data.get(TOPIC_ENU_GNSS, []), has_covariance=False)

    print(f'  /estimated_pose_fusion    : {len(df_fusion)} msgs')
    print(f'  /estimated_pose_gnss_only : {len(df_gnss_only)} msgs')
    print(f'  /enu_gnss_pose            : {len(df_enu_gnss)} msgs')

    if len(df_fusion) == 0 and len(df_gnss_only) == 0:
        print('[error] EKF 推定結果のトピックが見つかりません。', file=sys.stderr)
        sys.exit(1)

    plot_trajectory(df_fusion, df_gnss_only, df_enu_gnss, args.out)
    print(f'Saved plot: {args.out}')

    if not args.no_show:
        # GUI ウインドウで表示。バックエンドが GUI 不可なら
        # OS ビューアで PNG を開く。
        try:
            plt.show()
        except Exception:
            _try_open_image(args.out)


if __name__ == '__main__':
    main()
