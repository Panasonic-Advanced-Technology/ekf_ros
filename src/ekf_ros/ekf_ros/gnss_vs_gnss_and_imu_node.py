"""
gnss_vs_gnss_and_imu_node.py

GNSS単独(IMU入力=0)の EKF と GNSS+IMU 融合の EKF を同時に走らせて
位置推定結果を比較するROS2ノード。

入力:
  /imu   (sensor_msgs/Imu)
  /gnss  (sensor_msgs/NavSatFix)
  /vel   (geometry_msgs/TwistStamped)

`sim_view.ipynb` の
    t_block_start = float(t[55])
    t_block_end   = t_block_start + 1.00
に相当する GNSS 欠損区間では両EKFで correct() を呼ばない。

観測ベクトル: z = [px, py, yaw, vx, vy]
  - px, py  : GNSS を ENU 変換した位置
  - yaw     : 最新 /vel の arctan2(vy, vx)（低速時は状態 yaw）
  - vx, vy  : 最新 /vel
"""

import os
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped, TwistStamped,
)
from sensor_msgs.msg import Imu, NavSatFix
from ament_index_python.packages import get_package_share_directory

from ekf_ros.ekf import State2D, EKF
from ekf_ros.math_tools import lla_to_enu
from ekf_ros.msg_utils import (
    get_timestamp,
    extract_imu_data_2d,
    make_posestamped_msg_2d,
    make_posestamped_cov_msg_2d,
)
from ekf_ros.load_config import load_gnss_vs_gnss_and_imu_config


class GNSSvsGNSSAndIMU(Node):
    """GNSS単独とGNSS+IMU融合の推定結果を比較するノード"""

    def __init__(self):
        super().__init__('gnss_vs_gnss_and_imu_node')

        # パラメータ
        self.declare_parameter('speed_threshold', 0.3)
        self.declare_parameter('gnss_block_start_index', 55)
        self.declare_parameter('gnss_block_duration', 1.0)
        self.declare_parameter('config_file', '')

        self._speed_threshold = self.get_parameter(
            'speed_threshold').get_parameter_value().double_value
        self._block_start_index = self.get_parameter(
            'gnss_block_start_index').get_parameter_value().integer_value
        self._block_duration = self.get_parameter(
            'gnss_block_duration').get_parameter_value().double_value
        config_file_param = self.get_parameter(
            'config_file').get_parameter_value().string_value

        # 設定ファイル
        share_dir = get_package_share_directory('ekf_ros')
        config_path = config_file_param or os.path.join(
            share_dir, 'config', 'gnss_vs_gnss_and_imu.yaml')
        self._config = load_gnss_vs_gnss_and_imu_config(config_path)
        self.get_logger().info(f'Loaded config: {config_path}')

        # EKF
        self._ekf_fusion = None
        self._ekf_gnss_only = None
        self._initialized = False

        # IMU
        self._first_imu = True
        self._prev_imu_timestamp = None

        # 最新 /vel キャッシュ
        self._latest_vel = None

        # GNSS 欠損管理
        self._gnss_count = 0
        self._block_start_time = None

        # 原点
        self._origin_lat = 0.0
        self._origin_lon = 0.0
        self._origin_alt = 0.0

        self._init_publishers()
        self._init_subscriptions()

        self.get_logger().info(
            f'gnss_vs_gnss_and_imu_node initialized '
            f'(block_start_index={self._block_start_index}, '
            f'block_duration={self._block_duration}s)'
        )

    # =========================================================================
    def _init_publishers(self):
        self._fusion_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/estimated_pose_fusion', 2)
        self._gnss_only_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/estimated_pose_gnss_only', 2)
        self._enu_gnss_pose_pub = self.create_publisher(
            PoseStamped, '/enu_gnss_pose', 2)

    def _init_subscriptions(self):
        self._imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, 1)
        self._gnss_sub = self.create_subscription(
            NavSatFix, '/gnss', self._gnss_callback, 1)
        self._vel_sub = self.create_subscription(
            TwistStamped, '/vel', self._vel_callback, 1)

    # =========================================================================
    # /vel
    # =========================================================================
    def _vel_callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        # NaN は 0 扱い（CSV 処理時の仕様と揃える）
        if not np.isfinite(vx):
            vx = 0.0
        if not np.isfinite(vy):
            vy = 0.0
        self._latest_vel = np.array([vx, vy], dtype=float)

    # =========================================================================
    # 初期化
    # =========================================================================
    def _initialize_from_gnss(self, msg):
        self._origin_lat = msg.latitude
        self._origin_lon = msg.longitude
        self._origin_alt = msg.altitude

        init = self._config.get('initial_state') or {}
        init_yaw = float(init.get('yaw', 0.0))
        init_ba = np.array(init.get('accel_bias', [0.0, 0.0]), dtype=float)
        init_bg = float(init.get('gyro_bias', 0.0))

        init_vel = (self._latest_vel.copy()
                    if self._latest_vel is not None
                    else np.array([0.0, 0.0]))

        initial_state = State2D(
            pos=np.array([0.0, 0.0]),
            yaw=np.array([init_yaw]),
            vel=init_vel,
            ba=init_ba,
            bg=np.array([init_bg]),
        )

        def _make_ekf(process_noise_cov):
            return EKF(
                initial_mean=initial_state,
                initial_cov=self._config['initial_cov'],
                process_noise_cov=process_noise_cov,
                measurement_noise_cov=self._config['measurement_noise'],
            )

        self._ekf_fusion = _make_ekf(self._config['process_noise_fusion'])
        self._ekf_gnss_only = _make_ekf(self._config['process_noise_gnss_only'])
        self._initialized = True
        self.get_logger().info(
            f'EKFs initialized (yaw={init_yaw:.3f}, vel={init_vel.tolist()})')

    # =========================================================================
    # IMU (予測)
    # =========================================================================
    def _imu_callback(self, msg):
        if not self._initialized:
            return
        if self._first_imu:
            self._prev_imu_timestamp = get_timestamp(msg)
            self._first_imu = False
            return

        current_ts = get_timestamp(msg)
        dt = current_ts - self._prev_imu_timestamp
        self._prev_imu_timestamp = current_ts
        if dt <= 0:
            return

        imu_data = extract_imu_data_2d(msg)
        self._ekf_fusion.predict(imu_data, dt)
        self._ekf_gnss_only.predict(np.zeros(3), dt)

        self._publish_estimates()

    # =========================================================================
    # GNSS (補正)
    # =========================================================================
    def _gnss_callback(self, msg):
        if not self._initialized:
            self._initialize_from_gnss(msg)
            self._gnss_count += 1
            return

        self._gnss_count += 1
        current_ts = get_timestamp(msg)

        x, y, _ = lla_to_enu(
            msg.latitude, msg.longitude, msg.altitude,
            self._origin_lat, self._origin_lon, self._origin_alt,
        )
        gnss_pos = np.array([x, y])

        # /vel 未受信なら correct スキップ
        if self._latest_vel is None:
            self._publish_gnss_enu_pose(gnss_pos, float(self._ekf_fusion.x.yaw))
            self._publish_estimates()
            return

        vx, vy = float(self._latest_vel[0]), float(self._latest_vel[1])

        # ENU 観測のパブリッシュ (yaw は fusion 側の基準)
        yaw_pub = self._compute_yaw_from_vel(
            vx, vy, float(self._ekf_fusion.x.yaw))
        self._publish_gnss_enu_pose(gnss_pos, yaw_pub)

        # GNSS欠損区間なら correct しない
        if self._in_block_period(current_ts):
            self._publish_estimates()
            return

        yaw_f = self._compute_yaw_from_vel(
            vx, vy, float(self._ekf_fusion.x.yaw))
        yaw_g = self._compute_yaw_from_vel(
            vx, vy, float(self._ekf_gnss_only.x.yaw))

        z_f = np.array([gnss_pos[0], gnss_pos[1], yaw_f, vx, vy])
        z_g = np.array([gnss_pos[0], gnss_pos[1], yaw_g, vx, vy])

        self._ekf_fusion.correct(z_f)
        self._ekf_gnss_only.correct(z_g)

        self._publish_estimates()

    # =========================================================================
    def _compute_yaw_from_vel(self, vx, vy, fallback_yaw):
        """速度ベクトルから yaw を算出。低速時は fallback_yaw。"""
        speed = float(np.hypot(vx, vy))
        if speed > self._speed_threshold:
            return float(np.arctan2(vy, vx))
        return float(fallback_yaw)

    def _in_block_period(self, current_ts):
        """現在のGNSSがGNSS欠損期間に入っているか"""
        if self._gnss_count < self._block_start_index + 1:
            return False
        if self._block_start_time is None:
            self._block_start_time = current_ts
            self.get_logger().info(
                f'GNSS block period started at t={current_ts:.3f}s '
                f'(gnss_count={self._gnss_count})'
            )
        return current_ts - self._block_start_time <= self._block_duration

    # =========================================================================
    def _publish_estimates(self):
        if not self._initialized:
            return
        stamp = self.get_clock().now().to_msg()

        x_f, P_f = self._ekf_fusion.get_estimate()
        self._fusion_pose_pub.publish(
            make_posestamped_cov_msg_2d(
                pos=x_f.pos, yaw=float(x_f.yaw), cov=P_f,
                frame_id='map', stamp=stamp,
            )
        )

        x_g, P_g = self._ekf_gnss_only.get_estimate()
        self._gnss_only_pose_pub.publish(
            make_posestamped_cov_msg_2d(
                pos=x_g.pos, yaw=float(x_g.yaw), cov=P_g,
                frame_id='map', stamp=stamp,
            )
        )

    def _publish_gnss_enu_pose(self, gnss_pos, yaw):
        msg = make_posestamped_msg_2d(
            pos=gnss_pos, yaw=yaw,
            frame_id='map',
            stamp=self.get_clock().now().to_msg(),
        )
        self._enu_gnss_pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GNSSvsGNSSAndIMU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
