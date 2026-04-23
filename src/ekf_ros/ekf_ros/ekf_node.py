"""
ekf_ros_node.py - 2D EKF ROS2ノード（IMU予測 + GNSS補正）
"""

import os
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
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
from ekf_ros.load_config import load_ekf_config


class EKFROS(Node):
    """2D EKF ROS2ノード"""

    def __init__(self):
        super().__init__('ekf_ros_node')

        # パラメータ宣言・取得
        self.declare_parameter('speed_threshold', 0.3)
        self._speed_threshold = self.get_parameter('speed_threshold').get_parameter_value().double_value

        # 設定ファイル読み込み
        share_dir = get_package_share_directory('ekf_ros')
        config_path = os.path.join(share_dir, 'config', 'config.yaml')

        self._init_publishers()
        self._init_subscriptions()
        self._init_ekf_params(config_path)

        self.get_logger().info('EKFROS node initialized')

    def _init_publishers(self):
        """パブリッシャ初期化"""
        self._estimated_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/estimated_pose', 2
        )
        self._enu_gnss_pose_pub = self.create_publisher(
            PoseStamped, '/enu_gnss_pose', 2
        )

    def _init_subscriptions(self):
        """サブスクリプション初期化"""
        self._imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, 1
        )
        self._gnss_sub = self.create_subscription(
            NavSatFix, '/gnss', self._gnss_callback, 1
        )

    def _init_ekf_params(self, config_path):
        """EKFパラメータ初期化"""
        # EKF状態
        self._ekf = None
        self._initialized = False
        
        # IMU状態
        self._first_imu = True
        self._prev_imu_timestamp = None
        
        # GNSS状態
        self._prev_gnss_pos = None
        self._prev_gnss_timestamp = None
        
        # 原点座標
        self._origin_lat = 0.0
        self._origin_lon = 0.0
        self._origin_alt = 0.0
        
        # 設定読み込み
        self._config = load_ekf_config(config_path)

    # =========================================================================
    # 推定結果パブリッシュ
    # =========================================================================
    def _publish_estimated_pose(self):
        """推定結果をパブリッシュ"""
        if not self._initialized:
            return

        x_est, P = self._ekf.get_estimate()

        pose_msg = make_posestamped_cov_msg_2d(
            pos=x_est.pos,
            yaw=float(x_est.yaw),
            cov=P,
            frame_id='map',
            stamp=self.get_clock().now().to_msg()
        )
        self._estimated_pose_pub.publish(pose_msg)

    # =========================================================================
    # IMU コールバック
    # =========================================================================
    def _imu_callback(self, msg):
        """IMUコールバック - 予測ステップ"""
        if not self._initialized:
            return

        if self._first_imu:
            self._prev_imu_timestamp = get_timestamp(msg)
            self._first_imu = False
            return

        # 時間差分計算
        current_timestamp = get_timestamp(msg)
        dt = current_timestamp - self._prev_imu_timestamp
        self._prev_imu_timestamp = current_timestamp

        if dt <= 0:
            return

        # IMUデータ抽出 [ax, ay, wz]
        imu_data = extract_imu_data_2d(msg)
        
        # EKF予測ステップ
        self._ekf.predict(imu_data, dt)

        # 推定結果をパブリッシュ
        self._publish_estimated_pose()

    # =========================================================================
    # GNSS コールバック
    # =========================================================================
    def _gnss_callback(self, msg):
        """GNSSコールバック - 補正ステップ"""
        if not self._initialized:
            self._initialize_from_gnss(msg)
            return

        # ENU座標に変換
        x, y, _ = lla_to_enu(
            msg.latitude, msg.longitude, msg.altitude,
            self._origin_lat, self._origin_lon, self._origin_alt
        )
        gnss_pos = np.array([x, y])

        # yaw角計算
        yaw = self._compute_yaw_from_gnss(gnss_pos, msg)

        # 速度計算
        vel = self._compute_velocity_from_gnss(gnss_pos, msg)

        # 観測ベクトル [px, py, yaw, vx, vy]
        z = np.array([gnss_pos[0], gnss_pos[1], yaw, vel[0], vel[1]])

        # EKF補正ステップ
        self._ekf.correct(z)

        # GNSS位置をパブリッシュ
        self._publish_gnss_enu_pose(gnss_pos, yaw)

        # 推定結果をパブリッシュ
        self._publish_estimated_pose()

    def _initialize_from_gnss(self, msg):
        """最初のGNSSメッセージでEKFを初期化"""
        # 原点設定
        self._origin_lat = msg.latitude
        self._origin_lon = msg.longitude
        self._origin_alt = msg.altitude

        # 初期状態（原点なので位置は0）
        initial_state = State2D(
            pos=np.array([0.0, 0.0]),
            yaw=np.array([0.0]),
            vel=np.array([0.0, 0.0]),
            ba=np.array([0.0, 0.0]),
            bg=np.array([0.0])
        )

        # EKF初期化
        self._ekf = EKF(
            initial_mean=initial_state,
            initial_cov=self._config['initial_cov'],
            process_noise_cov=self._config['process_noise'],
            measurement_noise_cov=self._config['measurement_noise']
        )

        self._initialized = True
        self.get_logger().info('EKF initialized from GNSS')

    def _compute_yaw_from_gnss(self, gnss_pos, msg):
        """GNSS位置変化からyaw角を計算"""
        current_timestamp = get_timestamp(msg)

        if self._prev_gnss_pos is None or self._prev_gnss_timestamp is None:
            self._prev_gnss_pos = gnss_pos.copy()
            self._prev_gnss_timestamp = current_timestamp
            return 0.0

        dt = current_timestamp - self._prev_gnss_timestamp
        if dt <= 0:
            return float(self._ekf.x.yaw)

        # 位置差分から速度・yaw計算
        diff = gnss_pos - self._prev_gnss_pos
        speed = np.linalg.norm(diff) / dt

        if speed > self._speed_threshold:
            yaw = np.arctan2(diff[1], diff[0])
        else:
            # 低速時は現在の推定yawを使用
            yaw = float(self._ekf.x.yaw)

        return yaw

    def _compute_velocity_from_gnss(self, gnss_pos, msg):
        """GNSS位置変化から速度を計算"""
        current_timestamp = get_timestamp(msg)

        if self._prev_gnss_pos is None or self._prev_gnss_timestamp is None:
            self._prev_gnss_pos = gnss_pos.copy()
            self._prev_gnss_timestamp = current_timestamp
            return np.array([0.0, 0.0])

        dt = current_timestamp - self._prev_gnss_timestamp
        if dt <= 0:
            return self._ekf.x.vel.copy()

        # 速度計算
        vel = (gnss_pos - self._prev_gnss_pos) / dt

        # 状態更新
        self._prev_gnss_pos = gnss_pos.copy()
        self._prev_gnss_timestamp = current_timestamp

        return vel

    def _publish_gnss_enu_pose(self, gnss_pos, yaw):
        """GNSS ENU位置をパブリッシュ"""
        pose_msg = make_posestamped_msg_2d(
            pos=gnss_pos,
            yaw=yaw,
            frame_id='map',
            stamp=self.get_clock().now().to_msg()
        )
        self._enu_gnss_pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFROS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()