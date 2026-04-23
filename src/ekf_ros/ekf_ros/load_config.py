"""
load_config.py - EKF用設定ファイル読み込みユーティリティ

State2D: [pos(2), yaw(1), vel(2), bg(1), ba(2)] = 8次元
プロセスノイズ: [n_wz, n_ax, n_ay, n_bg, n_bax, n_bay] = 6次元
観測ノイズ: [px, py, yaw, vx, vy] = 5次元
"""

import yaml
import numpy as np


def read_yaml(file_path):
    """YAMLファイルを読み込む"""
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data


def get_init_cov(config):
    """
    初期共分散行列を生成 (8x8)
    
    状態ベクトル: [px, py, yaw, vx, vy, bg, bax, bay]
    """
    cov_params = config['initial_covariance']
    cov = np.eye(8)
    
    # 位置 (2次元)
    cov[0:2, 0:2] *= cov_params['position']
    
    # yaw (1次元)
    cov[2, 2] *= cov_params['attitude']
    
    # 速度 (2次元)
    cov[3:5, 3:5] *= cov_params['velocity']
    
    # ジャイロバイアス (1次元)
    cov[5, 5] *= cov_params['gyro_bias']
    
    # 加速度バイアス (2次元)
    cov[6:8, 6:8] *= cov_params['accel_bias']
    
    return cov


def get_process_noise(config):
    """
    プロセスノイズ共分散行列を生成 (6x6)
    
    ノイズベクトル: [n_wz, n_ax, n_ay, n_bg, n_bax, n_bay]
    """
    process_noise = config['process_noise']
    Q = np.eye(6)
    
    # 角速度ノイズ (1次元)
    Q[0, 0] *= process_noise['gyro_std'] ** 2
    
    # 加速度ノイズ (2次元)
    Q[1:3, 1:3] *= process_noise['accel_std'] ** 2
    
    # ジャイロバイアスランダムウォーク (1次元)
    Q[3, 3] *= process_noise['gyro_bias_rw_std'] ** 2
    
    # 加速度バイアスランダムウォーク (2次元)
    Q[4:6, 4:6] *= process_noise['accel_bias_rw_std'] ** 2
    
    return Q


def get_measurement_noise(config):
    """
    観測ノイズ共分散行列を生成 (5x5)
    
    観測ベクトル: [px, py, yaw, vx, vy]
    """
    meas_noise = config['measurement_noise']
    R = np.eye(5)
    
    # 位置ノイズ (2次元)
    R[0:2, 0:2] *= meas_noise['gnss_pos_std'] ** 2
    
    # yawノイズ (1次元)
    R[2, 2] *= meas_noise['gnss_yaw_std'] ** 2
    
    # 速度ノイズ (2次元)
    R[3:5, 3:5] *= meas_noise['gnss_vel_std'] ** 2
    
    return R


def load_ekf_config(file_path):
    """
    EKF用の全設定を一括読み込み
    
    Parameters
    ----------
    file_path : str
        YAMLファイルパス
        
    Returns
    -------
    dict
        'initial_cov': ndarray (8x8)
        'process_noise': ndarray (6x6)
        'measurement_noise': ndarray (5x5)
        'initial_state': dict (yaw, accel_bias, gyro_bias) or None
    """
    config = read_yaml(file_path)
    
    return {
        'initial_cov': get_init_cov(config),
        'process_noise': get_process_noise(config),
        'measurement_noise': get_measurement_noise(config),
        'initial_state': _load_initial_state(config),
    }


def _load_initial_state(config):
    """initial_state ブロックを dict で返す。無ければ None。"""
    if 'initial_state' not in config:
        return None
    blk = config['initial_state']
    return {
        'yaw': float(blk.get('yaw', 0.0)),
        'accel_bias': np.array(
            blk.get('accel_bias', [0.0, 0.0]), dtype=float),
        'gyro_bias': float(blk.get('gyro_bias', 0.0)),
    }


def _process_noise_from_block(block):
    """dict (gyro_std, accel_std, gyro_bias_rw_std, accel_bias_rw_std) -> 6x6 Q"""
    Q = np.eye(6)
    Q[0, 0] *= block['gyro_std'] ** 2
    Q[1:3, 1:3] *= block['accel_std'] ** 2
    Q[3, 3] *= block['gyro_bias_rw_std'] ** 2
    Q[4:6, 4:6] *= block['accel_bias_rw_std'] ** 2
    return Q


def load_gnss_vs_gnss_and_imu_config(file_path):
    """
    gnss_vs_gnss_and_imu_node 用の設定を読み込み。
    2種類のプロセスノイズ (fusion / gnss_only) を返す。

    Returns
    -------
    dict
        'initial_cov': ndarray (8x8)
        'process_noise_fusion': ndarray (6x6)
        'process_noise_gnss_only': ndarray (6x6)
        'measurement_noise': ndarray (5x5)
    """
    config = read_yaml(file_path)
    return {
        'initial_cov': get_init_cov(config),
        'process_noise_fusion': _process_noise_from_block(
            config['process_noise_fusion']),
        'process_noise_gnss_only': _process_noise_from_block(
            config['process_noise_gnss_only']),
        'measurement_noise': get_measurement_noise(config),
        'initial_state': _load_initial_state(config),
    }