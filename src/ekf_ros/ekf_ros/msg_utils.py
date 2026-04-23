# msg_utils.py に追加

import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import Imu
import math


def get_timestamp(msg):
    """ROSメッセージからタイムスタンプを秒で取得"""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def extract_imu_data_2d(msg):
    """
    IMUメッセージから2Dデータを抽出
    
    Returns
    -------
    ndarray (3,)
        [ax, ay, wz]
    """
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    wz = msg.angular_velocity.z
    return np.array([ax, ay, wz])


def yaw_to_quaternion(yaw):
    """yaw角をQuaternionに変換"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_posestamped_msg_2d(pos, yaw, frame_id, stamp):
    """
    2D位置・yawからPoseStampedメッセージを作成
    
    Parameters
    ----------
    pos : array-like (2,)
        [x, y]
    yaw : float
        yaw角 [rad]
    frame_id : str
    stamp : builtin_interfaces/Time
    """
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = float(pos[0])
    msg.pose.position.y = float(pos[1])
    msg.pose.position.z = 0.0
    msg.pose.orientation = yaw_to_quaternion(yaw)
    return msg


def make_posestamped_cov_msg_2d(pos, yaw, cov, frame_id, stamp):
    """
    2D位置・yaw・共分散からPoseWithCovarianceStampedメッセージを作成
    
    Parameters
    ----------
    pos : array-like (2,)
        [x, y]
    yaw : float
        yaw角 [rad]
    cov : ndarray (8x8)
        EKF共分散行列
    frame_id : str
    stamp : builtin_interfaces/Time
    """
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.pose.position.x = float(pos[0])
    msg.pose.pose.position.y = float(pos[1])
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation = yaw_to_quaternion(yaw)

    # 共分散行列を6x6（ROS形式）に変換
    # ROS: [x, y, z, roll, pitch, yaw]
    # EKF: [px, py, yaw, vx, vy, bg, bax, bay]
    cov_6x6 = np.zeros((6, 6))
    cov_6x6[0, 0] = cov[0, 0]  # x
    cov_6x6[1, 1] = cov[1, 1]  # y
    cov_6x6[0, 1] = cov[0, 1]
    cov_6x6[1, 0] = cov[1, 0]
    cov_6x6[5, 5] = cov[2, 2]  # yaw

    msg.pose.covariance = cov_6x6.flatten().tolist()
    return msg