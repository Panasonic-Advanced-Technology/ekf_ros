"""
ekf.launch.py - EKF ROS2 ノード起動ファイル
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _resolve_default_rviz_config(pkg_share):
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if '/install/' in launch_dir:
        workspace_root = launch_dir.split('/install/', 1)[0]
        workspace_source_config = os.path.join(
            workspace_root, 'src', 'ekf_ros', 'config', 'rviz_conf.rviz'
        )
        if os.path.exists(workspace_source_config):
            return workspace_source_config

    source_config = os.path.normpath(os.path.join(launch_dir, '..', 'config', 'rviz_conf.rviz'))
    if os.path.exists(source_config):
        return source_config

    return os.path.join(pkg_share, 'config', 'rviz_conf.rviz')


def generate_launch_description():
    # パッケージパス取得
    pkg_share = get_package_share_directory('ekf_ros')
    
    # デフォルトRViz設定ファイルパス
    default_rviz_config = _resolve_default_rviz_config(pkg_share)

    # =============================
    # Launch引数
    # =============================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    speed_threshold_arg = DeclareLaunchArgument(
        'speed_threshold',
        default_value='0.3',
        description='Speed threshold for yaw estimation'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config file'
    )

    # Launch設定値
    use_sim_time = LaunchConfiguration('use_sim_time')
    speed_threshold = LaunchConfiguration('speed_threshold')
    rviz_config = LaunchConfiguration('rviz_config')

    # =============================
    # ノード定義
    # =============================
    ekf_node = Node(
        package='ekf_ros',
        executable='ekf_node',
        name='ekf_ros_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'speed_threshold': speed_threshold,
        }],
        remappings=[
            ('/gnss', '/gnss1'),
            ('/imu', '/imu'),
        ]
    )

    viewer_node = Node(
        package='ekf_ros',
        executable='ekf_viewer',
        name='ekf_viewer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    # =============================
    # LaunchDescription
    # =============================
    return LaunchDescription([
        # 引数
        use_sim_time_arg,
        speed_threshold_arg,
        rviz_config_arg,
        # ノード
        ekf_node,
        viewer_node,
        rviz_node,
    ])