"""
gnss_vs_gnss_and_imu.launch.py

GNSS単独(IMU入力=0) vs GNSS+IMU 融合の EKF を比較するノードの launch ファイル。
sim_view.ipynb の GNSS 欠損区間の再現を ROS 上で行う。
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

    source_config = os.path.normpath(
        os.path.join(launch_dir, '..', 'config', 'rviz_conf.rviz')
    )
    if os.path.exists(source_config):
        return source_config

    return os.path.join(pkg_share, 'config', 'rviz_conf.rviz')


def generate_launch_description():
    pkg_share = get_package_share_directory('ekf_ros')
    default_rviz_config = _resolve_default_rviz_config(pkg_share)

    # Launch引数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    speed_threshold_arg = DeclareLaunchArgument(
        'speed_threshold', default_value='0.3',
        description='Speed threshold for yaw estimation'
    )
    gnss_block_start_index_arg = DeclareLaunchArgument(
        'gnss_block_start_index', default_value='55',
        description='GNSS count index to start blocking GNSS corrections'
    )
    gnss_block_duration_arg = DeclareLaunchArgument(
        'gnss_block_duration', default_value='1.0',
        description='Duration (sec) of the GNSS blackout window'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='Path to RViz config file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    speed_threshold = LaunchConfiguration('speed_threshold')
    gnss_block_start_index = LaunchConfiguration('gnss_block_start_index')
    gnss_block_duration = LaunchConfiguration('gnss_block_duration')
    rviz_config = LaunchConfiguration('rviz_config')

    # 比較ノード
    compare_node = Node(
        package='ekf_ros',
        executable='gnss_vs_gnss_and_imu_node',
        name='gnss_vs_gnss_and_imu_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'speed_threshold': speed_threshold,
            'gnss_block_start_index': gnss_block_start_index,
            'gnss_block_duration': gnss_block_duration,
        }],
        remappings=[
            ('/gnss', '/gnss1'),
            ('/imu', '/imu'),
            ('/vel', '/vel'),
        ],
    )

    # GNSS+IMU の軌跡表示用 viewer
    viewer_fusion_node = Node(
        package='ekf_ros',
        executable='ekf_viewer',
        name='ekf_viewer_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'estimated_pose_topic': '/estimated_pose_fusion',
            'enu_gnss_pose_topic': '/enu_gnss_pose',
            'ekf_marker_topic': '/ekf_fusion_markerarray',
            'gnss_marker_topic': '/gnss_markerarray',
        }],
    )

    # GNSS 単独 の軌跡表示用 viewer
    viewer_gnss_only_node = Node(
        package='ekf_ros',
        executable='ekf_viewer',
        name='ekf_viewer_gnss_only',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'estimated_pose_topic': '/estimated_pose_gnss_only',
            'enu_gnss_pose_topic': '/enu_gnss_pose',
            'ekf_marker_topic': '/ekf_gnss_only_markerarray',
            # GNSS の球体マーカーは fusion 側で既に表示しているので別トピックで発行
            'gnss_marker_topic': '/gnss_markerarray_dup',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        speed_threshold_arg,
        gnss_block_start_index_arg,
        gnss_block_duration_arg,
        rviz_config_arg,
        compare_node,
        viewer_fusion_node,
        viewer_gnss_only_node,
        rviz_node,
    ])
