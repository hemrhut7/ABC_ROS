import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    default_config_path = os.path.join(
        pkg_share, 'config', 'vins_fusion_stereo_imu_config.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to VINS-Fusion stereo+imu configuration file'
    )

    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        arguments=[config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_config_file,
        vins_node,
    ])
