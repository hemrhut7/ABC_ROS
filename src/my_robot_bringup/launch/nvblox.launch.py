import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    default_config_path = os.path.join(pkg_share, 'config', 'nvblox_config.yaml')

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
        description='Path to nvblox configuration yaml'
    )

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('camera_0/depth/image', '/stereo/depth'),
            ('camera_0/depth/camera_info', '/camera/left/camera_info_rect'),
            ('camera_0/color/image', '/camera/left/image_rect_rgb'),
            ('camera_0/color/camera_info', '/camera/left/camera_info_rect'),
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_config_file,
        nvblox_node,
    ])
