import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_share = get_package_share_directory('my_robot_bringup')

    # Paths to files
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'robot.urdf')

    # Read URDF content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Default VINS-Fusion config path
    default_config = os.path.join(
        config_dir, 'vins_fusion_stereo_imu_config.yaml'
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Bag) clock if true'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Full path to the VINS-Fusion config YAML file'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_config_file,

        # 0. Robot State Publisher (Static TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        ),

        # 1. VINS-Fusion Node
        Node(
            package='vins_fusion_ros2',
            executable='vins_fusion_node',
            name='vins_fusion_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'config_file': config_file,
            }]
        ),
    ])
