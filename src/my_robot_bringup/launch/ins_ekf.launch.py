from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true'
    )

    # Get config directory and params file path
    config_dir = os.path.join(get_package_share_directory('my_robot_bringup'), 'config')
    params_file = os.path.join(config_dir, 'params.yaml')

    return LaunchDescription([
        declare_use_sim_time,
        # INS EKF Fusion Node
        Node(
            package='ins_ekf',
            executable='ins_ekf_node',
            name='ins_ekf_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    ])
