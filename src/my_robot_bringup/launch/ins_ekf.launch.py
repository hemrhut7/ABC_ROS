from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get config directory and params file path
    config_dir = os.path.join(get_package_share_directory('my_robot_bringup'), 'config')
    params_file = os.path.join(config_dir, 'params.yaml')

    return LaunchDescription([
        # INS EKF Fusion Node
        Node(
            package='ins_ekf',
            executable='ins_ekf_node',
            name='ins_ekf_node',
            output='screen',
            parameters=[params_file]
        )
    ])
