from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get config directory and params file path
    config_dir = os.path.join(get_package_share_directory('my_robot_bringup'), 'config')
    params_file = os.path.join(config_dir, 'params.yaml')

    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '2000000'],
            output='screen'
        ),

        # 1. ESP32 Serial / Telemetry Node
        Node(
            package='my_robot_firmware',
            executable='esp32_serial_node',
            name='esp32_serial_node',
            output='screen',
            parameters=[params_file]
        ),
        
        # 2. Camera Image Splitter Node
        Node(
            package='my_robot_perception',
            executable='image_splitter_node',
            name='image_splitter_node',
            output='screen',
            parameters=[params_file]
        ),
        
        # 3. INS EKF Fusion Node
        # Node(
        #     package='ins_ekf',
        #     executable='ins_ekf_node',
        #     name='ins_ekf_node',
        #     output='screen',
        #     parameters=[params_file]
        # )
    ])

