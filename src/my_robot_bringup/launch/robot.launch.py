from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get config directory and params file path
    config_dir = os.path.join(get_package_share_directory('my_robot_bringup'), 'config')
    params_file = os.path.join(config_dir, 'params.yaml')
    urdf_file = os.path.join(config_dir, 'robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    auto_log_arg = DeclareLaunchArgument(
        'auto_log',
        default_value='true',
        description='Enable or disable automatic rosbag recording (true/false or enable/disable).'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='total',
        description='Recording mode: esp32 (data from esp32), lidar (esp32+lidar), camera (esp32+camera), total (esp32+lidar+camera).'
    )

    return LaunchDescription([
        auto_log_arg,
        mode_arg,

        # 0. Robot State Publisher (Static TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

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

        # 2. Data Logger Node (rosbag recorder)
        Node(
            package='my_robot_firmware',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen',
            parameters=[params_file, {
                'auto_log': LaunchConfiguration('auto_log'),
                'mode': LaunchConfiguration('mode')
            }]
        ),
        
        # 3. Camera Image Splitter Node
        Node(
            package='my_robot_perception',
            executable='image_splitter_node',
            name='image_splitter_node',
            output='screen',
            parameters=[params_file]
        ),

        # 4. N10 LiDAR Node
        Node(
            package='my_robot_perception',
            executable='n10_lidar_node',
            name='n10_lidar_node',
            output='screen',
            parameters=[params_file]
        ),
        
        # 5. INS EKF Fusion Node
        Node(
            package='ins_ekf',
            executable='ins_ekf_node',
            name='ins_ekf_node',
            output='screen',
            parameters=[params_file]
        )
    ])
