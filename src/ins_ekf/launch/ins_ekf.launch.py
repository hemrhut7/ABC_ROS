from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ins_ekf',
            executable='ins_ekf_node',
            name='ins_ekf_node',
            output='screen',
            parameters=[{
                'ekf_mode': 16,
                'imu_topic': '/imu/data',
                'gnss_topic': '/gnss/fix',
                'enable_gnss_pos': True,
                'enable_mag': True,
                'enable_baro': True,
                'enable_agv': True,
            }]
        )
    ])
