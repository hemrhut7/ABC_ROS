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
                'imu_topic': '/imu/data_raw',
                'gnss_topic': '/gnss/fix',
                'mag_topic': '/imu/mag',
                'baro_topic': '/baro/pressure',
                'enable_gnss_pos': False,
                'enable_gnss_vel': False,
                'enable_mag': True,
                'enable_baro': True,
                'enable_agv': True,
                'block_agv_h': False,
                'std_agv': [0.1, 0.1, 10.0],
                'lever_arm_agv': [0.0, 0.0, 0.0],
                'enable_nhc': False,
                'enable_zupt_hor': False,
                'press_params': 10.0,
                'std_mag': 50.0,
                'std_mag_yaw': 5.0,
                'init_cov_p': 5.0,
                'init_cov_v': 0.5,
                'init_cov_att': 3.0,
                'init_cov_yaw': 30.0,
            }]
        )
    ])
