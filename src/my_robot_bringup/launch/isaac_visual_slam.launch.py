import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')
    enable_ground_constraint = LaunchConfiguration('enable_ground_constraint')
    image_jitter_threshold_ms = LaunchConfiguration('image_jitter_threshold_ms')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Bag) clock if true'
    )

    # Note: On USB stereo + serial IMU systems without hardware PPS trigger,
    # pure Stereo Visual SLAM (enable_imu_fusion:=false) is recommended to prevent
    # drift caused by asynchronous ~29ms USB/Serial time offset.
    declare_enable_imu_fusion = DeclareLaunchArgument(
        'enable_imu_fusion',
        default_value='false',
        description='Enable IMU fusion in cuVSLAM (default false for async USB/Serial sensors)'
    )

    declare_ground_constraint = DeclareLaunchArgument(
        'enable_ground_constraint',
        default_value='true',
        description='Constraint odometry to 2D ground plane for ground robots'
    )

    declare_image_jitter_threshold = DeclareLaunchArgument(
        'image_jitter_threshold_ms',
        default_value='60.0',
        description='Max allowed inter-frame jitter in ms (default 60ms for USB cameras)'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_imu_fusion,
        declare_ground_constraint,
        declare_image_jitter_threshold,

        # 0. Robot State Publisher (Static TF for base_link, camera optical frames, and imu_link)
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

        # 1. Isaac ROS Visual SLAM Node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'num_cameras': 2,
                'rectified_images': False,
                'enable_imu_fusion': enable_imu_fusion,
                'enable_ground_constraint_in_odometry': enable_ground_constraint,
                'base_frame': 'base_link',
                'imu_frame': 'imu_link',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'camera_optical_frames': [
                    'left_camera_optical_frame',
                    'right_camera_optical_frame',
                ],
                'enable_slam_visualization': True,
                'enable_landmarks_view': True,
                'enable_observations_view': True,
                'image_jitter_threshold_ms': image_jitter_threshold_ms,
                # IMU noise parameters tuned for ICM-20948 (when IMU fusion is enabled)
                'gyro_noise_density': 0.010,
                'gyro_random_walk': 0.00010,
                'accel_noise_density': 0.10,
                'accel_random_walk': 0.0010,
                'calibration_frequency': 200.0,
            }],
            remappings=[
                ('visual_slam/image_0', '/camera/left/image_raw'),
                ('visual_slam/camera_info_0', '/camera/left/camera_info'),
                ('visual_slam/image_1', '/camera/right/image_raw'),
                ('visual_slam/camera_info_1', '/camera/right/camera_info'),
                ('visual_slam/imu', '/imu/data_raw'),
            ]
        ),
    ])
