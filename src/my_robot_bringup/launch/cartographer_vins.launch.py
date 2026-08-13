import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_robot_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'robot.urdf')
    vins_config_file = os.path.join(config_dir, 'vins_fusion_stereo_imu_config.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    vins_mode_str = LaunchConfiguration('vins_mode').perform(context)

    if vins_mode_str == 'vio_odom':
        configuration_basename = 'my_2d_cartographer_vins_vio_odom.lua'
    else:
        configuration_basename = 'my_2d_cartographer_vins_pure_imu.lua'

    return [
        # 1. Robot State Publisher
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

        # 2. VINS Node
        Node(
            package='vins',
            executable='vins_node',
            name='vins_estimator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[vins_config_file]
        ),

        # 3. PointCloud Converter Node (PointCloud -> PointCloud2)
        Node(
            package='my_robot_bringup',
            executable='pointcloud_converter_node',
            name='pointcloud_converter_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 4. Cartographer Node (PointCloud input, No LiDAR scans)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', configuration_basename,
                '-publish_tracked_pose'
            ],
            remappings=[
                ('imu', '/imu/data_raw'),
                ('points2', '/vins_estimator/point_cloud2'),
                ('odom', '/vins_estimator/odometry')
            ]
        ),

        # 5. Occupancy Grid Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        )
    ]

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Bag) clock if true'
    )

    declare_vins_mode = DeclareLaunchArgument(
        'vins_mode',
        default_value='pure_imu',
        description='VINS mode for Cartographer: "pure_imu" or "vio_odom"'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_vins_mode,
        OpaqueFunction(function=launch_setup)
    ])
