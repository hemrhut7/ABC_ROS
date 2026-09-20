import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'robot.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_basename = LaunchConfiguration('configuration_basename')
    launch_rsp = LaunchConfiguration('launch_rsp')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Bag) clock if true'
    )

    declare_config_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='my_2d_cartographer.lua',
        description='Cartographer Lua configuration file name'
    )

    declare_launch_rsp = DeclareLaunchArgument(
        'launch_rsp',
        default_value='true',
        description='Whether to launch robot_state_publisher'
    )

    # 1. Robot State Publisher (Static TF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
        condition=IfCondition(launch_rsp)
    )

    # 2. Cartographer Node
    cartographer_node = Node(
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
            ('scan', '/scan')
        ]
    )

    # 3. Occupancy Grid Node (converts Cartographer submaps to 2D occupancy grid map)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_config_basename,
        declare_launch_rsp,
        robot_state_pub,
        cartographer_node,
        occupancy_grid_node,
    ])
