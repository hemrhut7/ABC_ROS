import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'robot.urdf')
    vins_config = os.path.join(config_dir, 'vins_fusion_stereo_imu_config.yaml')
    nvblox_config = os.path.join(config_dir, 'nvblox_config.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'vins_ess_nvblox.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    engine_file_path = LaunchConfiguration('engine_file_path')
    threshold = LaunchConfiguration('threshold')
    enable_rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true'
    )

    default_engine_candidates = [
        '/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/light_ess.engine',
        '/workspace/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/light_ess.engine',
    ]
    default_engine = default_engine_candidates[0]
    for p in default_engine_candidates:
        if os.path.exists(p):
            default_engine = p
            break

    declare_engine_file_path = DeclareLaunchArgument(
        'engine_file_path',
        default_value=default_engine,
        description='Absolute path to ESS TensorRT engine plan'
    )

    declare_threshold = DeclareLaunchArgument(
        'threshold',
        default_value='0.6',
        description='Confidence threshold for ESS disparity'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with integrated perception layout'
    )

    # 1. Robot State Publisher for Static Robot TF Tree
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # 2. VINS-Fusion Stereo + IMU Visual-Inertial Odometry Node
    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        arguments=[vins_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 3. Isaac ROS ESS Disparity & Depth Pipeline (GPU Accelerated with Rectification)
    rectify_left_node = ComposableNode(
        name='rectify_left_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_width': 640,
            'output_height': 480,
            'type_negotiation_duration_s': 5,
        }],
        remappings=[
            ('image_raw', '/camera/left/image_raw'),
            ('camera_info', '/camera/left/camera_info'),
            ('image_rect', '/camera/left/image_rect'),
            ('camera_info_rect', '/camera/left/camera_info_rect'),
        ]
    )

    rectify_right_node = ComposableNode(
        name='rectify_right_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_width': 640,
            'output_height': 480,
            'type_negotiation_duration_s': 5,
        }],
        remappings=[
            ('image_raw', '/camera/right/image_raw'),
            ('camera_info', '/camera/right/camera_info'),
            ('image_rect', '/camera/right/image_rect'),
            ('camera_info_rect', '/camera/right/camera_info_rect'),
        ]
    )

    ess_disparity_node = ComposableNode(
        name='ess_disparity_node',
        package='isaac_ros_ess',
        plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
        parameters=[{
            'use_sim_time': use_sim_time,
            'engine_file_path': engine_file_path,
            'threshold': threshold,
            'input_layer_width': 480,
            'input_layer_height': 288,
            'type_negotiation_duration_s': 5,
        }],
        remappings=[
            ('left/image_rect', '/camera/left/image_rect'),
            ('right/image_rect', '/camera/right/image_rect'),
            ('left/camera_info', '/camera/left/camera_info_rect'),
            ('right/camera_info', '/camera/right/camera_info_rect'),
            ('disparity', '/stereo/disparity'),
        ]
    )

    # 3.4 Disparity to Depth Image Node (metric depth in meters)
    disparity_to_depth_node = ComposableNode(
        name='disparity_to_depth_node',
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
        parameters=[{
            'use_sim_time': use_sim_time,
            'type_negotiation_duration_s': 5,
        }],
        remappings=[
            ('disparity', '/stereo/disparity'),
            ('depth', '/stereo/depth'),
        ]
    )

    # 3.5 Hardware-Accelerated Image Format Converter (BGR8 -> RGB8 for NVblox and Color PointCloud)
    image_format_converter_node = ComposableNode(
        name='image_format_converter_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'use_sim_time': use_sim_time,
            'encoding_desired': 'rgb8',
            'image_width': 640,
            'image_height': 480,
            'type_negotiation_duration_s': 5,
        }],
        remappings=[
            ('image_raw', '/camera/left/image_rect'),
            ('image', '/camera/left/image_rect_rgb'),
        ]
    )

    # 3.6 PointCloud Generator from Depth Image (for 3D Visual vs LiDAR Comparison)
    point_cloud_xyz_node = ComposableNode(
        name='point_cloud_xyz_node',
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzNode',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('image_rect', '/stereo/depth'),
            ('camera_info', '/camera/left/camera_info_rect'),
            ('points', '/stereo/points'),
        ]
    )

    ess_container = ComposableNodeContainer(
        name='ess_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_left_node,
            rectify_right_node,
            ess_disparity_node,
            disparity_to_depth_node,
            image_format_converter_node,
            point_cloud_xyz_node,
        ],
        output='screen'
    )

    # 4. NVIDIA NVblox 3D TSDF & 2D ESDF Reconstruction Node
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        parameters=[
            nvblox_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('camera_0/depth/image', '/stereo/depth'),
            ('camera_0/depth/camera_info', '/camera/left/camera_info_rect'),
            ('camera_0/color/image', '/camera/left/image_rect_rgb'),
            ('camera_0/color/camera_info', '/camera/left/camera_info_rect'),
        ],
        output='screen'
    )

    # 5. Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_engine_file_path,
        declare_threshold,
        declare_rviz,
        robot_state_pub,
        vins_node,
        ess_container,
        nvblox_node,
        rviz_node,
    ])
