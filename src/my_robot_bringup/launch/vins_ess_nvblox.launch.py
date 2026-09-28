import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_robot_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'robot.urdf')
    vins_config = os.path.join(config_dir, 'vins_fusion_stereo_imu_config.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'vins_ess_nvblox.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    engine_file_path = LaunchConfiguration('engine_file_path')
    threshold = LaunchConfiguration('threshold')
    enable_rviz = LaunchConfiguration('rviz')
    nvblox_config = LaunchConfiguration('nvblox_config')
    override_bag_camera_info = LaunchConfiguration('override_bag_camera_info')

    # Resolve input layer dimensions dynamically based on model if not overridden
    engine_path_str = context.perform_substitution(engine_file_path)
    width_str = context.perform_substitution(LaunchConfiguration('input_layer_width'))
    height_str = context.perform_substitution(LaunchConfiguration('input_layer_height'))
    override_info_str = context.perform_substitution(override_bag_camera_info).lower()

    try:
        req_w = int(width_str)
        req_h = int(height_str)
    except ValueError:
        req_w, req_h = 0, 0

    if req_w <= 0 or req_h <= 0:
        if 'light_ess' in os.path.basename(engine_path_str):
            input_w, input_h = 480, 288
        else:
            # Standard full ESS model uses 960x576
            input_w, input_h = 960, 576
    else:
        input_w, input_h = req_w, req_h

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

    # Determine input camera_info topics for rectification
    # If playing older bags with unrectified CameraInfo, remap rectification to calibrated info
    left_camera_info_topic = '/camera/left/camera_info_rectified' if override_info_str in ('true', '1') else '/camera/left/camera_info'
    right_camera_info_topic = '/camera/right/camera_info_rectified' if override_info_str in ('true', '1') else '/camera/right/camera_info'

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
            ('camera_info', left_camera_info_topic),
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
            ('camera_info', right_camera_info_topic),
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
            'input_layer_width': input_w,
            'input_layer_height': input_h,
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

    # 6. Optional CameraInfo Relayer (when playing older bags with unrectified CameraInfo)
    camera_info_relay_node = Node(
        package='my_robot_perception',
        executable='camera_info_relay_node.py',
        name='camera_info_relay_node',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(override_bag_camera_info),
        output='screen'
    )

    nodes = [
        robot_state_pub,
        vins_node,
        ess_container,
        nvblox_node,
        rviz_node,
    ]
    if override_info_str in ('true', '1'):
        nodes.append(camera_info_relay_node)

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    default_nvblox_config = os.path.join(config_dir, 'nvblox_config.yaml')

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

    declare_input_layer_width = DeclareLaunchArgument(
        'input_layer_width',
        default_value='0',
        description='Input layer width for ESS (0 for auto-detect based on engine name: 480 for light_ess, 960 for standard ess)'
    )

    declare_input_layer_height = DeclareLaunchArgument(
        'input_layer_height',
        default_value='0',
        description='Input layer height for ESS (0 for auto-detect based on engine name: 288 for light_ess, 576 for standard ess)'
    )

    declare_override_bag_camera_info = DeclareLaunchArgument(
        'override_bag_camera_info',
        default_value='false',
        description='If true, override bag CameraInfo topics with calibrated stereo rectification matrices'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with integrated perception layout'
    )

    declare_nvblox_config = DeclareLaunchArgument(
        'nvblox_config',
        default_value=default_nvblox_config,
        description='Path to the nvblox configuration yaml '
                    '(use nvblox_config_edge.yaml on the edge device)'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_engine_file_path,
        declare_threshold,
        declare_input_layer_width,
        declare_input_layer_height,
        declare_override_bag_camera_info,
        declare_rviz,
        declare_nvblox_config,
        OpaqueFunction(function=launch_setup),
    ])
