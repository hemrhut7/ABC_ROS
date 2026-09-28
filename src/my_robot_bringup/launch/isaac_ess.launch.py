import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_robot_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    engine_file_path = LaunchConfiguration('engine_file_path')
    threshold = LaunchConfiguration('threshold')

    engine_path_str = context.perform_substitution(engine_file_path)
    width_str = context.perform_substitution(LaunchConfiguration('input_layer_width'))
    height_str = context.perform_substitution(LaunchConfiguration('input_layer_height'))

    try:
        req_w = int(width_str)
        req_h = int(height_str)
    except ValueError:
        req_w, req_h = 0, 0

    if req_w <= 0 or req_h <= 0:
        if 'light_ess' in os.path.basename(engine_path_str):
            input_w, input_h = 480, 288
        else:
            input_w, input_h = 960, 576
    else:
        input_w, input_h = req_w, req_h

    # 0. Robot State Publisher for TF frames
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

    # 1. Left & Right Camera Hardware-Accelerated Rectify Nodes (Undistortion & Epipolar Alignment)
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

    # 2. Isaac ROS ESS Disparity Node (DNN Inference on Rectified Images)
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

    # 3. Disparity to Depth Image Node (metric depth in meters)
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

    # Composable Node Container running on GPU / multi-threaded runtime
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
        ],
        output='screen'
    )

    return [
        robot_state_pub,
        ess_container,
    ]


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Bag) clock if true'
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
        default_value='0.4',
        description='Confidence threshold (0.0 - 1.0) for filtering disparity outliers'
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

    return LaunchDescription([
        declare_use_sim_time,
        declare_engine_file_path,
        declare_threshold,
        declare_input_layer_width,
        declare_input_layer_height,
        OpaqueFunction(function=launch_setup),
    ])
