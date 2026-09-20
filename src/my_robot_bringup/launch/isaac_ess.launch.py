import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    engine_file_path = LaunchConfiguration('engine_file_path')
    threshold = LaunchConfiguration('threshold')

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

    # 1. Isaac ROS ESS Disparity Node (DNN Inference)
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
            ('left/image_rect', '/camera/left/image_raw'),
            ('right/image_rect', '/camera/right/image_raw'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/camera_info', '/camera/right/camera_info'),
            ('disparity', '/stereo/disparity'),
        ]
    )

    # 2. Disparity to Depth Image Node (metric depth in meters)
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
            ess_disparity_node,
            disparity_to_depth_node,
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_engine_file_path,
        declare_threshold,
        robot_state_pub,
        ess_container,
    ])
