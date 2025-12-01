from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanumbot_camera')

    params = ParameterFile(
        PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),
        allow_substs=True
    )

    use_video = LaunchConfiguration('use_video')
    video_path = LaunchConfiguration('video_path')
    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([

        # ------------------------------------
        # Basic launch args
        # ------------------------------------
        DeclareLaunchArgument('use_video', default_value='false'),
        DeclareLaunchArgument('video_path', default_value='/ws/src/mecanumbot_camera/media/sample.mp4'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),

        # ------------------------------------
        # Video → ROS2
        # ------------------------------------
        Node(
            package='mecanumbot_camera',
            executable='video_publisher',
            name='video_pub',
            output='screen',
            parameters=[{'video_path': video_path}],
            condition=IfCondition(use_video)
        ),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2',
            output='screen',
            parameters=[
                {'video_device': video_device},
                {'image_size': [320, 240]},        # Pi3 optimization
                {'output_encoding': 'bgr8'},       # direct for cv2
                {'qos_overrides./v4l2_camera_node.image_raw.depth': 1},
                {'qos_overrides./v4l2_camera_node.image_raw.reliability': 'best_effort'}
            ],
            condition=UnlessCondition(use_video)
        ),

        # ------------------------------------
        # People detector (optimized)
        # ------------------------------------
        Node(
            package='mecanumbot_camera',
            executable='people_detector',
            name='people_detector',
            output='screen',
            parameters=[params]
        ),

        # ------------------------------------
        # Optional extra nodes (disabled by default)
        # ------------------------------------
        # Uncomment if really needed:

        # Node(
        #     package='mecanumbot_camera',
        #     executable='overlay_fused',
        #     name='overlay_fused',
        #     output='screen',
        #     parameters=[params],
        # ),

        # Node(
        #     package="foxglove_bridge",
        #     executable="foxglove_bridge",
        #     name="foxglove_bridge",
        #     output="screen",
        #     parameters=[
        #         {"port": 8765},
        #         {"address": "0.0.0.0"}
        #     ]
        # )
    ])
