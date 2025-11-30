from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanumbot_camera')
    #params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    params = ParameterFile(
        PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),
        allow_substs=True
    )

    use_video = LaunchConfiguration('use_video')
    video_path = LaunchConfiguration('video_path')
    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([
        DeclareLaunchArgument('use_video', default_value='true'),
        DeclareLaunchArgument('model_path', default_value='/ws/src/mecanumbot_camera/models/yolov8n.onnx'),
        DeclareLaunchArgument('video_path', default_value='/ws/src/mecanumbot_camera/media/sample.mp4'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),

        # Video file source
        Node(
            package='mecanumbot_camera',
            executable='video_publisher',
            name='video_pub',
            output='screen',
            parameters=[{'video_path': video_path}],
            condition=IfCondition(use_video)
        ),

        # Real camera source (needs v4l2_camera installed)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2',
            output='screen',
            parameters=[{'video_device': video_device}],
            condition=UnlessCondition(use_video)
        ),

        # Detectors
        Node(
            package='mecanumbot_camera',
            executable='ball_tracker_rgb',
            name='ball_tracker_rgb',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='mecanumbot_camera',
            executable='people_detector',
            name='people_detector',
            output='screen',
            parameters=[params, {'model_path': LaunchConfiguration('model_path')}]
        ),

        # Fused overlay
        Node(
            package='mecanumbot_camera',
            executable='overlay_fused',
            name='overlay_fused',
            output='screen',
            parameters=[params],   # reuse your params.yaml (optional)
        ),

        # Behavior Manager
        Node(
            package='mecanumbot_camera',
            executable='behavior_manager',
            output='screen'
        ),

        # Foxglove
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[
                {"port": 8765},
                {"address": "0.0.0.0"}
            ]
        )
    ])
