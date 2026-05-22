from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanumbot_camera')

    params = ParameterFile(
        PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),
        allow_substs=True
    )

    enable_overlay = LaunchConfiguration('enable_overlay')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_overlay',
            default_value='false',
            description='Enable debug overlay_fused node publishing /camera/fused_debug'
        ),
        # External camera node is expected to publish /camera/image_raw.

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
            parameters=[params]
        ),

        Node(
            package='mecanumbot_camera',
            executable='overlay_fused',
            name='overlay_fused',
            output='screen',
            parameters=[params],
            condition=IfCondition(enable_overlay)
        ),

        Node(
            package='mecanumbot_camera',
            executable='behavior_manager',
            name='behavior_manager',
            output='screen'
        )
    ])
