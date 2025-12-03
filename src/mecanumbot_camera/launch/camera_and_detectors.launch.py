from launch import LaunchDescription
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

    return LaunchDescription([

        # --------------------------------------------------
        # 1) PiCam → /camera/image_raw (video_publisher)
        # --------------------------------------------------
        Node(
            package='mecanumbot_camera',
            executable='video_publisher',
            name='video_publisher',
            output='screen'
        ),

        # --------------------------------------------------
        # 2) Ball tracker
        # --------------------------------------------------
        Node(
            package='mecanumbot_camera',
            executable='ball_tracker_rgb',
            name='ball_tracker_rgb',
            output='screen',
            parameters=[params]
        ),

        # --------------------------------------------------
        # 3) People detector
        # --------------------------------------------------
        Node(
            package='mecanumbot_camera',
            executable='people_detector',
            name='people_detector',
            output='screen',
            parameters=[params]
        ),

        # --------------------------------------------------
        # 4) Overlay node → /camera/image_overlayed
        # --------------------------------------------------
        Node(
            package='mecanumbot_camera',
            executable='overlay_fused',
            name='overlay_fused',
            output='screen',
            parameters=[params]
        ),

        # --------------------------------------------------
        # 5) Behavior manager
        # --------------------------------------------------
        Node(
            package='mecanumbot_camera',
            executable='behavior_manager',
            name='behavior_manager',
            output='screen'
        )
    ])
