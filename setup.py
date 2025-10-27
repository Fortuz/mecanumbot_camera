from setuptools import setup

package_name = 'mecanumbot_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_and_detectors.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mugambi Timothy Mwenda, Roland Sándor Nagy',
    maintainer_email='yc8eu4@inf.elte.hu, newageson@inf.elte.hu',
    description='Mecanumbot Camera: tennis ball (HSV) + people (YOLO ONNX) detectors for ROS 2 Humble.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_tracker_rgb = mecanumbot_camera.ball_tracker_rgb:main',
            'people_detector = mecanumbot_camera.people_detector:main',
            'video_publisher = mecanumbot_camera.video_publisher:main',
            'detection_overlay = mecanumbot_camera.detection_overlay:main',
            'overlay_fused   = mecanumbot_camera.overlay_fused:main', 
            'ball_follower = mecanumbot_camera.ball_follower:main',
            'search_action_server = mecanumbot_camera.search_action_server:main',
            'fetch_action_server  = mecanumbot_camera.fetch_action_server:main',
            'supervisor           = mecanumbot_camera.supervisor:main',
        ],
    },
)
