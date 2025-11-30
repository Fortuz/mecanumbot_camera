from setuptools import setup
from glob import glob
import os

package_name = 'mecanumbot_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # Config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),

        # Models (YOLO weights)
        (os.path.join('share', package_name, 'models'),
         glob('models/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roland Sándor Nagy',
    maintainer_email='newageson@inf.elte.hu',
    description='Camera + perception + tracking pipeline for mecanumbot.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_tracker_rgb = mecanumbot_camera.ball_tracker_rgb:main',
            'people_detector = mecanumbot_camera.people_detector:main',
            'overlay_fused = mecanumbot_camera.overlay_fused:main',
            'detection_overlay = mecanumbot_camera.detection_overlay:main',
            'video_publisher = mecanumbot_camera.video_publisher:main',
            'ball_follower = mecanumbot_camera.ball_follower:main',
            'behavior_manager = mecanumbot_camera.behavior_manager:main',
        ],
    },
)
