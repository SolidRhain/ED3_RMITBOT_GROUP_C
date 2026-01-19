import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for camera calibration.

    This launches the camera node and camera_calibration node together.
    Use a checkerboard pattern to calibrate the camera.

    Default checkerboard: 8x6 squares, 0.025m (25mm) square size

    Usage:
        ros2 launch rmitbot_vision camera_calibration.launch.py

    Once calibration is complete, the calibration file will be saved.
    Copy it to ~/.ros/camera_info/camera.yaml or the config folder.
    """

    # Launch arguments
    size_arg = DeclareLaunchArgument(
        'size',
        default_value='8x6',
        description='Checkerboard size (inner corners): columns x rows'
    )

    square_arg = DeclareLaunchArgument(
        'square',
        default_value='0.025',
        description='Checkerboard square size in meters'
    )

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='/camera',
        description='Camera namespace'
    )

    # Get configurations
    size = LaunchConfiguration('size')
    square = LaunchConfiguration('square')
    camera_ns = LaunchConfiguration('camera')

    # Camera Node (camera_ros)
    # Publishes to /camera/image_raw and /camera/camera_info
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'width': 640,
            'height': 480,
            'format': 'YUYV',
            'role': 'video',
        }],
        output='screen'
    )

    # Camera Calibration Node
    # Uses checkerboard pattern to compute camera intrinsics
    calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='cameracalibrator',
        output='screen',
        arguments=[
            '--size', size,
            '--square', square,
            '--ros-args', '--remap', 'image:=/camera/image_raw',
            '--ros-args', '--remap', 'camera:=/camera'
        ],
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera', '/camera')
        ]
    )

    return LaunchDescription([
        size_arg,
        square_arg,
        camera_arg,
        camera_node,
        calibration_node
    ])
