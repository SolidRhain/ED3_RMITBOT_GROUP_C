import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    vision_pkg = get_package_share_directory('rmitbot_vision')
    camera_pkg = get_package_share_directory('camera_ros')

    apriltag_params = os.path.join(vision_pkg, 'config', 'apriltag_params.yaml')
    camera_info_url = 'file://' + os.path.join(vision_pkg, 'config', 'camera.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. Camera Node (camera_ros)
    # Publishes to /camera/image_raw and /camera/camera_info
    # Uses calibration file for accurate pose estimation
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
            'camera_info_url': camera_info_url,
        }],
        output='screen'
    )

    # 2. Image Processing (Rectification)
    # Takes /camera/image_raw -> Publishes /camera/image_rect
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        namespace='camera',
        output='screen',
        remappings=[
            ('image', 'image_raw'),
            ('image_rect', 'image_rect')
        ]
    )

    # 3. AprilTag Node
    # Listens to /camera/image_rect and /camera/camera_info
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[
            apriltag_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('image_rect', '/camera/image_rect'),
            ('camera_info', '/camera/camera_info')
        ]
    )

    return LaunchDescription([
        camera_node,
        rectify_node,
        apriltag_node
    ])