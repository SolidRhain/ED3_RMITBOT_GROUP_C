import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

# Launch the file
# ros2 launch rmitbot_bringup rmitbot.launch.py

def generate_launch_description():
    
    # Path to the package 
    pkg_path_description =  get_package_share_directory("rmitbot_description")
    pkg_path_controller =   get_package_share_directory("rmitbot_controller")
    pkg_path_localization = get_package_share_directory("rmitbot_localization")
    pkg_path_mapping = get_package_share_directory("rmitbot_mapping")
    pkg_path_navigation = get_package_share_directory("rmitbot_navigation")
    pkg_path_vision = get_package_share_directory("rmitbot_vision")
    pkg_path_bridges = get_package_share_directory("web_bridge")
    pkg_path_system = get_package_share_directory("position_manager")

    
    # Launch rviz
    display = IncludeLaunchDescription(
        os.path.join(pkg_path_description,"launch","display.launch.py"),
    )
    
    
    # Launch the controller manager
    controller = IncludeLaunchDescription(
        os.path.join(pkg_path_controller,"launch","controller.launch.py"),
    )
        
    # Launch ekf node
    localization = IncludeLaunchDescription(
        os.path.join(pkg_path_localization,"launch","localization.launch.py"),
    )
    
    # Launch the rplidar hardware
    rplidar = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping,"launch", "rplidar.launch.py"),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
    
    # Launch the mapping node
    mapping = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping,"launch","mapping.launch.py"),
    )
    
        # Launch the twistmux instead of keyboard node only
    twistmux = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation,"launch","twistmux.launch.py"),
    )
    
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation,"launch","nav.launch.py"),
    )
    
    
    # Launch the navigation 10s after slamtoolbox, to make sure that a map is available
    navigation_delayed = TimerAction(
        period = 5., 
        actions=[navigation]
    )

    # Launch Vision (Camera + AprilTag)
    vision = IncludeLaunchDescription(
        os.path.join(pkg_path_vision, "launch", "vision.launch.py"),
        launch_arguments={
            "use_sim_time": "False" 
        }.items()
    )
    web_bridges = IncludeLaunchDescription(
        os.path.join(pkg_path_bridges, "launch","bridges.launch.py"),
    )

    system = IncludeLaunchDescription(
        os.path.join(pkg_path_system, "launch","system.launch.py"),
    )

    # foxglove_bridge = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(pkg_path_foxglove, "launch", "foxglove_bridge_launch.xml")
    #     ),
    #     launch_arguments={
    #         "port": "8765"
    #     }.items()
    # )
    # PC: display, twistmux, navigation_delayed
    #RPI: controller, localization, rplidar, mapping
    return LaunchDescription([
        display,
        # controller,
        twistmux,
        # localization,
        # rplidar, 
        # mapping, 
        vision,
        navigation_delayed, 
        web_bridges,
        system,
    ])