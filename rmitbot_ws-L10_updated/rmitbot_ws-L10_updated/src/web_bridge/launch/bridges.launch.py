import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen',
            emulate_tty=True,  
            respawn=True,
            respawn_delay=2.0,
            parameters=[],  
        ),
        
        TimerAction(
            period=3.0,  
            actions=[
                Node(
                    package='rosbridge_server',
                    executable='rosbridge_websocket',
                    name='rosbridge_websocket',
                    output='screen',
                    emulate_tty=True,
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[{
                        'port': 9090
                    }],
                ),
            ]
        ),
        
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'send_buffer_limit': 10000000, 
                'use_compression': True
            }],
            output='screen'
        )

    ])
