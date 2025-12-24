"""keyboard_publisher と wasd_controller を同時起動するローンチファイル"""

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    keyboard_publisher_node = Node(
        package='keyboard',
        executable='keyboard_publisher',
        name='keyboard_publisher',
        output='screen',
    )
    
    wasd_controller_node = Node(
        package='keyboard',
        executable='wasd_controller',
        name='wasd_controller',
        output='screen',
    )
    
    # image_transport republish ノード（ffmpeg -> raw）
    image_transport_republish_node = Node(
        package='image_transport',
        executable='republish',
        name='image_transport_republish',
        output='screen',
        parameters=[
            {'in_transport': 'ffmpeg'},
            {'out_transport': 'raw'},
        ],
        remappings=[
            ('in/ffmpeg', '/camera_encoded/ffmpeg'),
            ('out', '/camera_decoded/image_raw'),
        ],
    )
    
    return LaunchDescription(
        [
            keyboard_publisher_node,
            wasd_controller_node,
            image_transport_republish_node,
            # keyboard_publisherが終了したらシャットダウン
            RegisterEventHandler(
                OnProcessExit(
                    target_action=keyboard_publisher_node,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
            # wasd_controllerが終了したらシャットダウン
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wasd_controller_node,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
        ]
    )
