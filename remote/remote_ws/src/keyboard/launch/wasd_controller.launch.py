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
    
    return LaunchDescription(
        [
            keyboard_publisher_node,
            wasd_controller_node,
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
