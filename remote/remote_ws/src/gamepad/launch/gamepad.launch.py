"""
Launchファイル: gamepad_publisher と joy_to_cmd_vel ノードを起動
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():    
    # gamepad_publisher ノード
    gamepad_publisher_node = Node(
        package='gamepad',
        executable='gamepad_publisher',
        name='gamepad_publisher',
        output='screen',
    )
    
    # joy_to_cmd_vel ノード
    joy_to_cmd_vel_node = Node(
        package='gamepad',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen',
    )
    
    # LaunchDescription を返す
    return LaunchDescription([
        gamepad_publisher_node,
        joy_to_cmd_vel_node,
    ])
