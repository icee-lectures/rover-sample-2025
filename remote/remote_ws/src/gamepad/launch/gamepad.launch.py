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
    
    # LaunchDescription を返す
    return LaunchDescription([
        gamepad_publisher_node,
        joy_to_cmd_vel_node,
        image_transport_republish_node,
    ])
