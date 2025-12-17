from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # camera_main - camera node
        Node(
            package='camera_main',
            executable='camera',
            name='camera',
            output='screen',
        ),
        # camera_main - aruco_detector node
        Node(
            package='camera_main',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
        ),
        # robot_control_board - driver_node
        Node(
            package='robot_control_board',
            executable='driver_node',
            name='driver_node',
            output='screen',
        ),
    ])
