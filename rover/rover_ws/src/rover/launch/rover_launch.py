from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # camera - camera node
        Node(
            package='camera',
            executable='camera',
            name='camera',
            output='screen',
        ),
        # camera - aruco_detector node
        Node(
            package='camera',
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
