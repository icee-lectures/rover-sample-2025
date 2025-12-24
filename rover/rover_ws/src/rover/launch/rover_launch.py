import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # orbbec_camera の astra_pro_plus を起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('orbbec_camera'),
                    'launch',
                    'astra_pro_plus.launch.py',
                )
            )
        ),
        # camera パッケージの camera_fallback ノード（圧縮ストリーム出力）
        Node(
            package='camera',
            executable='camera_fallback',
            name='camera_fallback',
            output='screen',
            parameters=[
                {
                    'input_topic': 'camera/color/image_raw',
                    'output_topic': 'camera_fallback/color/image_raw/compressed',
                    'output_width': 320,
                    'output_height': 240,
                    'output_ms': 500,
                    'jpeg_quality': 30,
                }
            ],
        ),
        # camera パッケージの aruco_detector ノード
        Node(
            package='camera',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[
                {'image_topic': 'camera/color/image_raw/compressed'},
            ],
        ),
        # robot_control_board の driver_node ノード
        Node(
            package='robot_control_board',
            executable='driver_node',
            name='driver_node',
            output='screen',
        ),
    ])
