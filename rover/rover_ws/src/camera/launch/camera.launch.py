from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # デバイスパスを引数で指定できるように設定
    declare_video_device_cmd = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path (e.g., /dev/video0, /dev/video1)'
    )

    # カメラ名を引数で指定できるように設定
    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name prefix for topic (e.g., camera, front_camera, rear_camera)'
    )

    # camera ノードの起動
    camera_cmd = Node(
        package='camera',
        executable='camera',
        name='camera',
        parameters=[
            {
                'video_device': LaunchConfiguration('video_device'),
                'camera_name': LaunchConfiguration('camera_name'),
            }
        ],
        output='screen'
    )

    ld = LaunchDescription([
        declare_video_device_cmd,
        declare_camera_name_cmd,
        camera_cmd,
    ])

    return ld
