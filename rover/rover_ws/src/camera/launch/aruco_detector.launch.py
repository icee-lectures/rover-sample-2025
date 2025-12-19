from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 入力画像トピック
    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='camera/image_raw/compressed',
        description='Input image topic name'
    )

    # 出力トピックのプレフィックス（"aruco" の置き換え用）
    declare_namespace_prefix_cmd = DeclareLaunchArgument(
        'namespace_prefix',
        default_value='aruco',
        description='Prefix for output topics (e.g., "/<prefix>/markers")'
    )

    # aruco_detector ノードの起動
    aruco_detector_cmd = Node(
        package='camera',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'namespace_prefix': LaunchConfiguration('namespace_prefix'),
                'dictionary_id': 10,  # cv::aruco::DICT_4X4_50
                'publish_debug_image': True,
            }
        ],
        output='screen'
    )

    ld = LaunchDescription([
        declare_image_topic_cmd,
        declare_namespace_prefix_cmd,
        aruco_detector_cmd,
    ])

    return ld

