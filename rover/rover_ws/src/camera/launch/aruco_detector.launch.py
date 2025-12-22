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

    # 処理間隔（ミリ秒）
    declare_process_interval_cmd = DeclareLaunchArgument(
        'process_interval_ms',
        default_value='500',
        description='Processing interval in milliseconds (default: 500ms)'
    )

    # 検出用画像スケール（0.0-1.0）
    declare_detection_scale_cmd = DeclareLaunchArgument(
        'detection_scale',
        default_value='0.5',
        description='Detection image scale (0.0-1.0, lower = faster, default: 0.5)'
    )

    # グレースケール処理
    declare_use_grayscale_cmd = DeclareLaunchArgument(
        'use_grayscale',
        default_value='true',
        description='Use grayscale for detection (true/false, default: true)'
    )

    # デバッグ画像JPEG品質
    declare_debug_jpeg_quality_cmd = DeclareLaunchArgument(
        'debug_jpeg_quality',
        default_value='50',
        description='Debug image JPEG quality (0-100, default: 50)'
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
                'process_interval_ms': LaunchConfiguration('process_interval_ms'),
                'detection_scale': LaunchConfiguration('detection_scale'),
                'use_grayscale': LaunchConfiguration('use_grayscale'),
                'debug_jpeg_quality': LaunchConfiguration('debug_jpeg_quality'),
            }
        ],
        output='screen'
    )

    ld = LaunchDescription([
        declare_image_topic_cmd,
        declare_namespace_prefix_cmd,
        declare_process_interval_cmd,
        declare_detection_scale_cmd,
        declare_use_grayscale_cmd,
        declare_debug_jpeg_quality_cmd,
        aruco_detector_cmd,
    ])

    return ld

