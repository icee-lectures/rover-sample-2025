from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 入力トピックを引数で指定できるように設定
    declare_input_topic_cmd = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic path'
    )

    # 出力トピックを引数で指定できるように設定
    declare_output_topic_cmd = DeclareLaunchArgument(
        'output_topic',
        default_value='/camera_fallback/color/image_raw/compressed',
        description='Output image topic path (compressed)'
    )

    # 出力解像度の幅を指定
    declare_output_width_cmd = DeclareLaunchArgument(
        'output_width',
        default_value='320',
        description='Output image width (default: 320)'
    )

    # 出力解像度の高さを指定
    declare_output_height_cmd = DeclareLaunchArgument(
        'output_height',
        default_value='240',
        description='Output image height (default: 240)'
    )

    # 出力間隔（ミリ秒）を指定
    declare_output_ms_cmd = DeclareLaunchArgument(
        'output_ms',
        default_value='500',
        description='Output interval in milliseconds (default: 500 ms)'
    )

    # JPEG圧縮品質を指定 (0-100, 高いほど高品質・低圧縮率)
    declare_jpeg_quality_cmd = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='30',
        description='JPEG compression quality (0-100, higher = better quality, lower compression)'
    )

    # camera_fallback ノードの起動
    camera_fallback_cmd = Node(
        package='camera',
        executable='camera_fallback',
        name='camera_fallback',
        parameters=[
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'output_width': LaunchConfiguration('output_width'),
                'output_height': LaunchConfiguration('output_height'),
                'output_ms': LaunchConfiguration('output_ms'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }
        ],
        output='screen'
    )

    # Launch description を作成
    ld = LaunchDescription()

    # 引数を追加
    ld.add_action(declare_input_topic_cmd)
    ld.add_action(declare_output_topic_cmd)
    ld.add_action(declare_output_width_cmd)
    ld.add_action(declare_output_height_cmd)
    ld.add_action(declare_output_ms_cmd)
    ld.add_action(declare_jpeg_quality_cmd)

    # ノードを追加
    ld.add_action(camera_fallback_cmd)

    return ld
