from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
	republish_node = Node(
		package="image_transport",
		executable="republish",
		name="camera_ffmpeg_republish",
		output="screen",
		parameters=[
			{"in_transport": "raw"},
			{"out_transport": "ffmpeg"},
			{"out.ffmpeg.encoder": "libx264"},
			{"out.ffmpeg.bit_rate": 200000},
			{"out.ffmpeg.qmax": 30},
			{"out.ffmpeg.gop_size": 30},
			{"out.ffmpeg.pixel_format": "yuv420p"},
			{
				"out.ffmpeg.encoder_av_options": (
					"preset:ultrafast,profile:baseline,tune:zerolatency,crf:45"
				)
			},
		],
		remappings=[
			("in", "/camera/color/image_raw"),
			("out/ffmpeg", "/camera_encoded/ffmpeg"),
		],
	)

	return LaunchDescription([republish_node])


