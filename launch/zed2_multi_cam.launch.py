from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='zed_multi_camera', executable='zed_multi_camera',
			parameters=[{
				"left_sn": 28846348,
				"right_sn": 21128703,
				"width": 1280,
				"height": 720,
				"downsampling": 1.0,
				"fps": 15
			}],
			output="screen",
		)
	])
