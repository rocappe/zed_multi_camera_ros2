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
		),
		Node(
			package="tf2_ros", executable="static_transform_publisher",
			arguments=["0.0", "0.20", "0.0", "0", "0", "0", "zed2_l_left_camera_optical_frame", "zed2_r_left_camera_optical_frame"],
			output="screen",
		),
	])
