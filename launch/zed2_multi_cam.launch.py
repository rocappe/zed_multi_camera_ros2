from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	config_common_path = LaunchConfiguration('config_common_path')
	config_camera_path = LaunchConfiguration('config_camera_path')

	config_common_path_defalut = os.path.join(
		get_package_share_directory('zed_wrapper'),
		'config',
		'common.yaml'
	)

	config_camera_path_defalut = os.path.join(
		get_package_share_directory('zed_wrapper'),
		'config',
		'zed2.yaml'
	)

	declare_config_common_path_cmd = DeclareLaunchArgument(
		'config_common_path',
		description='Path to the `<common>.yaml` file.', default_value=config_common_path_defalut)

	declare_config_camera_path_cmd = DeclareLaunchArgument(
		'config_camera_path',
		description='Path to the `<camera_model>.yaml` file.', default_value=config_camera_path_defalut)

	#"left_sn": 28846348,
	#"right_sn": 21128703,
	#"width": 1280,
	#"height": 720,
	#"downsampling": 1.0,
	#"fps": 15

	zed_multi_node = Node(
					package='zed_multi_camera', executable='zed_multi_camera',
					parameters=[
						# YAML files
						config_common_path,  # Common parameters
						config_camera_path,  # Camera related parameters
						# Overriding
						{
							'general.svo_file': "",
							'pos_tracking.base_frame': "base_frame"
						}
					],
					output="screen",
	)

	tf_node = Node(
				package="tf2_ros", executable="static_transform_publisher",
				arguments=["0.0", "0.20", "0.0", "0", "0", "0", "zed2_l_left_camera_optical_frame", "zed2_r_left_camera_optical_frame"],
				output="screen",
	)

	# Configuration variables
	# Camera name. Can be different from camera model, used to distinguish camera in multi-camera systems
	camera_name = 'zed2'
	node_name = 'zed_node'  # Zed Node name
	publish_urdf = 'true'  # Publish static frames from camera URDF
	# Robot base frame. Note: overrides the parameter `pos_tracking.base_frame` in `common.yaml`.
	base_frame = 'base_link'
	# Position X of the camera with respect to the base frame [m].
	cam_pos_x = '0.0'
	# Position Y of the camera with respect to the base frame [m].
	cam_pos_y = '0.0'
	# Position Z of the camera with respect to the base frame [m].
	cam_pos_z = '0.0'
	# Roll orientation of the camera with respect to the base frame [rad].
	cam_roll = '0.0'
	# Pitch orientation of the camera with respect to the base frame [rad].
	cam_pitch = '0.0'
	# Yaw orientation of the camera with respect to the base frame [rad].
	cam_yaw = '0.0'
	xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', 'zed_descr.urdf.xacro'
    )

	camera_model = 'zed2'

	rsp_node = Node(
			condition=IfCondition(publish_urdf),
			package='robot_state_publisher',
			namespace=camera_name,
			executable='robot_state_publisher',
			name='zed_state_publisher',
			output='screen',
			parameters=[{
				'robot_description': Command(
					[
						'xacro', ' ', xacro_path, ' ',
						'camera_name:=', camera_name, ' ',
						'camera_model:=', camera_model, ' ',
						'base_frame:=', base_frame, ' ',
						'cam_pos_x:=', cam_pos_x, ' ',
						'cam_pos_y:=', cam_pos_y, ' ',
						'cam_pos_z:=', cam_pos_z, ' ',
						'cam_roll:=', cam_roll, ' ',
						'cam_pitch:=', cam_pitch, ' ',
						'cam_yaw:=', cam_yaw
					])
			}]
		)

	return LaunchDescription([
		declare_config_common_path_cmd,
		declare_config_camera_path_cmd,
		zed_multi_node,
		tf_node,
		rsp_node
	])
