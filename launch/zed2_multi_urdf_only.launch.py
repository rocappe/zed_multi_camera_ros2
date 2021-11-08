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

		# Configuration variables
	# Camera name. Can be different from camera model, used to distinguish camera in multi-camera systems
	camera_name_l = 'zed2_l'
	camera_name_r = 'zed2_r'
	publish_urdf = 'true'  # Publish static frames from camera URDF
	# Robot base frame. Note: overrides the parameter `pos_tracking.base_frame` in `common.yaml`.
	base_frame_l = camera_name_l + '_base_link'
	base_frame_r = camera_name_r + '_base_link'
	# Position X of the camera with respect to the base frame [m].
	cam_pos_x_l = '0.0'
	cam_pos_x_r = '0.0'
	# Position Y of the camera with respect to the base frame [m].
	cam_pos_y_l = '0.0'
	cam_pos_y_r = '0.0'
	# Position Z of the camera with respect to the base frame [m].
	cam_pos_z_l = '0.0'
	cam_pos_z_r = '0.0'
	# Roll orientation of the camera with respect to the base frame [rad].
	cam_roll_l = '0.0'
	cam_roll_r = '0.0'
	# Pitch orientation of the camera with respect to the base frame [rad].
	cam_pitch_l = '0.0'
	cam_pitch_r = '0.0'
	# Yaw orientation of the camera with respect to the base frame [rad].
	cam_yaw_l = '0.0'
	cam_yaw_r = '0.0'
	xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', 'zed_descr.urdf.xacro'
    )
	camera_model = 'zed2'

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


	tf_node = Node(
				package="tf2_ros", executable="static_transform_publisher",
				arguments=["0.0", "0.-20", "0.0", "0", "0", "0", camera_name_l + "_left_camera_frame", camera_name_r + "_left_camera_frame"],
				output="screen",
	)

	rsp_node_l = Node(
		condition=IfCondition(publish_urdf),
		package='robot_state_publisher',
		namespace=camera_name_l,
		executable='robot_state_publisher',
		name='zed_state_publisher',
		output='screen',
		parameters=[{
			'robot_description': Command(
				[
					'xacro', ' ', xacro_path, ' ',
					'camera_name:=', camera_name_l, ' ',
					'camera_model:=', camera_model, ' ',
					'base_frame:=', base_frame_l, ' ',
					'cam_pos_x:=', cam_pos_x_l, ' ',
					'cam_pos_y:=', cam_pos_y_l, ' ',
					'cam_pos_z:=', cam_pos_z_l, ' ',
					'cam_roll:=', cam_roll_l, ' ',
					'cam_pitch:=', cam_pitch_l, ' ',
					'cam_yaw:=', cam_yaw_l
				])
		}]
	)

	rsp_node_r = Node(
		condition=IfCondition(publish_urdf),
		package='robot_state_publisher',
		namespace=camera_name_r,
		executable='robot_state_publisher',
		name='zed_state_publisher',
		output='screen',
		parameters=[{
			'robot_description': Command(
				[
					'xacro', ' ', xacro_path, ' ',
					'camera_name:=', camera_name_r, ' ',
					'camera_model:=', camera_model, ' ',
					'base_frame:=', base_frame_r, ' ',
					'cam_pos_x:=', cam_pos_x_r, ' ',
					'cam_pos_y:=', cam_pos_y_r, ' ',
					'cam_pos_z:=', cam_pos_z_r, ' ',
					'cam_roll:=', cam_roll_r, ' ',
					'cam_pitch:=', cam_pitch_r, ' ',
					'cam_yaw:=', cam_yaw_r
				])
		}]
	)

	return LaunchDescription([
		declare_config_common_path_cmd,
		declare_config_camera_path_cmd,
		tf_node,
		rsp_node_l,
		rsp_node_r,
	])
