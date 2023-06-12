import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	# args that can be set from the command line or a default will be used
	namespace_arg = DeclareLaunchArgument(
		"namespace", 
		default_value="anafi", 
		description="Namespace for this Anafi")
	ip_arg = DeclareLaunchArgument(
		"ip", 
		default_value="192.168.53.1",
		description="IP address of Anafi to connect")

	config = os.path.join(
		get_package_share_directory('anafi_ros_nodes'),
		'params.yaml'
	)
	
	anafi_node = Node(
		package='anafi_ros_nodes',
		namespace=LaunchConfiguration('namespace'),
		executable='anafi',
		name='anafi',
		output="screen",
		emulate_tty=True,
		arguments=['--ros-args', '--params-file', config, '--log-level', 'INFO'],
		parameters=[
			{'model': '4k'},  # {'4k', 'thermal', 'usa', 'ai', 'sphinx'}
			{'ip': LaunchConfiguration('ip')},  # Anafi: '192.168.42.1', SkyController: '192.168.53.1'
			{'skycontroller_enabled': True},  # {True, False}
			{'rest_api_version': 1},
			{'drone_serial': ''},  # 'PS728000BA8G055937'
			{'wifi_key': ''},  # 'JP2GZQNQTTHZ'
		]
	)
        
	return LaunchDescription([
		namespace_arg,
		ip_arg,
		anafi_node
	])
