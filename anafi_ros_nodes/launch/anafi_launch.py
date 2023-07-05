# Usage: 
# 	- connection through Skycontroiller [recommended]:
# 		ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='192.168.53.1'
#	- direct connection to Anafi:
# 		ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='192.168.42.1'
#	- connection to the simulated drone in Sphinx:
# 		ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='10.202.0.1'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	# args that can be set from the command line or default will be used
	namespace_arg = DeclareLaunchArgument(
		"namespace", 
		default_value="anafi", 
		description="Namespace for this Anafi")
	ip_arg = DeclareLaunchArgument(
		"ip", 
		default_value="192.168.53.1",  # Anafi: '192.168.42.1', SkyController: '192.168.53.1', Sphinx: '10.202.0.1'
		description="IP address of the device")

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
			{'drone/model': 'ai'},  # {'4k', 'thermal', 'usa', 'ai'},
			{'device/ip': LaunchConfiguration('ip')}
		]
	)
        
	return LaunchDescription([
		namespace_arg,
		ip_arg,
		anafi_node
	])
