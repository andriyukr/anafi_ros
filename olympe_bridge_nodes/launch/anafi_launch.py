from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='olympe_bridge_nodes',
			namespace='anafi',
			executable='anafi',
			name='anafi',
			output="screen",
			emulate_tty=True,
			arguments=['--ros-args', '--log-level', 'INFO'],
			parameters=[
				{'model': ''},  # {'4k', 'thermal', 'usa', 'ai', 'sphinx'}
				{'ip': '192.168.53.1'},  # Anafi: '192.168.42.1', SkyController: '192.168.53.1'
				{'skycontroller_enabled': True},  # {True, False}
				{'rest_api_version': 1},
				{'drone_serial': ''},  # 'PS728000BA8G055937'
				{'wifi_key': ''}  # 'JP2GZQNQTTHZ'
			]
		),
	])