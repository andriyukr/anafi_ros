#!/usr/bin/env python3

# ros2 run anafi_ros_nodes example --ros-args -r __ns:=/anafi

import rclpy
import sys

from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, SetBool

from anafi_ros_interfaces.msg import PilotingCommand

class Example(Node):
	def __init__(self):
		self.node = rclpy.create_node('anafi_ros_example')

		self.node.get_logger().info("Example is running...")

		# Subscribers
		self.node.create_subscription(NavSatFix, 'drone/gps/location', self.gps_callback, qos_profile_sensor_data)
		self.node.create_subscription(String, 'drone/state', self.drone_state_callback, qos_profile_system_default)

		# Publishers
		self.pub_drone_command = self.node.create_publisher(PilotingCommand, 'drone/command', qos_profile_system_default)

		# Services
		self.drone_takeoff_client = self.node.create_client(Trigger, 'drone/takeoff')
		self.drone_land_client = self.node.create_client(Trigger, 'drone/land')
		self.skycontroller_offboard_client = self.node.create_client(SetBool, 'skycontroller/offboard')

		# Variables
		self.drone_state = ""
		self.time = Time(seconds=0.0)
		self.initial_time = Time(seconds=0.0)

		# Timer
		self.timer = self.node.create_timer(0.1, self.on_timer)
	
	# Timer callback
	def on_timer(self):
		piloting_command_msg = PilotingCommand()

		if self.time < self.initial_time + Duration(seconds=1.0):
			self.node.get_logger().info("Waiting for take-off time...", throttle_duration_sec=1.0)
		elif self.drone_state == "LANDED" and self.time < self.initial_time + Duration(seconds=2.0):
			self.node.get_logger().info("Taking-off...", throttle_duration_sec=1.0)
			req = Trigger.Request()
			self.drone_takeoff_client.call_async(req)
		elif self.time < self.initial_time + Duration(seconds=5.0):
			self.node.get_logger().info("Ascending...", throttle_duration_sec=1.0)
			piloting_command_msg.gaz = 1.0
			self.pub_drone_command.publish(piloting_command_msg)
		elif self.time < self.initial_time + Duration(seconds=10.0):
			self.node.get_logger().info("Hovering...", throttle_duration_sec=1.0)
			piloting_command_msg.gaz = 0.0
			self.pub_drone_command.publish(piloting_command_msg)
		elif self.drone_state != "LANDED" and self.drone_state != "LANDING" and self.time < self.initial_time + Duration(seconds=11.0):
			self.node.get_logger().info("Landing...", throttle_duration_sec=1.0)
			req = Trigger.Request()
			self.drone_land_client.call_async(req)
		elif self.drone_state == "LANDED":
			self.node.get_logger().info("The mission is over", throttle_duration_sec=1.0)
			exit()

	# GPS subscriber callback
	def gps_callback(self, msg):
		self.time = Time.from_msg(msg.header.stamp)
		if self.initial_time.nanoseconds == 0:
			self.initial_time = self.time
	
	# Drone state subscriber callback
	def drone_state_callback(self, msg):
		self.drone_state = msg.data  # 'LANDED', 'MOTOR_RAMPING', 'USER_TAKEOFF', 'TAKINGOFF', 'HOVERING', 'FLYING', 'LANDING', 'EMERGENCY'

def main(args=None):
	rclpy.init(args=sys.argv)

	example = Example()

	rclpy.spin(example.node)

	example.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
