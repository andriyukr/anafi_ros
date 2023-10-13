#!/usr/bin/env python3

# ros2 run anafi_ros_nodes sphinx --ros-args -r __ns:=/anafi -p drone_name:=anafi

import rclpy
import sys
import pysphinx  # . /opt/parrot-sphinx/usr/bin/parrot-sphinx-setenv.sh

from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from anafi_ros_nodes.utils import euler_from_quaternion, quaternion_from_euler


class Sphinx(Node):
	def __init__(self):
		self.node = rclpy.create_node('sphinx')

		self.node.get_logger().info("Sphinx node is running...")

		# Subscribers
		self.node.create_subscription(PoseStamped, 'drone/move', self.move_callback, qos_profile_system_default)

		# Publishers
		self.pub_pose = self.node.create_publisher(PoseStamped, 'drone/pose', qos_profile_sensor_data)

		# Messages
		self.msg_pose = PoseStamped()

		# Parameters from the launch file
		self.drone_name = self.node.declare_parameter('drone_name', 'anafi').value
		self.node.get_logger().info('Drone name: ' + self.drone_name)

		self.sphinx = pysphinx.Sphinx(ip="127.0.0.1", port=8383)

		machine_names = self.sphinx.get_machine_names() # https://developer.parrot.com/docs/sphinx/pysphinxapi.html#pysphinx.Sphinx.get_machine_names

		if machine_names is None:
			self.node.get_logger().fatal("Sphinx is not running")
			exit()
		
		if self.drone_name not in machine_names:
			self.node.get_logger().fatal(self.drone_name + " does not exist in Sphinx")
			self.node.get_logger().info("Available drones: " + str(machine_names))
			exit()

		self.timer_fast = self.node.create_timer(0.001, self.pose_callback)
		
	def pose_callback(self):
		pose = self.sphinx.get_drone_pose(machine_name=self.drone_name)  # https://developer.parrot.com/docs/sphinx/pysphinxapi.html#pysphinx.Sphinx.get_drone_pose
		quaternion = quaternion_from_euler(pose[3], pose[4], pose[5])
		
		self.msg_pose.header.stamp = self.node.get_clock().now().to_msg()
		self.msg_pose.header.frame_id = '/world'
		self.msg_pose.pose.position.x = pose[0]
		self.msg_pose.pose.position.y = pose[1]
		self.msg_pose.pose.position.z = pose[2]
		self.msg_pose.pose.orientation.x = quaternion[0]
		self.msg_pose.pose.orientation.y = quaternion[1]
		self.msg_pose.pose.orientation.z = quaternion[2]
		self.msg_pose.pose.orientation.w = quaternion[3]
		self.pub_pose.publish(self.msg_pose)

	def move_callback(self, msg):
		(roll, pitch, yaw) = euler_from_quaternion(msg.pose.orientation)
		desired_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw)
		self.sphinx.move_drone(machine_name=self.drone_name, pose_offset=desired_pose) # https://developer.parrot.com/docs/sphinx/pysphinxapi.html#pysphinx.Sphinx.move_drone


def main(args=None):
	rclpy.init(args=sys.argv)

	sphinx = Sphinx()

	rclpy.spin(sphinx.node)

	sphinx.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
