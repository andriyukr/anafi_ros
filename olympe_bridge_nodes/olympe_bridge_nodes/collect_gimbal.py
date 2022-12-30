#!/usr/bin/env python3

# ros2 run olympe_bridge_nodes test_anafi --ros-args -r __ns:=/anafi1

import numpy as np
import rclpy
import math
import threading
import time
import datetime
import sys
import os

from termcolor import colored
from timeit import default_timer as timer
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data, qos_profile_services_default, qos_profile_parameters, qos_profile_parameter_events
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, SetParametersResult
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import UInt8, UInt16, UInt32, UInt64, Int8, Float32, String, Header, Bool
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, TwistStamped, Vector3Stamped, Quaternion, Twist, Vector3
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
from olympe_bridge_interfaces.msg import PilotingCommand, MoveByCommand, MoveToCommand, CameraCommand, GimbalCommand, SkycontrollerCommand, TargetTrajectory
from olympe_bridge_interfaces.srv import PilotedPOI, FlightPlan, FollowMe, Location, Photo, Recording, String as StringSRV

class CollectGimbal(Node):
	def __init__(self):
		self.node = rclpy.create_node('collect_gimbal')

		self.node.get_logger().info("collect_gimbal is running...")

		# Publishers
		self.pub_gimbal_command = self.node.create_publisher(GimbalCommand, 'gimbal/command', qos_profile_system_default)

		# Subscribers
		self.node.create_subscription(Vector3Stamped, 'gimbal/relative', self.gimbal_relative_callback, qos_profile_sensor_data)
		self.node.create_subscription(Vector3Stamped, 'gimbal/absolute', self.gimbal_absolute_callback, qos_profile_sensor_data)
		self.node.create_subscription(GimbalCommand, 'gimbal/command', self.gimbal_command_callback, qos_profile_system_default)

		# Services
		self.gimbal_reset_client = self.node.create_client(Trigger, 'gimbal/reset')
		self.gimbal_calibrate_client = self.node.create_client(Trigger, 'gimbal/calibrate')
		self.get_parameters_client = self.node.create_client(GetParameters, 'anafi/get_parameters')
		self.set_parameters_client = self.node.create_client(SetParameters, 'anafi/set_parameters')
		while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
			self.node.get_logger().info('No connection...')

		# Variables
		self.gimbal_relative = np.nan
		self.gimbal_absolute = np.nan
		self.gimbal_command = np.nan

		self.test_thread = threading.Thread(target=self.test_gimbal)
		self.test_thread.start()

	def test_gimbal(self):
		# Drone name
		req = GetParameters.Request()
		req.names = ['model']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		self.model = future.result().values[0].string_value
		print("Drone model: " + self.model)

		print("\n***** GIMBAL TEST *****")

		# 4K:
		# 	GimbalMaxSpeed (deg/s): roll=180.0 [0.0, 180.0], pitch=180.0 [0.0, 180.0], yaw=0.0 [0.0, 0.0]
		# Thermal:
		# 	GimbalMaxSpeed (deg/s): roll=180.0 [0.0, 180.0], pitch=180.0 [0.0, 180.0], yaw=0.0 [0.0, 0.0]
		# USA:
		# 	GimbalMaxSpeed (deg/s): roll=180.0 [0.0, 180.0], pitch=180.0 [0.0, 180.0], yaw=0.0 [0.0, 0.0]
		# AI:
		# 	GimbalMaxSpeed (deg/s): roll=180.0 [1.0, 180.0], pitch=180.0 [1.0, 180.0], yaw=180.0 [1.0, 180.0]

		# Set max gimbal speed
		req = SetParameters.Request()
		req.parameters = [Parameter(name='max_gimbal_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=180.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)

		# Test gimbal calibration
		print("Gimbal calibration")
		req = Trigger.Request()
		future = self.gimbal_calibrate_client.call_async(req)
		for i in range(100):
			if future.result() is not None:
				break
			time.sleep(0.1)

		# Test gimbal reset
		print("Gimbal reset")
		req = Trigger.Request()
		future = self.gimbal_reset_client.call_async(req)
		for i in range(100):
			if future.result() is not None:
				break
			time.sleep(0.1)

		download_folder = "Documents/MATLAB/anafi/gimbal/"
		if not os.path.exists(download_folder):
			os.makedirs(download_folder)

		# 4K:
		# 	GimbalRelativeBounds (deg): roll=[-38.0, 38.0], pitch=[-105.0, 135.0], yaw=[0.0, 0.0]
		# Thermal:
		# 	GimbalRelativeBounds (deg): roll=[-38.0, 38.0], pitch=[-100.0, 135.0], yaw=[0.0, 0.0]
		# USA:
		# 	GimbalRelativeBounds (deg): roll=[-38.0, 38.0], pitch=[-105.0, 135.0], yaw=[0.0, 0.0]
		# AI:
		# 	GimbalRelativeBounds (deg):

		self.file = open(download_folder + "roll_relative_" + self.model + ".txt", "w")
		self.file.write("time, command_mode, command_frame, command_roll, command_pitch, " +
						"relative_roll, relative_pitch, relative_yaw, absolute_roll, absolute_pitch, absolute_yaw\n")
		self.file.write("0.0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0\n")
		self.time_initial = timer() - 0.01
		self.timer = self.node.create_timer(0.01, self.timer_callback)

		print("Roll Relative")
		msg_gimbal_command = GimbalCommand()
		# Positive relative roll orientation
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 1
		if self.model == "ai":
			msg_gimbal_command.frame = 2
		msg_gimbal_command.roll = 38.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		time.sleep(3.0)
		# Negative relative roll orientation
		msg_gimbal_command.roll = -38.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		time.sleep(3.0)
		# Zero roll orientation
		msg_gimbal_command.roll = 0.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		for i in range(3):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			time.sleep(1.0)
		# Positive relative roll velocity
		self.gimbal_reset_client.call_async(req)
		msg_gimbal_command.mode = 1
		msg_gimbal_command.roll = 38.0
		for i in range(3):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			time.sleep(1.0)
		# Negative relative roll velocity
		msg_gimbal_command.roll = -38.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		for i in range(3):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			time.sleep(1.0)
		# Reset roll orientation
		msg_gimbal_command.roll = 0.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		self.gimbal_reset_client.call_async(req)
		time.sleep(3.0)

		self.timer.destroy()
		self.file.close()

		self.file = open(download_folder + "pitch_relative_" + self.model + ".txt", "w")
		self.file.write("time, command_mode, command_frame, command_roll, command_pitch, " +
						"relative_roll, relative_pitch, relative_yaw, absolute_roll, absolute_pitch, absolute_yaw\n")
		self.file.write("0.0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0\n")
		self.time_initial = timer() - 0.01
		self.timer = self.node.create_timer(0.01, self.timer_callback)

		print("Pitch Relative")
		msg_gimbal_command = GimbalCommand()
		# Positive relative pitch orientation
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 1
		if self.model == "ai":
			msg_gimbal_command.frame = 2
		msg_gimbal_command.pitch = 135.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		time.sleep(3.0)
		# Negative relative roll orientation
		msg_gimbal_command.pitch = -105.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		time.sleep(3.0)
		# Zero roll orientation
		msg_gimbal_command.pitch = 0.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		time.sleep(3.0)
		# Positive relative pitch velocity
		self.gimbal_reset_client.call_async(req)
		msg_gimbal_command.mode = 1
		msg_gimbal_command.pitch = 135.0
		for i in range(3):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			time.sleep(1.0)
		# Negative relative pitch velocity
		msg_gimbal_command.pitch = -105.0
		for i in range(3):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			time.sleep(1.0)
		# Reset pitch orientation
		msg_gimbal_command.pitch = 0.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		self.gimbal_reset_client.call_async(req)
		time.sleep(3.0)

		self.timer.destroy()
		self.file.close()

		# 4K:
		# 	GimbalAbsoluteBounds (deg): roll=[0.0, 0.0], pitch=[-90.0, 90.0], yaw=[0.0, 0.0]
		# Thermal:
		# 	GimbalAbsoluteBounds (deg): roll=[0.0, 0.0], pitch=[-90.0, 90.0], yaw=[0.0, 0.0]
		# USA:
		# 	GimbalAbsoluteBounds (deg): roll=[0.0, 0.0], pitch=[-90.0, 90.0], yaw=[0.0, 0.0]
		# AI:
		# 	GimbalAbsoluteBounds (deg): roll=[-30.0, 30.0], pitch=[-90.0, 90.0], yaw=[-180.0, 180.0]

		self.file = open(download_folder + "pitch_absolute_" + self.model + ".txt", "w")
		self.file.write("time, command_mode, command_frame, command_roll, command_pitch, " +
						"relative_roll, relative_pitch, relative_yaw, absolute_roll, absolute_pitch, absolute_yaw\n")
		self.file.write("0.0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0\n")
		self.time_initial = timer() - 0.01
		self.timer = self.node.create_timer(0.01, self.timer_callback)

		print("Pitch Absolute")
		msg_gimbal_command = GimbalCommand()
		# Positive relative pitch orientation
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 2
		msg_gimbal_command.pitch = 90.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		time.sleep(3.0)
		# Negative relative roll orientation
		msg_gimbal_command.pitch = -90.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		time.sleep(3.0)
		# Zero roll orientation
		msg_gimbal_command.pitch = 0.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		time.sleep(3.0)
		# Positive relative pitch velocity
		self.gimbal_reset_client.call_async(req)
		msg_gimbal_command.mode = 1
		msg_gimbal_command.pitch = 90.0
		for i in range(3):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			time.sleep(1.0)
		# Negative relative pitch velocity
		msg_gimbal_command.pitch = -90.0
		for i in range(3):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			time.sleep(1.0)
		# Reset pitch orientation
		msg_gimbal_command.pitch = 0.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		self.gimbal_reset_client.call_async(req)
		time.sleep(3.0)

		self.timer.destroy()
		self.file.close()

		print("End")

	def timer_callback(self):
		self.file.write(str(timer() - self.time_initial) + ", " +
						str(self.gimbal_command.mode) + ", " +
						str(self.gimbal_command.frame) + ", " +
						str(self.gimbal_command.roll) + ", " +
						str(self.gimbal_command.pitch) + ", " +
						str(self.gimbal_relative.vector.x) + ", " +
						str(self.gimbal_relative.vector.y) + ", " +
						str(self.gimbal_relative.vector.z) + ", " +
						str(self.gimbal_absolute.vector.x) + ", " +
						str(self.gimbal_absolute.vector.y) + ", " +
						str(self.gimbal_absolute.vector.z) + "\n")

	def gimbal_relative_callback(self, msg):
		self.gimbal_relative = msg

	def gimbal_absolute_callback(self, msg):
		self.gimbal_absolute = msg

	def gimbal_command_callback(self, msg):
		self.gimbal_command = msg

def main(args=None):
	rclpy.init(args=sys.argv)

	collect_gimbal = CollectGimbal()

	rclpy.spin(collect_gimbal.node)

	collect_gimbal.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
