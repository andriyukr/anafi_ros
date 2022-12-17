#!/usr/bin/env python3

import numpy as np
from timeit import default_timer as timer

import rclpy
import math
import os
import requests
import shlex
import subprocess
import tempfile
import threading
import traceback
import time
import datetime
import logging
import sys

from termcolor import colored
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
from olympe_bridge_interfaces.msg import PilotingCommand, MoveByCommand, MoveToCommand, CameraCommand, GimbalCommand, SkycontrollerCommand, TargetTrajectory
from olympe_bridge_interfaces.srv import PilotedPOI, FlightPlan, FollowMe, Location, String as StringSRV

class TestAnafi(Node):
	def __init__(self):
		self.node = rclpy.create_node('test_anafi')

		self.node.get_logger().info("test_anafi is running...")

		# Publishers
		self.pub_drone_command = self.node.create_publisher(PilotingCommand, 'drone/command', qos_profile_system_default)
		self.pub_drone_moveto= self.node.create_publisher(MoveToCommand, 'drone/moveto', qos_profile_system_default)
		self.pub_drone_moveby = self.node.create_publisher(MoveByCommand, 'drone/moveby', qos_profile_system_default)
		self.pub_camera_command = self.node.create_publisher(CameraCommand, 'camera/command', qos_profile_system_default)
		self.pub_gimbal_command = self.node.create_publisher(GimbalCommand, 'gimbal/command', qos_profile_system_default)

		# Subscribers
		self.node.create_subscription(Image, 'camera/image', self.camera_image_callback, qos_profile_system_default)
		self.node.create_subscription(CameraInfo, 'camera/camera_info', self.camera_camera_info_callback, qos_profile_system_default)
		self.node.create_subscription(Time, 'time', self.time_callback, qos_profile_sensor_data)
		self.node.create_subscription(QuaternionStamped, 'drone/attitude', self.drone_attitude_callback, qos_profile_sensor_data)
		self.node.create_subscription(Float32, 'drone/altitude', self.drone_altitude_callback, qos_profile_sensor_data)
		self.node.create_subscription(Vector3Stamped, 'drone/speed', self.drone_speed_callback, qos_profile_sensor_data)
		self.node.create_subscription(UInt16, 'link/goodput', self.link_goodput_callback, qos_profile_system_default)
		self.node.create_subscription(UInt8, 'link/quality', self.link_quality_callback, qos_profile_system_default)
		self.node.create_subscription(Int8, 'link/rssi', self.link_rssi_callback, qos_profile_system_default)
		self.node.create_subscription(UInt8, 'battery/percentage', self.battery_percentage_callback, qos_profile_system_default)
		self.node.create_subscription(String, 'drone/state', self.drone_state_callback, qos_profile_system_default)
		self.node.create_subscription(Vector3Stamped, 'drone/rpy', self.drone_rpy_callback, qos_profile_sensor_data)
		self.node.create_subscription(QuaternionStamped, 'camera/attitude', self.camera_attitude_callback, qos_profile_sensor_data)
		self.node.create_subscription(Float32, 'camera/exposure_time', self.camera_exposure_time_callback, qos_profile_system_default)
		self.node.create_subscription(UInt16, 'camera/iso_gain', self.camera_iso_gain_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/awb_r_gain', self.camera_awb_r_gain_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/awb_b_gain', self.camera_awb_b_gain_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/hfov', self.camera_hfov_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/vfov', self.camera_vfov_callback, qos_profile_system_default)
		self.node.create_subscription(Bool, 'drone/gps/fix', self.drone_gps_fix_callback, qos_profile_system_default)
		self.node.create_subscription(Bool, 'drone/steady', self.drone_steady_callback, qos_profile_sensor_data)
		self.node.create_subscription(UInt8, 'battery/health', self.battery_health_callback, qos_profile_system_default)
		self.node.create_subscription(SkycontrollerCommand, 'skycontroller/command', self.skycontroller_command_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/zoom', self.camera_zoom_callback, qos_profile_system_default)
		self.node.create_subscription(UInt8, 'drone/gps/satellites', self.drone_gps_satellites_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'drone/altitude_above_to', self.drone_altitude_above_to_callback, qos_profile_sensor_data)
		self.node.create_subscription(Vector3Stamped, 'drone/rpy_slow', self.drone_rpy_slow_callback, qos_profile_sensor_data)
		self.node.create_subscription(NavSatFix, 'drone/gps/location', self.drone_gps_location_callback, qos_profile_sensor_data)
		self.node.create_subscription(Float32, 'battery/voltage', self.battery_voltage_callback, qos_profile_system_default)
		self.node.create_subscription(TargetTrajectory, 'target/trajectory', self.target_trajectory_callback, qos_profile_system_default)
		self.node.create_subscription(Vector3Stamped, 'gimbal/relative', self.gimbal_relative_callback, qos_profile_sensor_data)
		self.node.create_subscription(Vector3Stamped, 'gimbal/absolute', self.gimbal_absolute_callback, qos_profile_sensor_data)
		self.node.create_subscription(UInt64, 'media/available', self.media_available_callback, qos_profile_system_default)
		self.node.create_subscription(PointStamped, 'home/location',  self.home_location_callback, qos_profile_system_default)
		self.node.create_subscription(QuaternionStamped, 'skycontroller/attitude', self.skycontroller_attitude_callback, qos_profile_sensor_data)
		self.node.create_subscription(Vector3Stamped, 'skycontroller/rpy', self.skycontroller_rpy_callback, qos_profile_sensor_data)

		# Services
		self.drone_arm_client = self.node.create_client(SetBool, 'drone/arm')
		self.drone_takeoff_client = self.node.create_client(Trigger, 'drone/takeoff')
		self.drone_land_client = self.node.create_client(Trigger, 'drone/land')
		self.drone_emergency_client = self.node.create_client(Trigger, 'drone/emergency')
		self.drone_halt_client = self.node.create_client(Trigger, 'drone/halt')
		self.drone_rth_client = self.node.create_client(Trigger, 'drone/rth')
		self.drone_reboot_client = self.node.create_client(Trigger, 'drone/reboot')
		self.drone_calibrate_client = self.node.create_client(Trigger, 'drone/calibrate')
		self.skycontroller_offboard_client = self.node.create_client(SetBool, 'skycontroller/offboard')
		self.skycontroller_discover_drones_client = self.node.create_client(Trigger, 'skycontroller/discover_drones')
		self.skycontroller_forget_drone_client = self.node.create_client(Trigger, 'skycontroller/forget_drone')
		self.home_set_client = self.node.create_client(Location, 'home/set')
		self.home_navigate_client = self.node.create_client(SetBool, 'home/navigate')
		self.POI_start_client = self.node.create_client(PilotedPOI, 'POI/start')
		self.POI_stop_client = self.node.create_client(Trigger, 'POI/stop')
		self.flightplan_upload_client = self.node.create_client(FlightPlan, 'flightplan/upload')
		self.flightplan_start_client = self.node.create_client(FlightPlan, 'flightplan/start')
		self.flightplan_pause_client = self.node.create_client(Trigger, 'flightplan/pause')
		self.flightplan_stop_client = self.node.create_client(Trigger, 'flightplan/stop')
		self.followme_start_client = self.node.create_client(FollowMe, 'followme/start')
		self.followme_stop_client = self.node.create_client(Trigger, 'followme/stop')
		self.gimbal_reset_client = self.node.create_client(Trigger, 'gimbal/reset')
		self.gimbal_calibrate_client = self.node.create_client(Trigger, 'gimbal/calibrate')
		self.camera_reset_client = self.node.create_client(Trigger, 'camera/reset')
		self.camera_photo_take_client = self.node.create_client(Trigger, 'camera/photo/take')
		self.camera_photo_stop_client = self.node.create_client(Trigger, 'camera/photo/stop')
		self.camera_recording_start_client = self.node.create_client(Trigger, 'camera/recording/start')
		self.camera_recording_stop_client = self.node.create_client(Trigger, 'camera/recording/stop')
		self.storage_download_client = self.node.create_client(Trigger, 'storage/download')
		self.storage_format_client = self.node.create_client(Trigger, 'storage/format')

		# Dynamic parameters
		self.node.declare_parameter("max_tilt", 10.0)
		self.node.declare_parameter("max_vertical_speed", 1.0)
		self.node.declare_parameter("max_horizontal_speed", 1.0)
		self.node.declare_parameter("max_yaw_rotation_speed", 180.0)
		self.node.declare_parameter("max_pitch_roll_rotation_speed", 200.0)
		self.node.declare_parameter("max_distance", 10.0)
		self.node.declare_parameter("max_altitude", 2.0)
		self.node.declare_parameter("banked_turn", True)
		self.node.declare_parameter("camera_operated", False)
		self.node.declare_parameter("home_type", 1)
		self.node.declare_parameter("rth_min_altitude", 20.0)
		self.node.declare_parameter("rth_ending_behavior", 1)
		self.node.declare_parameter("hovering_altitude", 10.0)
		self.node.declare_parameter("rth_autotrigger", True)
		self.node.declare_parameter("precise_home", True)
		self.node.declare_parameter("hdr", True)
		self.node.declare_parameter("camera_mode", 0)
		self.node.declare_parameter("ev_compensation", 9)
		self.node.declare_parameter("streaming_mode", 0)
		self.node.declare_parameter("image_style", 0)
		self.node.declare_parameter("max_zoom_speed", 10.0)
		self.node.declare_parameter("photo_mode", 0)
		self.node.declare_parameter("photo_format", 1)
		self.node.declare_parameter("file_format", 0)
		self.node.declare_parameter("autorecord", False)
		self.node.declare_parameter("recording_mode", 0)
		self.node.declare_parameter("download_folder", "~/Pictures/Anafi")
		self.node.declare_parameter("cut_media", True)
		self.node.declare_parameter("gimbal_absolute", False)
		self.node.declare_parameter("max_gimbal_speed", 180.0)

		# Variables
		self.camera_image = np.nan
		self.camera_camera_info = np.nan
		self.time = np.nan
		self.drone_attitude = np.nan
		self.drone_altitude = np.nan
		self.drone_speed = np.nan
		self.link_goodput = np.nan
		self.link_quality = np.nan
		self.link_rssi = np.nan
		self.battery_percentage = np.nan
		self.drone_state = ""
		self.drone_rpy = np.nan
		self.camera_attitude = np.nan
		self.camera_exposure_time = np.nan
		self.camera_iso_gain = np.nan
		self.camera_awb_r_gain = np.nan
		self.camera_awb_b_gain = np.nan
		self.camera_hfov = np.nan
		self.camera_vfov = np.nan
		self.drone_gps_fix = np.nan
		self.drone_steady = np.nan
		self.battery_health = np.nan
		self.skycontroller_command = np.nan
		self.camera_zoom = np.nan
		self.drone_gps_satellites = np.nan
		self.drone_altitude_above_to = np.nan
		self.drone_rpy_slow = np.nan
		self.drone_gps_location = np.nan
		self.battery_voltage = np.nan
		self.target_trajectory = np.nan
		self.gimbal_relative = np.nan
		self.gimbal_absolute = np.nan
		self.media_available = np.nan
		self.home_location = np.nan
		self.skycontroller_attitude = np.nan
		self.skycontroller_rpy = np.nan

		self.test_thread = threading.Thread(target=self.tests)
		self.test_thread.start()

	def tests(self):
		if self.skycontroller_connection_test():
			self.skycontroller_test()

		if not self.drone_connection_test():
			print(colored("Impossible establish connection with the drone", 'magenta'))
			exit()

		self.camera_test()

	def skycontroller_connection_test(self):
		success = False
		for i in range(100):
			if self.drone_state != "" and self.drone_state != "CONNECTING":
				success = True
				break
			time.sleep(0.1)
		verdict("Skycontroller connection", success, "drone_state == " + self.drone_state)
		return success

	def skycontroller_test(self):
		success = False
		return success

	def drone_connection_test(self):
		success = False
		for i in range(10):
			if self.drone_state != "" and self.drone_state != "CONNECTING" and self.drone_state != "SERCHING_DRONE":
				success = True
				break
			time.sleep(0.1)
		verdict("Drone connection", success, "drone_state == " + self.drone_state)
		return success

	def camera_test(self):
		# Test zoom level
		self.msg_camera_command = CameraCommand()
		self.msg_camera_command.mode = b'\x00'
		self.msg_camera_command.zoom = 2.0
		success = False
		for i in range(100):
			self.pub_camera_command.publish(self.msg_camera_command)
			if self.camera_zoom == 2.0:
				success = True
				break
			time.sleep(0.1)
		verdict("Camera zoom level", success, "camera_zoom == " + str(self.camera_zoom))

		# Test zoom velocity
		self.msg_camera_command.mode = b'\x01'
		self.msg_camera_command.zoom = -1.0
		success1 = False
		for i in range(100):
			self.pub_camera_command.publish(self.msg_camera_command)
			if self.camera_zoom == 1.0:
				success1 = True
				break
			time.sleep(0.1)
		self.msg_camera_command.zoom = 1.0
		success2 = False
		for i in range(100):
			self.pub_camera_command.publish(self.msg_camera_command)
			if self.camera_zoom > 2.0:
				success2 = True
				break
			time.sleep(0.1)
		success = success1 and success2
		verdict("Camera zoom velocity", success, "camera_zoom == " + str(self.camera_zoom))

		# Test camera reset
		self.req = Trigger.Request()
		self.future = self.camera_reset_client.call_async(self.req)
		success = False
		for i in range(100):
			if self.camera_zoom == 1.0:
				success = True
				break
			time.sleep(0.1)
		verdict("Camera reset", success, "camera_zoom == " + str(self.camera_zoom))

	def camera_image_callback(self, msg):
		self.camera_image = True

	def camera_camera_info_callback(self, msg):
		self.camera_camera_info = True

	def time_callback(self, msg):
		self.time = msg.sec + msg.nanosec/1e9

	def drone_attitude_callback(self, msg):
		pass

	def drone_altitude_callback(self, msg):
		pass

	def drone_speed_callback(self, msg):
		pass

	def link_goodput_callback(self, msg):
		pass

	def link_quality_callback(self, msg):
		pass

	def link_rssi_callback(self, msg):
		pass

	def battery_percentage_callback(self, msg):
		pass

	def drone_state_callback(self, msg):
		self.drone_state = msg.data

	def drone_rpy_callback(self, msg):
		pass

	def camera_attitude_callback(self, msg):
		pass

	def camera_exposure_time_callback(self, msg):
		pass

	def camera_iso_gain_callback(self, msg):
		pass

	def camera_awb_r_gain_callback(self, msg):
		pass

	def camera_awb_b_gain_callback(self, msg):
		pass

	def camera_hfov_callback(self, msg):
		pass

	def camera_vfov_callback(self, msg):
		pass

	def drone_gps_fix_callback(self, msg):
		pass

	def drone_steady_callback(self, msg):
		pass

	def battery_health_callback(self, msg):
		pass

	def skycontroller_command_callback(self, msg):
		pass

	def camera_zoom_callback(self, msg):
		self.camera_zoom = msg.data

	def drone_gps_satellites_callback(self, msg):
		pass

	def drone_altitude_above_to_callback(self, msg):
		pass

	def drone_rpy_slow_callback(self, msg):
		pass

	def drone_gps_location_callback(self, msg):
		pass

	def battery_voltage_callback(self, msg):
		pass

	def target_trajectory_callback(self, msg):
		pass

	def gimbal_relative_callback(self, msg):
		pass

	def gimbal_absolute_callback(self, msg):
		pass

	def media_available_callback(self, msg):
		pass

	def home_location_callback(self, msg):
		pass

	def skycontroller_attitude_callback(self, msg):
		self.skycontroller_attitude = msg

	def skycontroller_rpy_callback(self, msg):
		pass

def verdict(test, success, explanation=""):
	if success:
		print(colored(test + " -> PASS", 'green'))
	else:
		print(colored(test + " -> FAIL" + ("" if explanation=="" else " (" + explanation + ")"), 'red'))

def main(args=None):
	rclpy.init(args=sys.argv)

	test_anafi = TestAnafi()

	rclpy.spin(test_anafi.node)

	test_anafi.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
