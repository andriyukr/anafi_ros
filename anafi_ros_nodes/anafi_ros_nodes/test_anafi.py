#!/usr/bin/env python3

# ros2 run anafi_ros_nodes test_anafi --ros-args -r __ns:=/anafi

import numpy as np
import rclpy
import math
import threading
import time
import datetime
import sys

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
from anafi_ros_interfaces.msg import PilotingCommand, MoveByCommand, MoveToCommand, CameraCommand, GimbalCommand, SkycontrollerCommand, TargetTrajectory
from anafi_ros_interfaces.srv import PilotedPOI, FlightPlan, FollowMe, Location, Photo, Recording, String as StringSRV

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
		self.node.create_subscription(Vector3Stamped, 'gimbal/attitude/relative', self.gimbal_relative_callback, qos_profile_sensor_data)
		self.node.create_subscription(Vector3Stamped, 'gimbal/attitude/absolute', self.gimbal_absolute_callback, qos_profile_sensor_data)
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
		self.camera_photo_take_client = self.node.create_client(Photo, 'camera/photo/take')
		self.camera_photo_stop_client = self.node.create_client(Trigger, 'camera/photo/stop')
		self.camera_recording_start_client = self.node.create_client(Recording, 'camera/recording/start')
		self.camera_recording_stop_client = self.node.create_client(Recording, 'camera/recording/stop')
		self.storage_download_client = self.node.create_client(Trigger, 'storage/download')
		self.storage_format_client = self.node.create_client(Trigger, 'storage/format')
		self.get_parameters_client = self.node.create_client(GetParameters, 'anafi/get_parameters')
		self.set_parameters_client = self.node.create_client(SetParameters, 'anafi/set_parameters')
		while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
			self.node.get_logger().info('No connection...')

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
		time.sleep(2)

		# if self.skycontroller_connection_test():
		# 	self.skycontroller_test()

		# if not self.drone_connection_test():
		# 	print(colored("Impossible establish connection with the drone", 'magenta'))
		# 	exit()

		#self.camera_test()
		self.gimbal_test()
		#self.drone__takeoff_test()
		#self.drone_land_test()
		#self.drone_rth_test()
		#self.drone_param_set_get()

	def skycontroller_connection_test(self):
		print("\n***** SKYCONTROLLER CONNECTION TEST *****")

		print("Skycontroller connection", end=" ", flush=True)
		success = False
		for i in range(100):
			if self.drone_state != "" and self.drone_state != "CONNECTING":
				success = True
				break
			time.sleep(0.1)
		verdict(success, "", "drone_state == " + self.drone_state)
		return success

	def skycontroller_test(self):
		success = False
		return success

	def drone_connection_test(self):
		print("\n***** DRONE CONNECTION TEST *****")

		print("Drone connection", end=" ", flush=True)
		success = False
		for i in range(10):
			if self.drone_state != "" and self.drone_state != "CONNECTING" and self.drone_state != "CONNECTED_SKYCONTROLLER" and self.drone_state != "SERCHING_DRONE":
				success = True
				break
			time.sleep(0.1)
		verdict(success, "", "drone_state == " + self.drone_state)
		return success

	def drone__takeoff_test(self):
		#takeoff test
		print("drone takeoff test", end=" ", flush=True)
		req = Trigger.Request()
		timer_start = timer()
		to_altitude=self.drone_altitude
		future = self.drone_takeoff_client.call_async(req)
		success = False
		for i in range(100):
			if self.drone_state == "HOVERING" and (self.drone_altitude-to_altitude>0.0):
				success = True 
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start), "drone_state == " + self.drone_state)
			
	def drone_land_test(self):		
		print("drone landing test", end=" ", flush=True)
		req = Trigger.Request()
		timer_start = timer()
		future = self.drone_land_client.call_async(req)
		success = False
		for i in range(100):
			if self.drone_state == "LANDED" and np.abs(self.drone_altitude-0.0) < 0.1:
				success = True 
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start), "drone_state == " + self.drone_state)
			
	def drone_param_set_get(self):

		#test set/get max altitude
		print("Set/get 'drone max altitude' parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['drone/max_altitude']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_altitude = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_altitude', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_altitude/2 + 0.1))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		req = GetParameters.Request()
		req.names = ['drone/max_altitude']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_altitude/2 + 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

        #set max altitude to default
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_altitude', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=2.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)		
			

		#test set/get max_horizontal_speed
		print("Set/get 'drone max_horizontal_speed' parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['drone/max_horizontal_speed']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_horizontal_speed = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_horizontal_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_horizontal_speed/2 + 0.1))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		req = GetParameters.Request()
		req.names = ['drone/max_horizontal_speed']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_horizontal_speed/2 + 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

        #set max_horizontal_speed to default
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_horizontal_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)


        #test set/get max_pitch_roll_rate
		print("Set/get 'max_pitch_roll_rate' parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['drone/max_pitch_roll_rate']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_pitch_roll_rate = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_pitch_roll_rate', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_pitch_roll_rate/2 + 0.1))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		req = GetParameters.Request()
		req.names = ['drone/max_pitch_roll_rate']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_pitch_roll_rate/2 + 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

        #set max_pitch_roll_rate to default
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_pitch_roll_rate', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=200.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)


		#test set/get max_vertical_speed
		print("Set/get max_vertical_speed parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['drone/max_vertical_speed']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_vertical_speed = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_vertical_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_vertical_speed/2 + 0.1))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		req = GetParameters.Request()
		req.names = ['drone/max_vertical_speed']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_vertical_speed/2 + 0.1:
				success = True 
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

        #set max_vertical_speed to default
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_vertical_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)


		#test set/get max_distance
		print("Set/get 'max_distance' parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['drone/max_distance']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_distance = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_distance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_distance + 5.0))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		
		req = GetParameters.Request()
		req.names = ['drone/max_distance']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_distance + 5.0:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

        #set max_distance to default
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_distance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)


        #test set/get max_yaw_rate
		print("Set/get 'max_yaw_rate' parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['drone/max_yaw_rate']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_yaw_rate = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_yaw_rate', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_yaw_rate/2 + 0.1))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		req = GetParameters.Request()
		req.names = ['drone/max_yaw_rate']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_yaw_rate/2 + 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

        #set max_yaw_rate to default
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_yaw_rate', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=180.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
	

		#test set/get max_pitch_roll
		print("Set/get 'drone max_pitch_roll' parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['drone/max_pitch_roll']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_pitch_roll = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_pitch_roll', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_pitch_roll/2 + 0.1))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		req = GetParameters.Request()
		req.names = ['drone/max_pitch_roll']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_pitch_roll/2 + 0.1:
				success = True
				break
			time.sleep(0.1)			
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

        #set max_pitch_roll to default
		req = SetParameters.Request()
		req.parameters = [Parameter(name='drone/max_pitch_roll', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
	    

	def drone_rth_test(self):
		print("drone rth+moveby +potential halt test", end=" ", flush=True)
		to_location=(round(self.drone_gps_location.latitude,2), round(self.drone_gps_location.longitude,2) , round(self.drone_gps_location.altitude,2))
		msg_moveby_command = MoveByCommand()
		msg_moveby_command.dx = 1.0
		msg_moveby_command.dy = 4.0
		msg_moveby_command.dz = 5.0
		msg_moveby_command.dyaw = 0.0
		success = False
		timer_start = timer()
		for i in range(100):
			self.pub_drone_moveby.publish(msg_moveby_command)
			time.sleep(0.1)
		new_loc=(round(self.drone_gps_location.latitude,2), round(self.drone_gps_location.longitude,2) , round(self.drone_gps_location.altitude,2))
		print(new_loc)
		#verdict(success, "duration = %.3fs" % (timer() - timer_start),  )


	def camera_test(self):
		print("\n***** CAMERA TEST *****")

		# Test set/get 'max_zoom_speed'
		print("Set/get 'max_zoom_speed' parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['camera/max_zoom_speed']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_zoom_speed = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='camera/max_zoom_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_zoom_speed/2 + 0.1))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		req = GetParameters.Request()
		req.names = ['camera/max_zoom_speed']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_zoom_speed/2 + 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

		# Set max zoom speed
		req = SetParameters.Request()
		req.parameters = [Parameter(name='camera/max_zoom_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)

		# Test zoom level
		print("Camera zoom level", end=" ", flush=True)
		print("current zoom level is " , self.camera_zoom)
		msg_camera_command = CameraCommand()
		msg_camera_command.mode = 0
		msg_camera_command.zoom = 2.0
		success = False
		timer_start = timer()
		for i in range(100):
			self.pub_camera_command.publish(msg_camera_command)
			if np.abs(self.camera_zoom - 2.0) < 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "camera_zoom == %.3fX" % self.camera_zoom)

		# Test zoom velocity
		print("Camera zoom velocity", end=" ", flush=True)
		msg_camera_command = CameraCommand()
		msg_camera_command.mode = 1
		msg_camera_command.zoom = -1.0
		success1 = False
		timer_start = timer()
		for i in range(100):
			self.pub_camera_command.publish(msg_camera_command)
			if np.abs(self.camera_zoom - 1.0) < 0.1:
				success1 = True
				break
			time.sleep(0.1)
		msg_camera_command.zoom = 1.0
		success2 = False
		for i in range(100):
			self.pub_camera_command.publish(msg_camera_command)
			if self.camera_zoom > 2.0:
				success2 = True
				break
			time.sleep(0.1)
		success = success1 and success2
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "camera_zoom == %.3fX" % self.camera_zoom)

		# Test camera reset
		print("Camera reset", end=" ", flush=True)
		req = Trigger.Request()
		timer_start = timer()
		future = self.camera_reset_client.call_async(req)
		success = False
		for i in range(100):
			if np.abs(self.camera_zoom - 1.0) < 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "camera_zoom == %.3fX" % self.camera_zoom)

		# Reset max zoom speed
		req = SetParameters.Request()
		req.parameters = [Parameter(name='max_zoom_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_zoom_speed))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)

		# Test photo take
		print("Take photo", end=" ", flush=True)
		req = Photo.Request()
		timer_start = timer()
		future = self.camera_photo_take_client.call_async(req)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().media_id != '':
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

		# Test video recording
		print("Video recording", end=" ", flush=True)
		req = Recording.Request()
		self.camera_recording_start_client.call_async(req)
		time.sleep(1.0)
		timer_start = timer()
		future = self.camera_recording_stop_client.call_async(req)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().media_id != '':
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

	def gimbal_test(self):
		print("\n***** GIMBAL TEST *****")

		# Test set/get 'max_gimbal_speed'
		print("Set/get 'max_gimbal_speed' parameter", end=" ", flush=True)
		req = GetParameters.Request()
		req.names = ['gimbal/max_speed']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		max_gimbal_speed = future.result().values[0].double_value
		req = SetParameters.Request()
		req.parameters = [Parameter(name='gimbal/max_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_gimbal_speed/2 + 1.1))]
		timer_start = timer()
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		req = GetParameters.Request()
		req.names = ['gimbal/max_speed']
		future = self.get_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)
		success = False
		for i in range(100):
			if future.result() is not None and future.result().values[0].double_value == max_gimbal_speed/2 + 1.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

		# Set max gimbal speed
		req = SetParameters.Request()
		req.parameters = [Parameter(name='gimbal/max_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=180.0))]
		future = self.set_parameters_client.call_async(req)
		while not future.done():
			time.sleep(0.1)

		# Test gimbal calibration
		print("Gimbal calibration", end=" ", flush=True)
		req = Trigger.Request()
		timer_start = timer()
		future = self.gimbal_calibrate_client.call_async(req)
		success = False
		for i in range(100):
			if future.result() is not None:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

		# Test gimbal relative roll orientation
		print("Gimbal relative roll orientation", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 1
		msg_gimbal_command.roll = 20.0
		success = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if np.abs(self.gimbal_relative.vector.x - 20.0) < 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_roll == %.3fdeg != %.3fdeg" % (self.gimbal_relative.vector.x, msg_gimbal_command.roll))

		# Test gimbal relative roll velocity
		print("Gimbal relative roll velocity", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 1
		msg_gimbal_command.frame = 1
		msg_gimbal_command.roll = 1.0
		success1 = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_relative.vector.x > 20.0:
				success1 = True
				break
			time.sleep(0.1)
		msg_gimbal_command.roll = -1.0
		success2 = False
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_relative.vector.x < -20.0:
				success2 = True
				break
			time.sleep(0.1)
		success = success1 and success2
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_roll == %.3fdeg" % self.gimbal_relative.vector.x)

		# Test gimbal absolute roll orientation
		print("Gimbal absolute roll orientation", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 2
		msg_gimbal_command.roll = -20.0
		success = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if np.abs(self.gimbal_absolute.vector.x + 20.0) < 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_roll == %.3fdeg" % self.gimbal_absolute.vector.x)

		# Test gimbal absolute roll velocity
		print("Gimbal absolute roll velocity", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 1
		msg_gimbal_command.frame = 2
		msg_gimbal_command.roll = 1.0
		success1 = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_absolute.vector.x > 20.0:
				success1 = True
				break
			time.sleep(0.1)
		msg_gimbal_command.roll = -1.0
		success2 = False
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_absolute.vector.x < -20.0:
				success2 = True
				break
			time.sleep(0.1)
		success = success1 and success2
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_roll == %.3fdeg" % self.gimbal_absolute.vector.x)

		# Test gimbal relative pitch orientation
		print("Gimbal relative pitch orientation", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 1
		msg_gimbal_command.roll = 0.0
		msg_gimbal_command.pitch = 20.0
		success = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if np.abs(self.gimbal_relative.vector.y - 20.0) < 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_pitch == %.3fdeg" % self.gimbal_relative.vector.y)

		# Test gimbal relative pitch velocity
		print("Gimbal relative pitch velocity", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 1
		msg_gimbal_command.frame = 1
		msg_gimbal_command.pitch = 1.0
		success1 = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_relative.vector.y > 20.0:
				success1 = True
				break
			time.sleep(0.1)
		msg_gimbal_command.pitch = -1.0
		success2 = False
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_relative.vector.y < -20.0:
				success2 = True
				break
			time.sleep(0.1)
		success = success1 and success2
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_pitch == %.3fdeg" % self.gimbal_relative.vector.y)

		# Test gimbal absolute pitch orientation
		print("Gimbal absolute pitch orientation", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 2
		msg_gimbal_command.pitch = -20.0
		success = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if np.abs(self.gimbal_absolute.vector.y + 20.0) < 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_pitch == %.3fdeg" % self.gimbal_absolute.vector.y)

		# Test gimbal absolute pitch velocity
		print("Gimbal absolute pitch velocity", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 1
		msg_gimbal_command.frame = 2
		msg_gimbal_command.pitch = 1.0
		success1 = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_absolute.vector.y > 20.0:
				success1 = True
				break
			time.sleep(0.1)
		msg_gimbal_command.pitch = -1.0
		success2 = False
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_absolute.vector.y < -20.0:
				success2 = True
				break
			time.sleep(0.1)
		success = success1 and success2
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_pitch == %.3fdeg" % self.gimbal_absolute.vector.y)

		# Test gimbal relative yaw orientation
		print("Gimbal relative yaw orientation", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 1
		msg_gimbal_command.pitch = 0.0
		msg_gimbal_command.yaw = 20.0
		success = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if np.abs(self.gimbal_relative.vector.y - 20.0) < 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_yaw == %.3fdeg" % self.gimbal_relative.vector.z)

		# Test gimbal relative yaw velocity
		print("Gimbal relative yaw velocity", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 1
		msg_gimbal_command.frame = 1
		msg_gimbal_command.yaw = 1.0
		success1 = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_relative.vector.z > 20.0:
				success1 = True
				break
			time.sleep(0.1)
		msg_gimbal_command.yaw = -1.0
		success2 = False
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_relative.vector.z < -20.0:
				success2 = True
				break
			time.sleep(0.1)
		success = success1 and success2
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_yaw == %.3fdeg" % self.gimbal_relative.vector.z)

		# Test gimbal absolute yaw orientation
		print("Gimbal absolute yaw orientation", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 2
		msg_gimbal_command.yaw = -20.0
		success = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if np.abs(self.gimbal_absolute.vector.z + 20.0) < 0.1:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_yaw == %.3fdeg" % self.gimbal_absolute.vector.z)

		# Test gimbal absolute yaw velocity
		print("Gimbal absolute yaw velocity", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 1
		msg_gimbal_command.frame = 2
		msg_gimbal_command.yaw = 1.0
		success1 = False
		timer_start = timer()
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_absolute.vector.y > 20.0:
				success1 = True
				break
			time.sleep(0.1)
		msg_gimbal_command.yaw = -1.0
		success2 = False
		for i in range(100):
			self.pub_gimbal_command.publish(msg_gimbal_command)
			if self.gimbal_absolute.vector.z < -20.0:
				success2 = True
				break
			time.sleep(0.1)
		success = success1 and success2
		verdict(success, "duration = %.3fs" % (timer() - timer_start),  "gimbal_yaw == %.3fdeg" % self.gimbal_absolute.vector.z)

		# Test gimbal reset
		print("Gimbal reset", end=" ", flush=True)
		msg_gimbal_command = GimbalCommand()
		msg_gimbal_command.mode = 0
		msg_gimbal_command.frame = 1
		msg_gimbal_command.roll = 20.0
		msg_gimbal_command.pitch = 20.0
		msg_gimbal_command.yaw = 20.0
		self.pub_gimbal_command.publish(msg_gimbal_command)
		req = Trigger.Request()
		timer_start = timer()
		future = self.gimbal_reset_client.call_async(req)
		success = False
		for i in range(100):
			if future.result() is not None:
				success = True
				break
			time.sleep(0.1)
		verdict(success, "duration = %.3fs" % (timer() - timer_start))

	def camera_image_callback(self, msg):
		self.camera_image = True

	def camera_camera_info_callback(self, msg):
		self.camera_camera_info = True

	def time_callback(self, msg):
		self.time = msg.sec + msg.nanosec/1e9

	def drone_attitude_callback(self, msg):
		pass

	def drone_altitude_callback(self, msg):
		self.drone_altitude=msg.data

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
		self.drone_gps_location=msg

	def battery_voltage_callback(self, msg):
		pass

	def target_trajectory_callback(self, msg):
		pass

	def gimbal_relative_callback(self, msg):
		self.gimbal_relative = msg

	def gimbal_absolute_callback(self, msg):
		self.gimbal_absolute = msg

	def media_available_callback(self, msg):
		pass

	def home_location_callback(self, msg):
		pass

	def skycontroller_attitude_callback(self, msg):
		self.skycontroller_attitude = msg

	def skycontroller_rpy_callback(self, msg):
		pass

def verdict(success, explanation_pass="", explanation_fail=""):
	if success:
		print("-> " + colored("PASS", 'green') + ("" if explanation_pass=="" else " (" + explanation_pass + ")"))
	else:
		print("-> " + colored("FAIL", 'red') + ("" if explanation_fail=="" else " (" + explanation_fail + ")"))

def main(args=None):
	rclpy.init(args=sys.argv)

	test_anafi = TestAnafi()

	rclpy.spin(test_anafi.node)

	test_anafi.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
