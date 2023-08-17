#!/usr/bin/env python3

import numpy as np
import rclpy
import csv
import cv2
import math
import os
import requests
import queue
import shlex
import subprocess
import tempfile
import threading
import traceback
import time
import datetime
import logging
import logness
import sys
import yaml
import olympe
import anafi_ros_nodes

from timeit import default_timer as timer
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data, qos_profile_services_default, qos_profile_parameters, qos_profile_parameter_events
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, SetParametersResult
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import UInt8, UInt16, Int8, Float32, String, Header, Bool
from geometry_msgs.msg import PointStamped, QuaternionStamped, Vector3Stamped
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge
from olympe.messages import gimbal, camera, mapper, move, leds, thermal, obstacle_avoidance
from olympe.messages.drone_manager import connection_state
from olympe.messages.ardrone3.Piloting import TakeOff, UserTakeOff, Landing, Emergency, PCMD, NavigateHome
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt, MaxDistance, MaxAltitude, NoFlyOverMaxDistance, BankedTurn
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged, MaxDistanceChanged, MaxAltitudeChanged, NoFlyOverMaxDistanceChanged, BankedTurnChanged
from olympe.messages.ardrone3.SpeedSettings import MaxVerticalSpeed, MaxRotationSpeed, MaxPitchRollRotationSpeed
from olympe.messages.ardrone3.SpeedSettingsState import MaxVerticalSpeedChanged, MaxRotationSpeedChanged, MaxPitchRollRotationSpeedChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.piloting_style import set_style
from olympe.messages.controller_info import validity_from_drone
from olympe.messages.precise_home import set_mode as precise_home_set_mode
from olympe.messages.rth import return_to_home, abort, set_ending_behavior, set_ending_hovering_altitude, set_min_altitude
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.mediastore import state as mediastore_state_message
from olympe.media import download_media, indexing_state, delete_media, delete_all_media
from olympe.enums.ardrone3.PilotingState import AlertStateChanged_State, ForcedLandingAutoTrigger_Reason
from olympe.enums.camera import availability, state
from olympe.enums.common import CalibrationState
from olympe.enums.piloting_style import style
from olympe.enums.precise_home import mode as precise_home_mode
from olympe.enums.mediastore import state as mediastore_state_enum
from olympe.enums.obstacle_avoidance import mode as obstacle_avoidance_mode
from olympe.enums.thermal import rendering_mode
from anafi_ros_interfaces.msg import PilotingCommand, MoveByCommand, MoveToCommand, CameraCommand, GimbalCommand
from anafi_ros_interfaces.srv import PilotedPOI, FlightPlan, FollowMe, Location, Photo, Recording, String as StringSRV
from anafi_ros_nodes.event_listener_anafi import EventListenerAnafi
from anafi_ros_nodes.event_listener_skycontroller import EventListenerSkyController
from anafi_ros_nodes.utils import euler_from_quaternion, bound_percentage, quaternion_inverse, rotate_vector, rotate_quaternion

logness.update_config({  # https://developer.parrot.com/docs/olympe/olympeapi.html#olympe.log.update_config
	"handlers": {
		"log_file": {
			"class": "logness.FileHandler",
			"formatter": "default_formatter",
			"filename": "anafi.log"
		}
	},
	"loggers": {
		"olympe": {
			"level": "WARNING",  # {"CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"}
			"handlers": ["console"]
		},
		"ulog": {
			"level": "CRITICAL",  # {"CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"}
			"handlers": ["console"]
		}
	}
})

class Anafi(Node):
	models = {2324:"4k", 2329:"thermal", 2334:"usa", 2330:"ai"}  # correspondence between model's code and model's name

	def __init__(self):
		self.node = rclpy.create_node('anafi')

		self.node.get_logger().info("Anafi is running...")

		# Subscribers
		self.node.create_subscription(CameraCommand, 'camera/command', self.zoom_callback, qos_profile_system_default)
		self.node.create_subscription(PilotingCommand, 'drone/command', self.rpyt_callback, qos_profile_system_default)
		self.node.create_subscription(MoveToCommand, 'drone/moveto', self.moveTo_callback, qos_profile_system_default)
		self.node.create_subscription(MoveByCommand, 'drone/moveby', self.moveBy_callback, qos_profile_system_default)
		self.node.create_subscription(GimbalCommand, 'gimbal/command', self.gimbal_callback, qos_profile_system_default)

		# Publishers
		self.pub_image = self.node.create_publisher(Image, 'camera/image', qos_profile_system_default)
		self.pub_camera_info = self.node.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_system_default)
		self.pub_time = self.node.create_publisher(Time, 'time', qos_profile_sensor_data)
		self.pub_attitude = self.node.create_publisher(QuaternionStamped, 'drone/attitude', qos_profile_sensor_data)
		self.pub_altitude = self.node.create_publisher(Float32, 'drone/altitude', qos_profile_sensor_data)
		self.pub_position = self.node.create_publisher(PointStamped, 'drone/position', qos_profile_system_default)
		self.pub_local_position = self.node.create_publisher(PointStamped, 'drone/position_local', qos_profile_system_default)
		self.pub_speed = self.node.create_publisher(Vector3Stamped, 'drone/speed', qos_profile_sensor_data)
		self.pub_link_goodput = self.node.create_publisher(UInt16, 'link/goodput', qos_profile_system_default)
		self.pub_link_quality = self.node.create_publisher(UInt8, 'link/quality', qos_profile_system_default)
		self.pub_wifi_rssi = self.node.create_publisher(Int8, 'link/rssi', qos_profile_system_default)
		self.pub_battery_percentage = self.node.create_publisher(UInt8, 'battery/percentage', qos_profile_system_default)
		self.pub_state = self.node.create_publisher(String, 'drone/state', qos_profile_system_default)
		self.pub_rpy = self.node.create_publisher(Vector3Stamped, 'drone/rpy', qos_profile_sensor_data)
		self.pub_gimbal_attitude = self.node.create_publisher(QuaternionStamped, 'gimbal/attitude', qos_profile_sensor_data)
		self.pub_gimbal_rpy = self.node.create_publisher(Vector3Stamped, 'gimbal/rpy', qos_profile_sensor_data)
		self.pub_exposure_time = self.node.create_publisher(Float32, 'camera/exposure_time', qos_profile_system_default)
		self.pub_iso_gain = self.node.create_publisher(UInt16, 'camera/iso_gain', qos_profile_system_default)
		self.pub_awb_r_gain = self.node.create_publisher(Float32, 'camera/awb_r_gain', qos_profile_system_default)
		self.pub_awb_b_gain = self.node.create_publisher(Float32, 'camera/awb_b_gain', qos_profile_system_default)
		self.pub_hfov = self.node.create_publisher(Float32, 'camera/hfov', qos_profile_system_default)
		self.pub_vfov = self.node.create_publisher(Float32, 'camera/vfov', qos_profile_system_default)
		self.pub_gps_fix = self.node.create_publisher(Bool, 'drone/gps/fix', qos_profile_system_default)
		self.pub_steady = self.node.create_publisher(Bool, 'drone/steady', qos_profile_sensor_data)
		self.pub_battery_health = self.node.create_publisher(UInt8, 'battery/health', qos_profile_system_default)

		# Services
		self.node.create_service(SetBool, 'drone/arm', self.arm_callback)
		self.node.create_service(Trigger, 'drone/takeoff', self.takeoff_callback)
		self.node.create_service(Trigger, 'drone/land', self.land_callback)
		self.node.create_service(Trigger, 'drone/emergency', self.emergency_callback)
		self.node.create_service(Trigger, 'drone/halt', self.halt_callback)
		self.node.create_service(Trigger, 'drone/rth', self.rth_callback)
		self.node.create_service(Trigger, 'drone/reboot', self.reboot_callback)
		self.node.create_service(Trigger, 'drone/calibrate', self.calibrate_magnetometer_callback)
		self.node.create_service(SetBool, 'skycontroller/offboard', self.offboard_callback)
		self.node.create_service(Trigger, 'skycontroller/discover_drones', self.discover_drones_callback)
		self.node.create_service(Trigger, 'skycontroller/forget_drone', self.forget_drone_callback)
		self.node.create_service(Location, 'home/set', self.set_home_callback)
		self.node.create_service(SetBool, 'home/navigate', self.navigate_home_callback)
		self.node.create_service(PilotedPOI, 'POI/start', self.start_piloted_POI_callback)
		self.node.create_service(Trigger, 'POI/stop', self.stop_piloted_POI_callback)
		self.node.create_service(FlightPlan, 'flightplan/upload', self.flightplan_upload_callback)
		self.node.create_service(FlightPlan, 'flightplan/start', self.flightplan_start_callback)
		self.node.create_service(Trigger, 'flightplan/pause', self.flightplan_pause_callback)
		self.node.create_service(Trigger, 'flightplan/stop', self.flightplan_stop_callback)
		self.node.create_service(FollowMe, 'followme/start', self.followme_start_callback)
		self.node.create_service(Trigger, 'followme/stop', self.followme_stop_callback)
		self.node.create_service(Trigger, 'gimbal/reset', self.reset_gimbal_callback)
		self.node.create_service(Trigger, 'gimbal/calibrate', self.calibrate_gimbal_callback)
		self.node.create_service(Trigger, 'camera/reset', self.reset_zoom_callback)
		self.node.create_service(Photo, 'camera/photo/take', self.take_photo_callback)
		self.node.create_service(Photo, 'camera/photo/stop', self.stop_photo_callback)
		self.node.create_service(Recording, 'camera/recording/start', self.start_recording_callback)
		self.node.create_service(Recording, 'camera/recording/stop', self.stop_recording_callback)
		self.node.create_service(SetBool, 'storage/download', self.download_media_callback)
		self.node.create_service(Trigger, 'storage/format', self.format_callback)

		# Messages
		self.msg_camera_info = CameraInfo()
		self.msg_state = String()
		self.header = Header()
		self.msg_time = Time()
		self.msg_attitude = QuaternionStamped()
		self.msg_rpy = Vector3Stamped()
		self.msg_ground_distance = Float32()
		self.msg_position = PointStamped()
		self.msg_local_position = PointStamped()
		self.msg_speed = Vector3Stamped()
		self.msg_battery_percentage = UInt8()
		self.msg_exposure_time = Float32()
		self.msg_iso_gain = UInt16()
		self.msg_awb_r_gain = Float32()
		self.msg_awb_b_gain = Float32()
		self.msg_hfov = Float32()
		self.msg_vfov = Float32()
		self.msg_goodput = UInt16()
		self.msg_quality = UInt8()
		self.msg_rssi = Int8()
		self.msg_steady = Bool()
		self.msg_home_location = PointStamped()
		self.msg_battery_health = UInt8()
		self.msg_gps_fix = Bool()

		# Parameters from the launch file
		self.model = self.node.declare_parameter('drone/model', '4k').value
		self.ip = self.node.declare_parameter('device/ip', '192.168.53.1', ParameterDescriptor(read_only=True)).value
		self.drone_serial = self.node.declare_parameter('drone/serial', '', ParameterDescriptor(read_only=True)).value
		self.wifi_key = self.node.declare_parameter('link/wifi_key', '', ParameterDescriptor(read_only=True)).value
		self.rest_api_version = self.node.declare_parameter('storage/rest_api_version', 1, ParameterDescriptor(read_only=True)).value
		self.node.declare_parameter("drone/offboard", False,  ParameterDescriptor(description="Enable offboard or manual control"))

		if self.ip == '192.168.42.1':  # direct WiFi connection to the drone
			self.skycontroller_enabled = False
			self.simulation_environment = False
		else:
			if self.ip.startswith('192.168.'):  # connection through SkyController
				self.skycontroller_enabled = True
				self.simulation_environment = False
			else:
				if self.ip == '10.202.0.1':  # connection to the simulated drone
					if self.model in {'4k', 'ai'}:
						self.skycontroller_enabled = False
						self.simulation_environment = True
					else:
						self.node.get_logger().fatal("This model is not supported in simulation")
				else:  # not valid IP address
					self.node.get_logger().fatal("IP address is not valid")

		if self.skycontroller_enabled:  # connect to SkyController
			self.node.get_logger().info("Connecting through SkyController")
			self.drone = olympe.SkyController(self.ip) # self.drone = olympe.Drone(self.ip)
			#self.skyctrl = self.drone  # TODO: remove
		else:  # connect to Anafi
			self.node.get_logger().info("Connecting directly to Anafi")
			self.drone = olympe.Drone(self.ip)

		# Create event listeners
		self.event_listener_anafi = EventListenerAnafi(self)
		self.event_listener_skycontroller = EventListenerSkyController(self)

		self.bridge = CvBridge()  # to convert OpenCV images to ROS images

		self.state = 'LANDED'
		self.gps_fixed = False
		self.offboard = False

		self.connect()

		if self.skycontroller_enabled:  # only SkyController allows to retrieve the model of the drone
			#model = self.drone.get_state(olympe.messages.drone_manager.known_drone_item).popitem(last=False)[1]['model'] # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.known_drone_item
			model = self.drone.get_state(olympe.messages.mapper.active_product)['product'] # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.active_product
			self.model = self.models.get(model, self.model)

		self.node.add_on_set_parameters_callback(self.parameter_callback)

		# Dynamic parameters
		self.node.declare_parameter("drone/max_pitch_roll", 10.0,  # 10.0
									ParameterDescriptor(description="Max pitch/roll (in deg) [1.0, 40.0]",
														floating_point_range=[FloatingPointRange(from_value=1.0,
																								 to_value=40.0,
																								 step=0.0)]))
		self.node.declare_parameter("drone/max_vertical_speed", 1.0,  # 3.0
									ParameterDescriptor(description="Max vertical speed (in m/s) [0.1, 4.0]",
														floating_point_range=[FloatingPointRange(from_value=0.1,
																								 to_value=4.0,
																								 step=0.0)]))
		self.node.declare_parameter("drone/max_horizontal_speed", 1.0,  #
									ParameterDescriptor(description="Max horizontal speed (in m/s) [0.1, 15.0]",
														floating_point_range=[FloatingPointRange(from_value=0.1,
																								 to_value=15.0,
																								 step=0.0)]))
		self.node.declare_parameter("drone/max_yaw_rate", 180.0,  # 40.0
									ParameterDescriptor(description="Max yaw rate (in deg/s) [3.0, 200.0]",
														floating_point_range=[FloatingPointRange(from_value=3.0,
																								 to_value=200.0,
																								 step=0.0)]))
		self.node.declare_parameter("drone/max_pitch_roll_rate", 300.0,  # 116.0
									ParameterDescriptor(description="Max pitch and roll rate (in deg/s) [40.0, 300.0]",
														floating_point_range=[FloatingPointRange(from_value=40.0,
																								 to_value=300.0,
																								 step=0.0)]))
		self.node.declare_parameter("drone/max_distance", 10.0,  # 100.0
									ParameterDescriptor(description="Max distance (in m) [10.0, 4000.0]",
														floating_point_range=[FloatingPointRange(from_value=10.0,
																								 to_value=4000.0,
																								 step=0.0)]))
		self.node.declare_parameter("drone/max_altitude", 2.0,  # 150.0
									ParameterDescriptor(description="Max altitude (in m) [0.5, 4000.0]",
														floating_point_range=[FloatingPointRange(from_value=0.5,
																								 to_value=4000.0,
																								 step=0.0)]))
		self.node.declare_parameter("drone/banked_turn", False,  # False
									ParameterDescriptor(description="Enable banked turn"))
		self.node.declare_parameter("drone/camera_operated", False,  # False
									ParameterDescriptor(description="Commands are relative to the camera pitch"))
		self.node.declare_parameter("home/type", 4,   # 1
									ParameterDescriptor(description="Home type for RTH: 1 = return to the last takeoff location; 3 = return to a user-set custom location; 4 = return to the pilot position",
														integer_range=[IntegerRange(from_value=1,
																					to_value=4,
																					step=1)]))
		self.node.declare_parameter("home/min_altitude", 20.0,  # 20.0
									ParameterDescriptor(description="RTH minimum altitude (in m) [20.0, 100.0]",
														floating_point_range=[FloatingPointRange(from_value=20.0,
																								 to_value=100.0,
																								 step=0.0)]))
		self.node.declare_parameter("home/ending_behavior", 1,  # 0
									ParameterDescriptor(description="Ending behavior for RTH: 0 = land after RTH; 1 = hover after RTH",
														integer_range=[IntegerRange(from_value=0,
																					to_value=1,
																					step=1)]))
		self.node.declare_parameter("home/hovering_altitude", 10.0,  # 10.0
									ParameterDescriptor(description="RTH ending hovering altitude (in m) [1.0, 10.0]",
														floating_point_range=[FloatingPointRange(from_value=1.0,
																								 to_value=10.0,
																								 step=0.0)]))
		self.node.declare_parameter("home/autotrigger", True,  # True
									ParameterDescriptor(description="Enable auto trigger RTH"))
		self.node.declare_parameter("home/precise", True,  # False
									ParameterDescriptor(description="Enable precise RTH"))
		self.node.declare_parameter("camera/hdr", True,  # False
									ParameterDescriptor(description="Enable HDR"))
		self.node.declare_parameter("camera/mode", 0,  # 0
									ParameterDescriptor(description="Camera mode: 0 = camera in recording mode, 1 = camera in photo mode",
														integer_range=[IntegerRange(from_value=0,
																					to_value=1,
																					step=1)]))
		self.node.declare_parameter("camera/ev_compensation", 9,  # 3
									ParameterDescriptor(description="EV compensation: 0 = -3.00 EV; 3 = -2.00 EV; 6 = -1.00 EV; 9 = 0.00 EV; 12 = 1.00 EV; 15 = 2.00 EV; 18 = 3.00 EV",
														integer_range=[IntegerRange(from_value=0,
																				    to_value=18,
																				    step=1)]))
		self.node.declare_parameter("camera/streaming", 0,  # 0
									ParameterDescriptor(description="Streaming mode: 0 = minimize latency with average reliability (best for piloting); 1 = maximize the reliability with an average latency; 2 = maximize the reliability using a framerate decimation with an average latency",
														integer_range=[IntegerRange(from_value=0,
																					to_value=2,
																					step=1)]))
		self.node.declare_parameter("camera/style", 0,  # 0
									ParameterDescriptor(description="Images style: 0 = natural look style; 1 = produces flat and desaturated images, best for post-processing; 2 = intense style: bright colors, warm shade, high contrast; 3 = pastel style: soft colors, cold shade, low contrast",
														integer_range=[IntegerRange(from_value=0,
																					to_value=3,
																					step=1)]))
		self.node.declare_parameter("camera/max_zoom_speed", 10.0,  # 0.34
									ParameterDescriptor(description="Max zoom speed (in tan(deg)/sec) [0.1, 10.0]",
														floating_point_range=[FloatingPointRange(from_value=0.1,
																								 to_value=10.0,
																								 step=0.0)]))
		self.node.declare_parameter("camera/autorecord", False,  # True
									ParameterDescriptor(description="Enable autorecord at takeoff"))
		self.node.declare_parameter("storage/download_folder", "~/Pictures/Anafi",
									ParameterDescriptor(description="Absolute path to the download folder"))
		self.node.declare_parameter("gimbal/max_speed", 180.0,  #
									ParameterDescriptor(description="Max gimbal speed (in deg/s) [1.0, 180.0]",
														floating_point_range=[FloatingPointRange(from_value=1.0,
																								 to_value=180.0,
																								 step=0.0)]))
		if self.model in {'thermal', 'usa'}:
			self.node.declare_parameter("camera/thermal/rendering", 0,  # 0
										ParameterDescriptor(
											description="Thermal image rendering mode: 0 = visible; 1 = thermal; 2: blended",
											integer_range=[IntegerRange(from_value=0,
																		to_value=2,
																		step=1)]))
			self.node.declare_parameter("camera/thermal/emissivity", 1.0,  # 0.9
										ParameterDescriptor(description="Thermal emissivity [0.0, 1.0]",
															floating_point_range=[FloatingPointRange(from_value=0.0,
																									 to_value=1.0,
																									 step=0.0)]))
		if self.model in {'ai'}:
			self.node.declare_parameter("camera/stereo/disparity_map", False,  # False
										ParameterDescriptor(description="Enable stream disparity map image"))
			self.node.declare_parameter("camera/stereo/obstacle_avoidance", True,  # True
										ParameterDescriptor(description="Enable obstacle avoidance"))

		self.node.get_logger().info('Drone model: ' + self.model)
		self.node.set_parameters([Parameter('drone/model', rclpy.Parameter.Type.STRING, self.model)])
		self.node.declare_parameter('drone/name', self.model + "_" + self.ip, ParameterDescriptor(description="Drones name (model + IP)", read_only=True))  # TODO: replace IP with sequence number; make this parameter read only
		
		if self.model in {'4k', 'thermal', 'usa', 'ai'}:
			with open(get_package_share_directory('anafi_ros_nodes') + "/camera_" + self.model + ".yaml", "r") as file_handle:  # load camera info from file
				camera_info = yaml.load(file_handle, Loader=yaml.FullLoader)
			self.node.get_logger().info("Camera info loaded from " + os.path.abspath(os.path.dirname(__file__) + "/../../param/camera_" + self.model + ".yaml"))
			# Parse camera info		
			self.msg_camera_info.width = camera_info['image_width']
			self.msg_camera_info.height = camera_info['image_height']
			self.msg_camera_info.distortion_model = camera_info['distortion_model']
			self.msg_camera_info.k = camera_info['camera_matrix']['data']
			self.msg_camera_info.d = camera_info['distortion_coefficients']['data']
			self.msg_camera_info.r = camera_info['rectification_matrix']['data']
			self.msg_camera_info.p = camera_info['projection_matrix']['data']
		else:
			self.node.get_logger().fatal("Model is not supported")

		self.drone(camera.set_antiflicker_mode(mode="auto")) # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_antiflicker_mode
		self.drone(camera.set_white_balance(cam_id=0, mode="automatic", temperature="t_8000")) # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_white_balance
		self.drone(camera.set_zoom_velocity_quality_degradation(cam_id=0, allow=1)) # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_zoom_velocity_quality_degradation
		self.drone(NoFlyOverMaxDistance(shouldNotFlyOver=1)) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettings.NoFlyOverMaxDistance
		self.drone(olympe.messages.user_storage.start_monitoring(period=1)) # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.start_monitoring
		self.drone(gimbal.set_target(gimbal_id=0, control_mode='position', yaw_frame_of_reference='relative', yaw=0.0, pitch_frame_of_reference='relative', pitch=0.0, roll_frame_of_reference='relative', roll=0.0)) # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.set_target

		if self.skycontroller_enabled:
			self.drone(olympe.messages.skyctrl.Calibration.enableMagnetoCalibrationQualityUpdates(enable=1))  # https://developer.parrot.com/docs/olympe/arsdkng_skyctrl_calibration.html#olympe.messages.skyctrl.Calibration.enableMagnetoCalibrationQualityUpdates
		
		self.node.get_logger().debug('Drone name: %s' % (str(self.drone.get_state(olympe.messages.common.SettingsState.ProductNameChanged)['name'])))  # https://developer.parrot.com/docs/olympe/arsdkng_common_settings.html#olympe.messages.common.SettingsState.ProductNameChanged
		self.serial = self.drone.get_state(olympe.messages.common.SettingsState.ProductSerialHighChanged)['high'] + self.drone.get_state(olympe.messages.common.SettingsState.ProductSerialLowChanged)['low'] #
		self.node.get_logger().debug('Drone serial: %s' % self.serial)
		drone_version = self.drone.get_state(olympe.messages.common.SettingsState.ProductVersionChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_common_settings.html#olympe.messages.common.SettingsState.ProductVersionChanged
		self.node.get_logger().debug('Drone version: software: %s, hardware: %s' % (drone_version['software'], drone_version['hardware']))
		flights_status = self.drone.get_state(olympe.messages.ardrone3.SettingsState.MotorFlightsStatusChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_settings_state.html#olympe.messages.ardrone3.SettingsState.MotorErrorStateChanged
		self.node.get_logger().info('Number of flights: %i' % (flights_status['nbFlights']))
		self.node.get_logger().info('Total flight duration: %s' % (str(datetime.timedelta(seconds=flights_status['totalFlightDuration']))))
		if not self.simulation_environment:
			self.node.get_logger().debug('Battery serial: %s' % (self.drone.get_state(olympe.messages.battery.serial)['serial']))  # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.serial
		self.node.get_logger().info('Battery cycle count: %i' % (self.drone.get_state(olympe.messages.battery.cycle_count)['count']))  # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.cycle_count
		self.node.get_logger().info('Battery health: %i%%' % (self.drone.get_state(olympe.messages.battery.health)['state_of_health']))  # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.health
		max_tilt = self.drone.get_state(olympe.messages.ardrone3.PilotingSettingsState.MaxTiltChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettingsState.MaxTiltChanged
		self.node.get_logger().debug('MaxTilt = %f [%f, %f]' % (max_tilt["current"], max_tilt["min"], max_tilt["max"]))
		max_vertical_speed = self.drone.get_state(olympe.messages.ardrone3.SpeedSettingsState.MaxVerticalSpeedChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettingsState.MaxVerticalSpeedChanged
		self.node.get_logger().debug('MaxVerticalSpeed = %f [%f, %f]' % (max_vertical_speed["current"], max_vertical_speed["min"], max_vertical_speed["max"]))
		max_yaw_rate = self.drone.get_state(olympe.messages.ardrone3.SpeedSettingsState.MaxRotationSpeedChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettingsState.MaxRotationSpeedChanged
		self.node.get_logger().debug('MaxYawRotationSpeed = %f [%f, %f]' % (max_yaw_rate["current"], max_yaw_rate["min"], max_yaw_rate["max"]))
		max_pitch_roll_rate = self.drone.get_state(olympe.messages.ardrone3.SpeedSettingsState.MaxPitchRollRotationSpeedChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettingsState.MaxPitchRollRotationSpeedChanged
		self.node.get_logger().debug('MaxPitchRollRotationSpeed = %f [%f, %f]' % (max_pitch_roll_rate["current"], max_pitch_roll_rate["min"], max_pitch_roll_rate["max"]))
		max_distance = self.drone.get_state(olympe.messages.ardrone3.PilotingSettingsState.MaxDistanceChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettingsState.MaxDistanceChanged
		self.node.get_logger().debug('MaxDistance = %f [%f, %f]' % (max_distance["current"], max_distance["min"], max_distance["max"]))
		max_altitude = self.drone.get_state(olympe.messages.ardrone3.PilotingSettingsState.MaxAltitudeChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettingsState.MaxAltitudeChanged
		self.node.get_logger().debug('MaxAltitude = %f [%f, %f]' % (max_altitude["current"], max_altitude["min"], max_altitude["max"]))
		self.node.get_logger().debug('NoFlyOverMaxDistance = %i' % (self.drone.get_state(olympe.messages.ardrone3.PilotingSettingsState.NoFlyOverMaxDistanceChanged)["shouldNotFlyOver"]))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettingsState.NoFlyOverMaxDistanceChanged
		self.node.get_logger().debug('BankedTurn = %i' % (self.drone.get_state(olympe.messages.ardrone3.PilotingSettingsState.BankedTurnChanged)["state"]))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettingsState.BankedTurnChanged
		absolute_attitude_bounds = self.drone.get_state(olympe.messages.gimbal.absolute_attitude_bounds)[0]  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.absolute_attitude_bounds
		self.node.get_logger().debug('GimbalAbsoluteBounds (deg): roll=[%f, %f], pitch=[%f, %f], yaw=[%f, %f]' % (-absolute_attitude_bounds["max_roll"], -absolute_attitude_bounds["min_roll"], -absolute_attitude_bounds["max_pitch"], -absolute_attitude_bounds["min_pitch"], -absolute_attitude_bounds["max_yaw"], -absolute_attitude_bounds["min_yaw"]))
		max_speed = self.drone.get_state(olympe.messages.gimbal.max_speed)[0]  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.max_speed
		self.node.get_logger().debug('GimbalMaxSpeed (deg/s): roll=%f [%f, %f], pitch=%f [%f, %f], yaw=%f [%f, %f]' % (max_speed["current_roll"], max_speed["min_bound_roll"], max_speed["max_bound_roll"], max_speed["current_pitch"], max_speed["min_bound_pitch"], max_speed["max_bound_pitch"], max_speed["current_yaw"], max_speed["min_bound_yaw"], max_speed["max_bound_yaw"]))

		if self.model in {'4k', 'thermal', 'usa'}:
			self.node.get_logger().debug('Camera states: %s' % (str(self.drone.get_state(olympe.messages.camera.camera_states))))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.camera_states
			self.node.get_logger().debug('Camera capabilities: %s' % (str(self.drone.get_state(olympe.messages.camera.camera_capabilities))))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.camera_capabilities
			self.node.get_logger().debug('Recording capabilities: %s' % (str(self.drone.get_state(olympe.messages.camera.recording_capabilities))))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.recording_capabilities

			relative_attitude_bounds = self.drone.get_state(olympe.messages.gimbal.relative_attitude_bounds)[0]  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.relative_attitude_bounds
			self.node.get_logger().debug('GimbalRelativeBounds (deg): roll=[%f, %f], pitch=[%f, %f], yaw=[%f, %f]' % (-relative_attitude_bounds["max_roll"], -relative_attitude_bounds["min_roll"], -relative_attitude_bounds["max_pitch"], -relative_attitude_bounds["min_pitch"], -relative_attitude_bounds["max_yaw"], -relative_attitude_bounds["min_yaw"]))

		if self.model in {'thermal', 'usa'}:
			self.drone(thermal.set_palette_settings(  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.messages.thermal.set_palette_settings
				mode='relative',  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.enums.thermal.palette_mode
				lowest_temp=303,  # in Kelvin (303K = 30C)
				highest_temp=313,  # in Kelvin (313K = 40C)
				outside_colorization='extended',  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.enums.thermal.colorization_mode
				relative_range='unlocked',  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.enums.thermal.relative_range_mode
				spot_type='hot',  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.enums.thermal.spot_type
				spot_threshold=0.9)).wait()
			self.drone(thermal.set_sensitivity(range='low')).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.messages.thermal.set_sensitivity
			self.drone(thermal.shutter_mode(current_trigger='auto')).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.messages.thermal.shutter_mode

		if self.skycontroller_enabled:
			skyctrl_version = self.drone.get_state(olympe.messages.skyctrl.SettingsState.ProductVersionChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_skyctrl_settings.html#olympe.messages.skyctrl.SettingsState.ProductVersionChanged
			self.node.get_logger().debug('Controller version: software=%s, hardware=%s' % (skyctrl_version['software'], skyctrl_version['hardware']))
			#self.node.get_logger().debug('Validity from drone: %r ' % (self.drone.get_state(validity_from_drone)["is_valid"] == 1))  # https://developer.parrot.com/docs/olympe/arsdkng_controller_info.html#olympe.messages.controller_info.validity_from_drone
		
		mediastore_state = self.drone.get_state(mediastore_state_message)['state']  # https://developer.parrot.com/docs/olympe/arsdkng_mediastore.html#olympe.messages.mediastore.state
		if mediastore_state == mediastore_state_enum.not_available:
			self.node.get_logger().error('Mediastore state is not available')
		else:
			if mediastore_state == mediastore_state_enum.indexing:
				self.node.get_logger().warning('Mediastore state is indexig')
			else:
				self.node.get_logger().info('Mediastore state is indexed')

		# Setup the callback functions to do some live video processing
		self.processing_thread = threading.Thread(target=self.yuv_frame_processing)
		self.drone.streaming.set_callbacks(
			raw_cb=self.yuv_frame_cb,
			flush_raw_cb=self.flush_cb)
		self.drone.streaming.start(media_name="DefaultVideo")
		self.processing_thread.start()

	def connect(self):
		while True:
			self.msg_state.data = "CONNECTING"
			self.pub_state.publish(self.msg_state)
			if self.drone_serial != "":
				self.node.get_logger().info("Connecting to %s with %s" % (self.drone_serial, self.wifi_key), once=True)
				self.drone(olympe.messages.drone_manager.connect(  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.connect
					serial=self.drone_serial,
					key=self.wifi_key)).wait()  # TODO: check why sometimes doesn't connect
				if self.drone(connection_state(state="connected", _policy="check")):  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.enums.drone_manager.connection_state
					break
			else:
				if self.drone.connect():
					break
			if not rclpy.ok():
				self.disconnect()
				exit()

		if self.skycontroller_enabled:  # connect to the SkyController
			self.msg_state.data = "CONNECTED_SKYCONTROLLER"
			self.pub_state.publish(self.msg_state)
			self.node.get_logger().info("Connected to SkyController")

			self.switch_manual()
			self.event_listener_skycontroller.subscribe()
			self.timer_skycontroller = self.node.create_timer(0.01, self.event_listener_skycontroller.callback)  # create_timer() fires only after rclpy.spin()
			# self.skycontroller_thread = threading.Thread(target=self.event_listener_skycontroller.callback)
			# self.skycontroller_thread.start()

			while True:  # connect to the drone
				if self.drone(connection_state(state="connected", _policy="check")):
					break				
				if not rclpy.ok():
					self.disconnect()
					exit()
				else:
					self.msg_state.data = "SERCHING_DRONE"
					self.pub_state.publish(self.msg_state)
					self.node.get_logger().info("Connection to Anafi: %s" % self.drone.get_state(connection_state)["state"].name, once=True)
				time.sleep(1.0)
			self.node.get_logger().info("Connection to Anafi: %s" % (self.drone.get_state(connection_state)["state"].name))
		else:  # connect to Anafi
			self.node.get_logger().info("Connected to Anafi")
			self.switch_offboard()
			
		self.msg_state.data = "CONNECTED_DRONE"
		self.pub_state.publish(self.msg_state)

		self.event_listener_anafi.subscribe()
			
		self.node.get_logger().debug('Boot Id: %s' % (self.drone.get_state(olympe.messages.common.CommonState.BootId)['bootId']))  # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.BootId

		self.timer_check = self.node.create_timer(0.01, self.check_callback)
		self.timer_fast = self.node.create_timer(0.01, self.fast_callback)
		self.timer_slow = self.node.create_timer(1.00, self.slow_callback)

		self.frame_queue = queue.Queue(maxsize=1)  # TODO: replace by a shared variable
		
	def disconnect(self):
		if 'self.timer_skycontroller' in vars():
			self.timer_skycontroller.destroy()
			#self.skycontroller_thread.join()
		if 'self.timer_check' in vars():
			self.timer_check.destroy()
			self.timer_fast.destroy()
			self.timer_slow.destroy()
		if 'self.processing_thread' in vars():
			self.processing_thread.join()
			self.drone.streaming.stop()

		self.event_listener_anafi.unsubscribe()
		self.event_listener_skycontroller.unsubscribe()

		self.drone.disconnect()

	def check_callback(self):  # checks for the connection
		if not self.drone.connection_state():
			self.node.get_logger().fatal('Drone disconnected!!!')
			self.msg_state.data = "DISCONNECTING"
			self.pub_state.publish(self.msg_state)
			self.disconnect()
			self.msg_state.data = "DISCONNECTED"
			self.pub_state.publish(self.msg_state)

			self.node.get_logger().info('Reconnecting to the drone...')
			self.connect()

	def fast_callback(self):  # fast states triggered on changes
		# FIXME: check on steady state
		#motion_state = self.drone.get_state(olympe.messages.ardrone3.PilotingState.MotionState)['state']  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.MotionState  # FIXME: check why it fails
		#self.msg_steady.data = (self.state == "LANDED" and motion_state == olympe.enums.ardrone3.PilotingState.MotionState_State.steady)
		#self.pub_steady.publish(self.msg_steady)
		pass

	def slow_callback(self):  # slow states triggered on changes
		self.gps_fixed = bool(self.drone.get_state(GPSFixStateChanged)['fixed'])  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_gps.html#olympe.messages.ardrone3.GPSSettingsState.GPSFixStateChanged
		self.msg_gps_fix.data = self.gps_fixed
		self.pub_gps_fix.publish(self.msg_gps_fix)

		self.msg_battery_health.data = self.drone.get_state(olympe.messages.battery.health)['state_of_health']  # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.health
		self.pub_battery_health.publish(self.msg_battery_health)

	def parameter_callback(self, parameters):
		for parameter in parameters:
			# piloting related
			if parameter.name == 'drone/max_pitch_roll':
				self.max_tilt = parameter.value
				self.drone(MaxTilt(self.max_tilt))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html?#olympe.messages.ardrone3.PilotingSettings.MaxTilt
				self.node.get_logger().debug("Parameter 'drone/max_pitch_roll' set to %.1f (deg)" % self.max_tilt)
			if parameter.name == 'drone/max_vertical_speed':
				self.max_vertical_speed = parameter.value
				self.drone(MaxVerticalSpeed(self.max_vertical_speed))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettings.MaxVerticalSpeed
				self.node.get_logger().debug("Parameter 'drone/max_vertical_speed' set to %.1f (m/s)" % self.max_vertical_speed)
			if parameter.name == 'drone/max_horizontal_speed':
				self.max_horizontal_speed = parameter.value
				self.node.get_logger().debug("Parameter 'drone/max_horizontal_speed' set to %.1f (m/s)" % self.max_horizontal_speed)
			if parameter.name == 'drone/max_yaw_rate':
				self.max_yaw_rate = parameter.value
				self.drone(MaxRotationSpeed(self.max_yaw_rate))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettings.MaxRotationSpeed
				self.node.get_logger().debug("Parameter 'drone/max_yaw_rate' set to %.1f (deg/s)" % self.max_yaw_rate)
			if parameter.name == 'drone/max_pitch_roll_rate':
				max_pitch_roll_rate = parameter.value
				self.drone(MaxPitchRollRotationSpeed(max_pitch_roll_rate))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettings.MaxPitchRollRotationSpeed
				self.node.get_logger().debug("Parameter 'drone/max_pitch_roll_rate' set to %.1f (deg/s)" % max_pitch_roll_rate)
			if parameter.name == 'drone/max_distance':
				max_distance = parameter.value
				self.drone(MaxDistance(max_distance))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettings.MaxDistance
				self.node.get_logger().debug("Parameter 'drone/max_distance' set to %.1f (m)" % max_distance)
			if parameter.name == 'drone/max_altitude':
				max_altitude = parameter.value
				self.drone(MaxAltitude(max_altitude))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettings.MaxAltitude
				self.node.get_logger().debug("Parameter 'drone/max_altitude' set to %.1f (m)" % max_altitude)
			if parameter.name == 'drone/banked_turn':
				banked_turn = parameter.value
				self.drone(BankedTurn(int(banked_turn)))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettings.BankedTurn
				self.node.get_logger().debug("Parameter 'drone/banked_turn' set to %r" % banked_turn)
			if parameter.name == 'drone/camera_operated':
				camera_operated = parameter.value
				self.drone(set_style(style=style(int(camera_operated))))  # https://developer.parrot.com/docs/olympe/arsdkng_piloting_style.html#olympe.messages.piloting_style.set_style
				self.node.get_logger().debug("Parameter 'drone/camera_operated' set to %r" % camera_operated)
			if parameter.name == 'drone/offboard':
				offboard = parameter.value
				if offboard:
					self.switch_offboard()
				else:
					
					self.switch_manual()
					if not self.skycontroller_enabled:
						return SetParametersResult(successful=False, reason="Cannot swith to manual control without Skycontroller!")
				
			# RTH related
			if parameter.name == 'home/type':
				home_type = parameter.value
				self.drone(olympe.messages.rth.set_preferred_home_type(  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.set_preferred_home_type
					type=olympe.enums.rth.home_type(home_type)))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.home_type
				self.node.get_logger().debug("Parameter 'home/type' set to '%s'" % olympe.enums.rth.home_type(home_type))
			if parameter.name == 'home/min_altitude':
				rth_min_altitude = parameter.value
				self.drone(set_min_altitude(  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.set_min_altitude
					altitude=rth_min_altitude))  # ATO altitude (m)
				self.node.get_logger().debug("Parameter 'home/min_altitude' set to %.1f (m)" % rth_min_altitude)
			if parameter.name == 'home/ending_behavior':
				rth_ending_behavior = parameter.value
				self.drone(set_ending_behavior(  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.set_ending_behavior
					ending_behavior=olympe.enums.rth.ending_behavior(rth_ending_behavior)))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.ending_behavior
				self.node.get_logger().debug("Parameter 'home/ending_behavior' set to '%s'" % olympe.enums.rth.ending_behavior(rth_ending_behavior))
			if parameter.name == 'home/hovering_altitude':
				hovering_altitude = parameter.value
				self.drone(set_ending_hovering_altitude(  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.set_ending_hovering_altitude
					altitude=hovering_altitude))  # AGL altitude (m)
				self.node.get_logger().debug("Parameter 'home/hovering_altitude' set to %.1f (m)" % hovering_altitude)
			if parameter.name == 'home/autotrigger':
				rth_autotrigger = parameter.value
				self.drone(olympe.messages.rth.set_auto_trigger_mode(  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.set_auto_trigger_mode
					mode=olympe.enums.rth.auto_trigger_mode(rth_autotrigger)))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.auto_trigger_mode
				self.node.get_logger().debug("Parameter 'home/autotrigger' set to '%s'" % olympe.enums.rth.auto_trigger_mode(rth_autotrigger))
			if parameter.name == 'home/precise':
				precise_home = parameter.value
				self.drone(precise_home_set_mode(  # https://developer.parrot.com/docs/olympe/arsdkng_precise_home.html#olympe.messages.precise_home.set_mode
					mode=precise_home_mode(precise_home)))  # https://developer.parrot.com/docs/olympe/arsdkng_precise_home.html#olympe.enums.precise_home.mode
				self.node.get_logger().debug("Parameter 'home/precise' set to '%s'" % precise_home_mode(precise_home))

			# camera related
			if parameter.name == 'camera/mode':
				self.camera_mode = parameter.value
				self.drone(camera.set_camera_mode(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_camera_mode
					cam_id=0,
					value=olympe.enums.camera.camera_mode(self.camera_mode)))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.camera_mode
				self.node.get_logger().debug("Parameter 'camera/mode' set to '%s'" % olympe.enums.camera.camera_mode(self.camera_mode))
			if parameter.name == 'camera/hdr':
				hdr = parameter.value
				self.drone(camera.set_hdr_setting(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_hdr_setting
					cam_id=0,
					value=olympe.enums.camera.state(int(hdr))))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.state FIXME: CHECK for a potential bug
				self.node.get_logger().debug("Parameter 'camera/hdr' set to '%s'" % olympe.enums.camera.state(int(hdr)))
			if parameter.name == 'camera/ev_compensation':
				ev_compensation = parameter.value
				self.drone(camera.set_ev_compensation(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_ev_compensation
					cam_id=0,
					value=olympe.enums.camera.ev_compensation(ev_compensation)))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.ev_compensation
				self.node.get_logger().debug("Parameter 'camera/ev_compensation' set to '%s'" % olympe.enums.camera.ev_compensation(ev_compensation))
			if parameter.name == 'camera/streaming':
				streaming_mode = parameter.value
				self.drone(camera.set_streaming_mode(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_streaming_mode
					cam_id=0,
					value=olympe.enums.camera.streaming_mode(streaming_mode)))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.streaming_mode
				self.node.get_logger().debug("Parameter 'camera/streaming' set to '%s'" % olympe.enums.camera.streaming_mode(streaming_mode))
			if parameter.name == 'camera/style':
				image_style = parameter.value
				self.drone(camera.set_style(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_style
					cam_id=0,
					style=olympe.enums.camera.style(image_style)))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.style
				self.node.get_logger().debug("Parameter 'camera/style' set to '%s'" % olympe.enums.camera.style(image_style))
			if parameter.name == 'camera/max_zoom_speed':
				max_zoom_speed = parameter.value
				self.drone(camera.set_max_zoom_speed(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_max_zoom_speed
					cam_id=0,
					max=max_zoom_speed))  # [0.01, 10] (tan(deg)/sec)
				self.node.get_logger().debug("Parameter 'camera/max_zoom_speed' set to %.1f (tan(deg)/sec)" % max_zoom_speed)
			if parameter.name == 'camera/autorecord':
				autorecord = parameter.value
				self.drone(camera.set_autorecord(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_autorecord
					cam_id=0,
					state=olympe.enums.camera.state(autorecord)))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.state
				self.node.get_logger().debug("Parameter 'camera/autorecord' set to '%s'" % olympe.enums.camera.state(autorecord))

			# thermal related (ANAFI Thermal and ANAFI USA)
			if parameter.name == 'camera/thermal/rendering' and self.model in {'thermal', 'usa'}:
				self.thermal_rendering = parameter.value
				self.drone(thermal.set_rendering(  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.messages.thermal.set_rendering
					mode=rendering_mode(int(self.thermal_rendering)),  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.enums.thermal.rendering_mode
					blending_rate=1.0)).wait()
				self.change_thermal_rendering_thread = threading.Thread(target=self.change_thermal_rendering)  # to prevent rqt_reconfigure timeout warning
				self.change_thermal_rendering_thread.start()
				self.node.get_logger().debug("Parameter 'camera/thermal/rendering' set to '%s'" % rendering_mode(int(self.thermal_rendering)))
			if parameter.name == 'camera/thermal/emissivity' and self.model in {'thermal', 'usa'}:
				emissivity = parameter.value
				self.drone(thermal.set_emissivity(emissivity=emissivity)).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.messages.thermal.set_emissivity
				self.node.get_logger().debug("Parameter 'camera/thermal/emissivity' set to %.1f" % emissivity)

			# stereo related (ANAFI Ai)
			if parameter.name == 'camera/stereo/obstacle_avoidance' and self.model in {'ai'}:
				obstacle_avoidance_enabled = parameter.value
				self.drone(obstacle_avoidance.set_mode(  # TODO: make it parametric. https://developer.parrot.com/docs/olympe/arsdkng_obstacle_avoidance.html#olympe.messages.obstacle_avoidance.set_mode
					mode=obstacle_avoidance_mode(int(obstacle_avoidance_enabled))))  # https://developer.parrot.com/docs/olympe/arsdkng_obstacle_avoidance.html#olympe.enums.obstacle_avoidance.mode
				self.node.get_logger().debug("Parameter 'camera/stereo/obstacle_avoidance' set to '%s'" % obstacle_avoidance_mode(int(obstacle_avoidance_enabled)))
			if parameter.name == 'camera/stereo/disparity_map' and self.model in {'ai'} and not self.simulation_environment:  # FIXME: check why if does not work in simulation
				self.disparity_map = parameter.value
				self.drone.streaming.stop()
				self.drone.streaming.start(media_name=("Disparity map" if self.disparity_map else "Front camera"))
				self.node.get_logger().debug("Parameter 'camera/stereo/disparity_map' set to %r" % self.disparity_map)

			# media related
			if parameter.name == 'storage/download_folder':
				self.download_folder = os.path.expanduser(parameter.value)
				self.node.get_logger().debug("Parameter 'storage/download_folder' set to '%s'" % self.download_folder)

			# gimbal related
			if parameter.name == 'gimbal/max_speed 	':
				max_gimbal_speed = parameter.value
				self.drone(gimbal.set_max_speed(  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.max_speed
					gimbal_id=0,
					yaw=0,
					pitch=max_gimbal_speed,  # [1, 180] (deg/s)
					roll=max_gimbal_speed))  # [1, 180] (deg/s)
				self.node.get_logger().debug("Parameter 'gimbal/max_speed 	' set to %.1f deg" % max_gimbal_speed)

		return SetParametersResult(successful=True)

	def change_thermal_rendering(self):
		if self.thermal_rendering == 0 and self.drone.get_state(olympe.messages.thermal.mode)['mode'].value != 0:
			self.drone.streaming.stop()
			self.drone(thermal.set_mode(mode='disabled')).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.messages.thermal.set_mode
			self.drone.streaming.start(media_name="DefaultVideo")
		if self.thermal_rendering in {1, 2} and self.drone.get_state(olympe.messages.thermal.mode)['mode'].value == 0:
			self.drone.streaming.stop()
			self.drone(thermal.set_mode(mode='blended')).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_thermal.html#olympe.messages.thermal.set_mode
			self.drone.streaming.start(media_name="ParrotThermalBlendedVideo")

	def yuv_frame_cb(self, yuv_frame):  # this function will be called by Olympe for each decoded YUV frame
		yuv_frame.ref()
		if self.frame_queue.full():
			self.frame_queue.get_nowait().unref()
		self.frame_queue.put_nowait(yuv_frame)

	def flush_cb(self, stream):
		if stream["vdef_format"] != olympe.VDEF_I420:
			return True
		while not self.frame_queue.empty():
			self.frame_queue.get_nowait().unref()
		return True

	def yuv_frame_processing(self):
		while rclpy.ok():
			try:
				yuv_frame = self.frame_queue.get(timeout=0.1)
			except queue.Empty:
				continue

			info = yuv_frame.info()  # the VideoFrame.info() dictionary contains some useful information such as the video resolution
			self.node.get_logger().debug("yuv_frame.info = " + str(info), throttle_duration_sec=10)

			vmeta = yuv_frame.vmeta()  # yuv_frame.vmeta() returns a dictionary that contains additional metadata from the drone
			self.node.get_logger().debug("yuv_frame.vmeta = " + str(vmeta), throttle_duration_sec=10)

			# TODO: check "has_errors" and "is_silent" flags for possible errors (https://forum.developer.parrot.com/t/yuv-frame-vmeta-returns-empty-object/16958/7)
			if vmeta[1] != {}:
				self.header.stamp = self.node.get_clock().now().to_msg()

				timestamp = info['raw']['frame']['timestamp']
				timescale = info['raw']['frame']['timescale']
				self.msg_time.sec = int(timestamp//timescale)
				self.msg_time.nanosec = int((timestamp%timescale)*(1e9/timescale))
				self.pub_time.publish(self.msg_time)

				drone_quat = vmeta[1]['drone']['quat']  # attitude
				self.msg_attitude.header = self.header
				self.msg_attitude.header.frame_id = '/world'
				self.msg_attitude.quaternion.x = drone_quat['x']
				self.msg_attitude.quaternion.y = -drone_quat['y']
				self.msg_attitude.quaternion.z = -drone_quat['z']
				self.msg_attitude.quaternion.w = drone_quat['w']
				self.pub_attitude.publish(self.msg_attitude)

				(roll, pitch, yaw) = euler_from_quaternion(self.msg_attitude.quaternion)
				self.msg_rpy.header = self.header
				self.msg_rpy.header.frame_id = '/world'
				self.msg_rpy.vector.x = math.degrees(roll)
				self.msg_rpy.vector.y = math.degrees(pitch)
				self.msg_rpy.vector.z = math.degrees(yaw)
				self.pub_rpy.publish(self.msg_rpy)

				self.msg_ground_distance.data = vmeta[1]['drone']['ground_distance']  # barometer (m)
				self.pub_altitude.publish(self.msg_ground_distance)

				if 'position' in vmeta[1]['drone']:
					position = vmeta[1]['drone']['position']
					self.msg_position.header = self.header
					self.msg_position.header.frame_id = '/world'
					self.msg_position.point.x = position['north']
					self.msg_position.point.y = -position['east']
					self.msg_position.point.z = -position['down']
					self.pub_position.publish(self.msg_position)

				if 'local_position' in vmeta[1]['drone']:
					local_position = vmeta[1]['drone']['local_position']
					self.msg_local_position.header = self.header
					self.msg_local_position.header.frame_id = '/body'
					self.msg_local_position.point.x = local_position['x']
					self.msg_local_position.point.y = local_position['y']
					self.msg_local_position.point.z = local_position['z']
					self.pub_local_position.publish(self.msg_local_position)

				speed = vmeta[1]['drone']['speed']  # optical flow speed (m/s)
				v = [speed['north'], -speed['east'], -speed['down']]
				q = quaternion_inverse([drone_quat['w'], drone_quat['x'], -drone_quat['y'], -drone_quat['z']])
				v = rotate_vector(q, v)
				self.msg_speed.header = self.header
				self.msg_speed.header.frame_id = '/body'
				self.msg_speed.vector.x = v[0]
				self.msg_speed.vector.y = v[1]
				self.msg_speed.vector.z = v[2]
				#self.msg_speed.vector.x =  math.cos(yaw)*speed['north'] - math.sin(yaw)*speed['east']
				#self.msg_speed.vector.y = -math.sin(yaw)*speed['north'] - math.cos(yaw)*speed['east']
				#self.msg_speed.vector.z = -speed['down']
				self.pub_speed.publish(self.msg_speed)

				battery_percentage = vmeta[1]['drone']['battery_percentage']  # [0=empty, 100=full]
				self.msg_battery_percentage.data = battery_percentage
				self.pub_battery_percentage.publish(self.msg_battery_percentage)
				if battery_percentage%10 == 0:
					self.node.get_logger().info("Battery level: %s%%" % str(battery_percentage), throttle_duration_sec=100)

				gimbal_quat = vmeta[1]['camera']['quat']  # gimbal absolute attitude
				self.msg_attitude.header = self.header
				self.msg_attitude.header.frame_id = '/world'
				self.msg_attitude.quaternion.x = gimbal_quat['x']
				self.msg_attitude.quaternion.y = -gimbal_quat['y']
				self.msg_attitude.quaternion.z = -gimbal_quat['z']
				self.msg_attitude.quaternion.w = gimbal_quat['w']
				self.pub_gimbal_attitude.publish(self.msg_attitude)

				(gimbal_roll, gimbal_pitch, gimbal_yaw) = euler_from_quaternion(self.msg_attitude.quaternion)
				self.msg_rpy.header = self.header
				self.msg_rpy.header.frame_id = '/world'
				self.msg_rpy.vector.x = math.degrees(gimbal_roll)
				self.msg_rpy.vector.y = math.degrees(gimbal_pitch)
				self.msg_rpy.vector.z = math.degrees(gimbal_yaw)
				self.pub_gimbal_rpy.publish(self.msg_rpy)
				
				self.msg_exposure_time.data = vmeta[1]['camera']['exposure_time']
				self.pub_exposure_time.publish(self.msg_exposure_time)

				self.msg_iso_gain.data = vmeta[1]['camera']['iso_gain']
				self.pub_iso_gain.publish(self.msg_iso_gain)

				self.msg_awb_r_gain.data = vmeta[1]['camera']['awb_r_gain']
				self.pub_awb_r_gain.publish(self.msg_awb_r_gain)

				self.msg_awb_b_gain.data = vmeta[1]['camera']['awb_b_gain']
				self.pub_awb_b_gain.publish(self.msg_awb_b_gain)

				self.msg_hfov.data = vmeta[1]['camera']['hfov']
				self.pub_hfov.publish(self.msg_hfov)

				self.msg_vfov.data = vmeta[1]['camera']['vfov']
				self.pub_vfov.publish(self.msg_vfov)

				self.state = vmeta[1]['drone']['flying_state']  # ['FS_LANDED', 'FS_TAKINGOFF', 'FS_HOVERING', 'FS_FLYING', 'FS_LANDING', 'FS_EMERGENCY', 'FS_USER_TAKEOFF', 'FS_MOTOR_RAMPING', 'FS_EMERGENCY_LANDING']
				if self.state.startswith("FS_"):  # in Olympe 7.4, 'FS_' prefix was introduced
					self.state = self.state[3:]  # removes 'FS_' from the state value
				self.msg_state.data = self.state
				self.pub_state.publish(self.msg_state)

				if self.model in {'4k', 'thermal', 'usa'}:
					self.msg_goodput.data = vmeta[1]['links'][0]['wifi']['goodput']  # throughput of the connection (b/s)
					self.pub_link_goodput.publish(self.msg_goodput)

					self.msg_quality.data = vmeta[1]['links'][0]['wifi']['quality']  # [0=bad, 5=good]
					self.pub_link_quality.publish(self.msg_quality)

					rssi = vmeta[1]['links'][0]['wifi']['rssi']  # signal strength [-100=bad, 0=good] (dBm)
					self.msg_rssi.data = rssi
					self.pub_wifi_rssi.publish(self.msg_rssi)

					# log signal strength
					if rssi <= -60:
						if rssi >= -70:
							self.node.get_logger().info("Signal strength: %sdBm" % str(rssi), throttle_duration_sec=100)
						else:
							if rssi >= -80:
								self.node.get_logger().warning("Weak signal: %sdBm" % str(rssi), throttle_duration_sec=10)
							else:
								if rssi >= -90:
									self.node.get_logger().error("Unreliable signal: %sdBm" % str(rssi), throttle_duration_sec=1)
								else:
									self.node.get_logger().fatal("Unusable signal: %sdBm" % str(rssi), throttle_duration_sec=0.1)

				if self.model in {'ai'} and vmeta[1]['links'] != []:  # TODO: check to which network the drone is connected (wlan or cellular)
					self.msg_quality.data = vmeta[1]['links'][0]['starfish']['quality']  # [0=bad, 5=good]
					self.pub_link_quality.publish(self.msg_quality)

				cv2_cvt_color_flag = {  # convert pdraw YUV flag to OpenCV YUV flag
					olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
					olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
				}[yuv_frame.format()]
				cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)  # use OpenCV to convert the yuv frame to RGB
				if self.model in {'thermal', 'usa'} and self.thermal_rendering in {1, 2}:
					cv2frame = cv2.applyColorMap(cv2frame, cv2.COLORMAP_PLASMA)  # to mimic FLIR ironbow colormap
				# if self.model in {'ai'} and self.disparity_map:
				# 	mask = cv2.inRange(cv2frame, np.array([0,0,0]), np.array([254,254,254]))
				# 	mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # 3 channel mask
				# 	#cv2frame = cv2.applyColorMap(cv2frame, cv2.COLORMAP_TURBO)
				# 	cv2frame = cv2.bitwise_and(cv2frame, mask)

				msg_image = self.bridge.cv2_to_imgmsg(cv2frame, "bgr8")
				timestamp = info['ntp_raw_unskewed_timestamp']  # image capture timestamp (ms)
				msg_image.header.stamp.sec = int(timestamp//1e6)
				msg_image.header.stamp.nanosec = int((timestamp%1e6)*1e3)
				self.pub_image.publish(msg_image)

				self.msg_camera_info.header = self.header
				self.msg_camera_info.header.frame_id = '/camera'
				self.pub_camera_info.publish(self.msg_camera_info)
			else:
				self.node.get_logger().warning("Frame lost!")

			yuv_frame.unref()

	def takeoff_callback(self, request, response):
		self.node.get_logger().warning("Taking off")
		self.drone(TakeOff()).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.TakeOff
		if not self.simulation_environment:
			run_id = self.drone.get_state(olympe.messages.common.RunState.RunIdChanged) # https://developer.parrot.com/docs/olympe/arsdkng_common_runstate.html#olympe.messages.common.RunState.RunIdChanged
			self.node.get_logger().debug('Run Id: %s' % (run_id['runId']))
		return response
	
	def arm_callback(self, request, response):
		if request.data:
			self.node.get_logger().warning("Arming")
			self.drone(UserTakeOff(state = 1)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.UserTakeOff
		else:
			self.node.get_logger().info("Disarming")
			self.drone(
				Emergency() >>  # the fastest way to disarm
				FlyingStateChanged(state="landed")
			).wait()
		return response

	def land_callback(self, request, response):
		self.node.get_logger().info("Landing")
		self.drone(Landing()).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.Landing
		flights_status = self.drone.get_state(olympe.messages.ardrone3.SettingsState.MotorFlightsStatusChanged) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_settings_state.html#olympe.messages.ardrone3.SettingsState.MotorErrorStateChanged
		self.node.get_logger().info('Flight duration = %is' % (flights_status['lastFlightDuration']))
		return response
		
	def emergency_callback(self, request, response):
		self.drone(Emergency()).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.Emergency
		
		if self.drone.get_state(olympe.messages.ardrone3.SoundState.AlertSound)["state"] == olympe.enums.ardrone3.SoundState.AlertSound_State.stopped: # DEPRECATED: https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_sound.html#olympe.messages.ardrone3.SoundState.AlertSound
			self.node.get_logger().fatal("Emergency!!!")
			self.drone(olympe.messages.ardrone3.Sound.StartAlertSound()) # DEPRECATED: https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_sound.html#olympe.messages.ardrone3.Sound.StartAlertSound
		else:
			self.drone(olympe.messages.ardrone3.Sound.StopAlertSound()) # DEPRECATED: https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_sound.html#olympe.messages.ardrone3.Sound.StopAlertSound
		return response
		
	def halt_callback(self, request, response):  # calls all commands to halt
		self.node.get_logger().warning("HALT!!!")
		self.drone(PCMD(flag=1, roll=0, pitch=0, yaw=0, gaz=0, timestampAndSeqNum=0))
		self.drone(NavigateHome(start=0))
		self.drone(StopPilotedPOI())
		self.drone(olympe.messages.common.Mavlink.Stop())
		self.drone(olympe.messages.follow_me.stop())
		self.drone(olympe.messages.rth.abort())
		return response
		
	def navigate_home_callback(self, request, response):
		if request.data:
			self.node.get_logger().info("Navigating Home")
			self.drone(
				PCMD(flag=1, roll=0, pitch=0, yaw=0, gaz=0, timestampAndSeqNum=0)  >>
				FlyingStateChanged(state="hovering")
			).wait()
			self.drone(NavigateHome(start=1)).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.NavigateHome
		else:
			self.node.get_logger().info("Stopping Navigation Home")		
			self.drone(NavigateHome(start=0)).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.NavigateHome
		navigate_home_state = self.drone.get_state(olympe.messages.ardrone3.PilotingState.NavigateHomeStateChanged)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.NavigateHomeStateChanged
		self.node.get_logger().info("Navigate Home State: state = %s, reason = %s" % (navigate_home_state['state'].name, navigate_home_state['reason'].name))
		return response
		
	def rth_callback(self, request, response):
		self.node.get_logger().info("Returning to Home")
		self.drone(
			PCMD(flag=1, roll=0, pitch=0, yaw=0, gaz=0, timestampAndSeqNum=0) >>
			FlyingStateChanged(state="hovering")
		).wait()
		self.drone(return_to_home()).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.return_to_home
		return response
		
	def set_home_callback(self, request, response):
		self.node.get_logger().info("Setting Home")
		self.drone(olympe.messages.rth.set_custom_location(  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.set_custom_location
			latitude=request.latitude,  # latitude of the takeoff location
			longitude=request.longitude,  # longitude of the takeoff location
			altitude=request.altitude  # altitude of the custom location above takeoff (ATO).
		)).wait()
		return response
		
	def start_piloted_POI_callback(self, request, response):
		self.node.get_logger().info("Starting Piloted Point of Interest")		
		self.drone(StartPilotedPOIV2( # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.StartPilotedPOIV2
			latitude=request.latitude, # latitude of the location to look at (deg)
			longitude=request.longitude, # longitude of the location to look at (deg)
			altitude=request.altitude, # altitude above take off point to look at (m)
			mode=('locked_gimbal' if request.locked_gimbal else 'free_gimbal') # gimbal is locked on the POI OR freely controllable
		)).wait()
		piloted_poi_state = self.drone.get_state(olympe.messages.ardrone3.PilotingState.PilotedPOIV2) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.PilotedPOIV2
		self.node.get_logger().info("Piloted POI State: latitude = %f, longitude = %f, altitude = %f, mode = %s, status = %s" %
									(piloted_poi_state['latitude'], piloted_poi_state['longitude'], piloted_poi_state['altitude'],
									 piloted_poi_state['mode'].name, piloted_poi_state['status'].name))
		return response
	
	def stop_piloted_POI_callback(self, request, response):
		self.node.get_logger().info("Stopping Piloted Point of Interest")		
		self.drone(StopPilotedPOI()).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.StopPilotedPOI
		return response
			
	def flightplan_upload_callback(self, request, response): # https://forum.developer.parrot.com/t/olympe-mavlink-working-example/14041/2
		self.node.get_logger().info("FlightPlan uploading from " + request.file)
		self.node.get_logger().info("REST API: PUT http://" + self.ip + ":180/api/v" + self.rest_api_version + "/upload/flightplan,  data=" + request.file)
		response = requests.put(url="http://" + self.ip + ":180/api/v" + self.rest_api_version + "/upload/flightplan", data=open(request.file, "rb"))
		response.raise_for_status()
		self.uid = response.json()
		self.node.get_logger().info("FlightPlan uploaded with UID " + self.uid)
		return response
		
	def flightplan_start_callback(self, request, response): # https://forum.developer.parrot.com/t/olympe-mavlink-working-example/14041/2
		uid = (request.uid if request.uid != "" else self.uid)
		self.node.get_logger().warning("FlightPlan starting with UID " + uid)
		self.node.get_logger().debug("REST API: GET http://" + self.ip + ":180/api/v" + self.rest_api_version + "/upload/flightplan/" + uid)
		response = requests.get("http://" + self.ip + ":180/api/v" + self.rest_api_version + "/upload/flightplan/" + uid)
		response.raise_for_status()
		if response.status_code == requests.codes.ok:
			self.drone(
				olympe.messages.common.Mavlink.Start( # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.Mavlink.Start
				filepath=uid, # TODO: check why sometimes doesn't take the filepath
				type='flightPlan' # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.enums.common.Mavlink.Start_Type
			)).wait()
				
			if self.drone.get_state(olympe.messages.common.FlightPlanState.AvailabilityStateChanged)['AvailabilityState'] == 0: # https://developer.parrot.com/docs/olympe/arsdkng_common_flightplan.html#olympe.messages.common.FlightPlanState.AvailabilityStateChanged
				components = self.drone.get_state(olympe.messages.common.FlightPlanState.ComponentStateListChanged) # https://developer.parrot.com/docs/olympe/arsdkng_common_flightplan.html#olympe.messages.common.FlightPlanState.ComponentStateListChanged
				for component in components:
					if components[component]['State'] == 0:
						self.node.get_logger().warning("FlightPlan: %s is NOT OK" % str(components[component]['component'].name))
					else:
						self.node.get_logger().info("FlightPlan: %s is OK" % str(components[component]['component'].name))
		else:
			self.node.get_logger().warning("UID %s does not exist onboard" % uid)
		return response
		
	def flightplan_pause_callback(self, request, response):
		self.node.get_logger().info("FlightPlan pausing")
		self.drone(olympe.messages.common.Mavlink.Pause()).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.Mavlink.Pause
		return response
		
	def flightplan_stop_callback(self, request, response):
		self.node.get_logger().info("FlightPlan stopping")
		self.drone(olympe.messages.common.Mavlink.Stop()).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.Mavlink.Stop
		return response
		
	def followme_start_callback(self, request, response):
		self.node.get_logger().warning("FollowMe starting with " + olympe.enums.follow_me.mode(request.mode).name)
		self.node.get_logger().info("Target is controller: %r" % self.drone.get_state(olympe.messages.follow_me.target_is_controller)['state'] == 1) # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.target_is_controller
		self.drone(olympe.messages.follow_me.target_framing_position( # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.target_framing_position
			horizontal=request.horizontal,
			vertical=request.vertical
			)).wait()
		self.drone(olympe.messages.follow_me.target_image_detection( # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.target_image_detection
			target_azimuth=request.target_azimuth,
			target_elevation=request.target_elevation,
			change_of_scale=request.change_of_scale,
			confidence_index=request.confidence_index,
			is_new_selection=request.is_new_selection
			)).wait()
		self.drone(olympe.messages.follow_me.start( # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.start
			mode=olympe.enums.follow_me.mode(request.mode) # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.enums.follow_me.mode
			)).wait()
		return response
			
	def followme_stop_callback(self, request, response):
		self.node.get_logger().info("FollowMe stopping")
		self.drone(olympe.messages.follow_me.stop()).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.stop
		return response

	def calibrate_magnetometer_callback(self, request, response):
		if self.drone.get_state(olympe.messages.common.CalibrationState.MagnetoCalibrationRequiredState)['required'] == 1: # https://developer.parrot.com/docs/olympe/arsdkng_common_calibration.html#olympe.messages.common.CalibrationState.MagnetoCalibrationRequiredState
			self.node.get_logger().warning("Magnetometer calibration started")
			self.drone(olympe.messages.common.Calibration.MagnetoCalibration(calibrate=1, _timeout=100)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_common_calibration.html#olympe.messages.common.Calibration.MagnetoCalibration
			calibrating_axis = -1
			while rclpy.ok():
				calibration_axis = self.drone.get_state(olympe.messages.common.CalibrationState.MagnetoCalibrationAxisToCalibrateChanged)['axis']
				if calibration_axis is CalibrationState.MagnetoCalibrationAxisToCalibrateChanged_Axis.none:
					if self.drone.get_state(olympe.messages.common.CalibrationState.MagnetoCalibrationRequiredState)['required'] == 1:
						self.node.get_logger().warning("Magnetometer calibration restarted")
						self.drone(olympe.messages.common.Calibration.MagnetoCalibration(calibrate=1, _timeout=100)).wait()
						calibrating_axis = -1
					else:
						break
				if calibrating_axis != calibration_axis.value:
					self.node.get_logger().warning("Rotate the drone around " + calibration_axis.name)
					self.drone(olympe.messages.ardrone3.Sound.StartAlertSound()).wait()  # DEPRECATED: https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_sound.html#olympe.messages.ardrone3.Sound.StartAlertSound
					time.sleep(1.0)
					self.drone(olympe.messages.ardrone3.Sound.StopAlertSound()).wait()  # DEPRECATED: https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_sound.html#olympe.messages.ardrone3.Sound.StopAlertSound
					calibrating_axis = calibration_axis.value				
				time.sleep(1.0)
			state = self.drone.get_state(olympe.messages.common.CalibrationState.MagnetoCalibrationStateChanged) # https://developer.parrot.com/docs/olympe/arsdkng_common_calibration.html#olympe.messages.common.CalibrationState.MagnetoCalibrationStateChanged
			if state['calibrationFailed'] == 1:
				self.node.get_logger().warning("Calibration status: x-axis - %s, y-axis - %s, z-axis - %s" %
											   (('completed' if state['xAxisCalibration'] else 'failed'),
												('completed' if state['yAxisCalibration'] else 'failed'),
												('completed' if state['zAxisCalibration'] else 'failed')))
				self.node.get_logger().fatal("Calibration failed")
			else:
				self.node.get_logger().info("Calibration completed")
		else:
			self.node.get_logger().info("Magnetometer calibration is not required")
		return response
		
	def calibrate_gimbal_callback(self, request, response):
		if self.drone.get_state(olympe.messages.gimbal.calibration_state)[0]['state'] == olympe.enums.gimbal.calibration_state.required: # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.calibration_state
			self.node.get_logger().info("Calibrating gimbal")
			self.drone(olympe.messages.gimbal.calibrate(gimbal_id=0)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.calibrate
		else:
			self.node.get_logger().info("Gimbal calibration is not required")
		return response
		
	def reset_zoom_callback(self, request, response):
		self.node.get_logger().debug("Reseting zoom")
		self.drone(camera.reset_zoom(cam_id=0)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.reset_zoom
		return response
		
	def reset_gimbal_callback(self, request, response):
		self.node.get_logger().debug("Reseting gimbal")
		self.drone(olympe.messages.gimbal.reset_orientation(gimbal_id=0)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.reset_orientation
		return response
			
	def take_photo_callback(self, request, response):
		self.node.get_logger().info("Taking photo")
		self.drone(camera.set_camera_mode(cam_id=0, value="photo")).wait()
		self.drone(camera.set_photo_mode(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_photo_mode
			cam_id=0,
			mode=olympe.enums.camera.photo_mode(request.mode),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.photo_mode
			format=olympe.enums.camera.photo_format(request.photo_format),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.photo_format
			file_format=olympe.enums.camera.photo_file_format(request.file_format),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.photo_file_format
			burst=olympe.enums.camera.burst_value(request.burst),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.photo_file_format
			bracketing=olympe.enums.camera.bracketing_preset(request.bracketing),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.bracketing_preset
			capture_interval=request.capture_interval))   # [s\time_lapse; m\gps_lapse]
		photo_saved = self.drone(camera.photo_progress(result="photo_saved", _policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.photo_progress
		self.drone(camera.take_photo(cam_id=0)) # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.take_photo
		if photo_saved.wait(_timeout=5).success():
			response.media_id = photo_saved.received_events().last().args["media_id"]
		self.drone(camera.set_camera_mode(cam_id=0, value="recording" if self.camera_mode == 0 else "photo"))
		return response
		
	def stop_photo_callback(self, request, response):
		self.node.get_logger().info("Stopping photo")
		photo_stopped = self.drone(camera.recording_progress(result="stopped", _policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.recording_progress
		self.drone(camera.stop_photo(cam_id=0)) # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.stop_photo
		if photo_stopped.wait(_timeout=5).success():
			response.media_id = photo_stopped.received_events().last().args["media_id"]
		self.drone(camera.set_camera_mode(cam_id=0, value="recording" if self.camera_mode == 0 else "photo"))
		return response
		
	def start_recording_callback(self, request, response):
		self.node.get_logger().info("Starting recording")
		self.drone(camera.set_camera_mode(cam_id=0, value="recording")).wait() # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_camera_mode
		self.drone(camera.set_recording_mode(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_recording_mode
			cam_id=0,
			mode=olympe.enums.camera.recording_mode(request.mode),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.recording_mode
			resolution=olympe.enums.camera.resolution(request.resolution),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.resolution
			framerate=olympe.enums.camera.framerate(request.framerate),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.framerate
			hyperlapse=olympe.enums.camera.hyperlapse_value(request.hyperlapse)))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.hyperlapse_value
		self.drone(camera.start_recording(cam_id=0)) # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.start_recording
		return response
		
	def stop_recording_callback(self, request, response):
		self.node.get_logger().info("Stopping recording")
		recording_stopped = self.drone(camera.recording_progress(result="stopped", _policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.recording_progress
		self.drone(camera.stop_recording(cam_id=0)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.stop_recording
		if recording_stopped.wait(_timeout=10).success():
			response.media_id = recording_stopped.received_events().last().args["media_id"]
		self.drone(camera.set_camera_mode(cam_id=0, value="recording" if self.camera_mode == 0 else "photo"))
		return response
		
	def download_media_callback(self, request, response):
		if self.drone.media(indexing_state(state="indexed")).wait(_timeout=10).success():  # FIXME: fails on some drones
			media_id = olympe.Media.list_media(self.drone.media)
			num_media = len(media_id)

			if num_media > 0:
				if not os.path.exists(self.download_folder):  # check if folder exists
					os.makedirs(self.download_folder)
					self.node.get_logger().info("Folder '%s' created" % self.download_folder)
				self.drone.media.download_dir = self.download_folder  # download the photos associated with this media id

				self.node.get_logger().info("Downloading %i media" % num_media)

				media_count = 1
				for media in media_id:
					media_info = olympe.Media.media_info(self.drone.media, media_id = media)
					self.node.get_logger().info("Media %i/%i: downloading %.1fMB" % (media_count, num_media, media_info.size/(2**20)))
					media_download = self.drone(download_media(media))
					resources = media_download.as_completed(timeout=100)
					self.node.get_logger().info("Media %i/%i: downloaded %.1fMB" % (media_count, num_media, media_info.size/(2**20)))

					for resource in resources:
						if not resource.success():
							self.node.get_logger().error("Failed to download %s" % str(resource.resource_id))
							continue

					media_count += 1

				if request.data:  # cut media
					self.drone(delete_all_media())
			else:
				self.node.get_logger().info("No media found")
		else:
			self.node.get_logger().warning("Storage is not indexed :(")
		return response
	
	def reboot_callback(self, request, response):
		self.node.get_logger().warning("Rebooting...")
		assert self.drone(olympe.messages.common.Common.Reboot()).wait().success()  # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.Common.Reboot
		return response
		
	def format_callback(self, request, response):
		info = self.drone.get_state(olympe.messages.user_storage.info)  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.info
		if info['name'] != "":
			self.node.get_logger().info("Formatting media %s (%.1fGB)" % (info['name'], info['capacity']/(2**30)))
			self.drone(olympe.messages.user_storage.format_with_type(  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.format_with_type
				label="",
				type=olympe.enums.user_storage.formatting_type(0))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.enums.user_storage.formatting_type
				>>
				olympe.messages.user_storage.start_monitoring(period=1))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.start_monitoring
		else:
			self.node.get_logger().warning("There is no media to format")
		return response

	def discover_drones_callback(self, request, response):
		self.node.get_logger().info("Discovering drones...")
		self.drone(olympe.messages.drone_manager.discover_drones()).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.discover_drones
		drone_list = self.drone.get_state(olympe.messages.drone_manager.drone_list_item)  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.drone_list_item
		for drone in drone_list:
			self.node.get_logger().info(
				"Drone: serial=%s, model=%i, name=%s, connection order=%s, connecting=%s, visible=%s, security=%s, saved key=%s, rssi=%i" %
				(drone_list[drone]['serial'], drone_list[drone]['model'], drone_list[drone]['name'],
				 (drone_list[drone]['connection_order'] if drone_list[drone]['connection_order'] > 0 else 'never connected'),
				 ('yes' if drone_list[drone]['active'] == 1 else 'no'),
				 ('yes' if drone_list[drone]['visible'] == 1 else 'no'), drone_list[drone]['security'].name,
				 ('yes' if drone_list[drone]['has_saved_key'] == 1 else 'no'), drone_list[drone]['rssi']))
		return response

	def forget_drone_callback(self, request, response):
		self.node.get_logger().info("Forgetting drone %s" % self.serial)
		self.drone(olympe.messages.drone_manager.forget(serial=self.serial)).wait().success()  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.forget
		return response

	def offboard_callback(self, request, response):
		if request.data:
			self.switch_offboard()
		else:
			self.switch_manual()
		return response

	def rpyt_callback(self, msg):
		self.drone(PCMD(  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.PCMD
			flag=1,
			roll=int(bound_percentage(msg.roll/self.max_tilt*100)),  # roll [-100, 100] (% of max tilt)
			pitch=int(bound_percentage(msg.pitch/self.max_tilt*100)),  # pitch [-100, 100] (% of max tilt)
			yaw=int(bound_percentage(-msg.yaw/self.max_yaw_rate*100)),  # yaw rate [-100, 100] (% of max yaw rate)
			gaz=int(bound_percentage(msg.gaz/self.max_vertical_speed*100)),  # vertical speed [-100, 100] (% of max vertical speed)
			timestampAndSeqNum=0))

	def moveBy_callback(self, msg):		
		self.drone(move.extended_move_by( # https://developer.parrot.com/docs/olympe/arsdkng_move.html?#olympe.messages.move.extended_move_by
			d_x=msg.dx, # displacement along the front axis (m)
			d_y=msg.dy, # displacement along the right axis (m)
			d_z=msg.dz, # displacement along the down axis (m)
			d_psi=msg.dyaw, # rotation of heading (rad)
			max_horizontal_speed=self.max_horizontal_speed,
			max_vertical_speed=self.max_vertical_speed,
			max_yaw_rotation_speed=self.max_yaw_rate
		)).wait()
	
	def moveTo_callback(self, msg):		
		self.drone(move.extended_move_to( # https://developer.parrot.com/docs/olympe/arsdkng_move.html?#olympe.messages.move.extended_move_to
			latitude=msg.latitude, # latitude (degrees)
			longitude=msg.longitude, # longitude (degrees)
			altitude=msg.altitude, # altitude (m)
			orientation_mode=olympe.enums.move.orientation_mode(msg.orientation_mode), # https://developer.parrot.com/docs/olympe/arsdkng_move.html#olympe.enums.move.orientation_mode
			heading=msg.heading, # heading relative to the North (degrees)
			max_horizontal_speed=self.max_horizontal_speed,
			max_vertical_speed=self.max_vertical_speed,
			max_yaw_rotation_speed=self.max_yaw_rate
		)).wait()

	def zoom_callback(self, msg):
		self.drone(camera.set_zoom_target(  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_zoom_target
			cam_id=0,
			control_mode=olympe.enums.camera.zoom_control_mode(msg.mode),  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.enums.camera.zoom_control_mode
			target=msg.zoom)) # (in level mode [1,max_zoom])
		
	def gimbal_callback(self, msg):
		self.drone(gimbal.set_target(  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.set_target
			gimbal_id=0,
			control_mode=olympe.enums.gimbal.control_mode(msg.mode),  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.enums.gimbal.control_mode
			yaw_frame_of_reference=olympe.enums.gimbal.frame_of_reference(msg.frame),  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.enums.gimbal.frame_of_reference
			yaw=-msg.yaw, 
			pitch_frame_of_reference=olympe.enums.gimbal.frame_of_reference(msg.frame),  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.enums.gimbal.frame_of_reference
			pitch=-msg.pitch,
			roll_frame_of_reference=olympe.enums.gimbal.frame_of_reference(msg.frame),  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.enums.gimbal.frame_of_reference
			roll=msg.roll))

	def switch_manual(self):
		if self.offboard:
			if self.skycontroller_enabled:
				# button: 0 = return home, 1 = takeoff/land, 2 = back left, 3 = back right
				self.drone(mapper.grab(buttons=(0<<0|0<<1|0<<2|1<<3), axes=0)).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab
				self.drone(setPilotingSource(source="SkyController")).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_skyctrl_copiloting.html#olympe.messages.skyctrl.CoPiloting.setPilotingSource
				self.node.set_parameters([Parameter('drone/offboard', rclpy.Parameter.Type.BOOL, False)])
				self.node.get_logger().warning("Control: Manual")
				self.offboard = False
			else:
				self.node.get_logger().error("Cannot swith to manual control without Skycontroller!")
			
	def switch_offboard(self):
		if not self.offboard: 
			if self.skycontroller_enabled:
				# button: 0 = return home, 1 = takeoff/land, 2 = back left, 3 = back right
				# axis:   0 = yaw, 1 = throttle, 2 = roll, 3 = pitch, 4 = camera, 5 = zoom
				self.drone(mapper.grab(buttons=(1<<0|0<<1|1<<2|1<<3), axes=(1<<0|1<<1|1<<2|1<<3|1<<4|1<<5)))  # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab
				self.drone(setPilotingSource(source="Controller")).wait()  # https://developer.parrot.com/docs/olympe/arsdkng_skyctrl_copiloting.html#olympe.messages.skyctrl.CoPiloting.setPilotingSource
			self.node.set_parameters([Parameter('drone/offboard', rclpy.Parameter.Type.BOOL, True)])
			self.node.get_logger().warning("Control: Offboard")
			self.offboard = True

def main(args=None):
	rclpy.init(args=sys.argv)

	anafi = Anafi()

	rclpy.spin(anafi.node)

	anafi.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

