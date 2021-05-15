#!/usr/bin/env python3

import rospy
import csv
import cv2
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import traceback
import time
import logging
import roslib
import sys
import math
import olympe

from std_msgs.msg import UInt8, UInt16, UInt32, Int8, Float32, String, Header, Time, Empty, Bool
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, TwistStamped, Vector3Stamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from olympe.messages.drone_manager import connection_state
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, Emergency, PCMD, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged, SpeedChanged, AttitudeChanged, AltitudeChanged, GpsLocationChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt, MaxDistance, MaxAltitude, NoFlyOverMaxDistance, BankedTurn
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged, MaxDistanceChanged, MaxAltitudeChanged, NoFlyOverMaxDistanceChanged, BankedTurnChanged
from olympe.messages.ardrone3.SpeedSettings import MaxVerticalSpeed, MaxRotationSpeed, MaxPitchRollRotationSpeed
from olympe.messages.ardrone3.SpeedSettingsState import MaxVerticalSpeedChanged, MaxRotationSpeedChanged, MaxPitchRollRotationSpeedChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSState import NumberOfSatelliteChanged
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.skyctrl.CoPilotingState import pilotingSource
from olympe.messages.skyctrl.Common import AllStates
from olympe.messages.skyctrl.CommonState import AllStatesChanged
from olympe.messages import gimbal, camera, mapper
from olympe.enums.mapper import button_event
from olympe.enums.skyctrl.CoPilotingState import PilotingSource_Source

from scipy.spatial.transform import Rotation as R

from dynamic_reconfigure.server import Server
from olympe_bridge.cfg import setAnafiConfig
from olympe_bridge.msg import PilotingCommand, CameraCommand, MoveByCommand, MoveToCommand, SkyControllerCommand

olympe.log.update_config({"loggers": {"olympe": {"level": "ERROR"}}})

DRONE_IP = "192.168.42.1"
SKYCTRL_IP = "192.168.53.1"

class Anafi(threading.Thread):
	def __init__(self):	
		if rospy.get_param("/indoor"):			
			rospy.loginfo("We are indoor")
		else:
			rospy.loginfo("We are outdoor")
					
		self.pub_image = rospy.Publisher("/anafi/image", Image, queue_size=1)
		self.pub_time = rospy.Publisher("/anafi/time", Time, queue_size=1)
		self.pub_attitude = rospy.Publisher("/anafi/attitude", QuaternionStamped, queue_size=1)
		self.pub_location = rospy.Publisher("/anafi/location", PointStamped, queue_size=1)
		self.pub_height = rospy.Publisher("/anafi/height", Float32, queue_size=1)
		self.pub_speed = rospy.Publisher("/anafi/speed", Vector3Stamped, queue_size=1)
		self.pub_air_speed = rospy.Publisher("/anafi/air_speed", Float32, queue_size=1)
		self.pub_link_goodput = rospy.Publisher("/anafi/link_goodput", UInt16, queue_size=1)
		self.pub_link_quality = rospy.Publisher("/anafi/link_quality", UInt8, queue_size=1)
		self.pub_wifi_rssi = rospy.Publisher("/anafi/wifi_rssi", Int8, queue_size=1)
		self.pub_battery = rospy.Publisher("/anafi/battery", UInt8, queue_size=1)
		self.pub_state = rospy.Publisher("/anafi/state", String, queue_size=1)
		self.pub_mode = rospy.Publisher("/anafi/mode", String, queue_size=1)
		self.pub_pose = rospy.Publisher("/anafi/pose", PoseStamped, queue_size=1)
		self.pub_odometry = rospy.Publisher("/anafi/odometry", Odometry, queue_size=1)
		self.pub_rpy = rospy.Publisher("/anafi/rpy", Vector3Stamped, queue_size=1)
		self.pub_skycontroller = rospy.Publisher("/skycontroller/command", SkyControllerCommand, queue_size=1)

		rospy.Subscriber("/anafi/takeoff", Empty, self.takeoff_callback)
		rospy.Subscriber("/anafi/land", Empty, self.land_callback)
		rospy.Subscriber("/anafi/emergency", Empty, self.emergency_callback)
		rospy.Subscriber("/anafi/offboard", Bool, self.offboard_callback)
		rospy.Subscriber("/anafi/cmd_rpyt", PilotingCommand, self.rpyt_callback)
		rospy.Subscriber("/anafi/cmd_moveto", MoveToCommand, self.moveTo_callback)
		rospy.Subscriber("/anafi/cmd_moveby", MoveByCommand, self.moveBy_callback)
		rospy.Subscriber("/anafi/cmd_camera", CameraCommand, self.camera_callback)
		
		# Connect to the SkyController	
		if rospy.get_param("/skycontroller"):
			rospy.loginfo("Connecting through SkyController");
			self.drone = olympe.Drone(SKYCTRL_IP)
		# Connect to the Anafi
		else:
			rospy.loginfo("Connecting directly to Anafi");
			self.drone = olympe.Drone(DRONE_IP)
		
		# Create listener for RC events
		self.every_event_listener = EveryEventListener(self)
		
		rospy.on_shutdown(self.stop)
		
		self.srv = Server(setAnafiConfig, self.reconfigure_callback)
						
		self.connect()
				
		# To convert OpenCV images to ROS images
		self.bridge = CvBridge()
		
	def connect(self):
		self.every_event_listener.subscribe()
		
		rate = rospy.Rate(1) # 1hz
		while True:
			self.pub_state.publish("CONNECTING")
			connection = self.drone.connect()
			if getattr(connection, 'OK') == True:
				break
			if rospy.is_shutdown():
				exit()
			rate.sleep()
		
		# Connect to the SkyController	
		if rospy.get_param("/skycontroller"):
			self.pub_state.publish("CONNECTED_SKYCONTROLLER")
			rospy.loginfo("Connection to SkyController: " + getattr(connection, 'message'))
			self.switch_manual()
					
			# Connect to the drone
			while True:
				if self.drone(connection_state(state="connected", _policy="check")):
					break				
				if rospy.is_shutdown():
					exit()
				else:
					self.pub_state.publish("SERCHING_DRONE")
					rospy.loginfo_once("Connection to Anafi: " + str(self.drone.get_state(connection_state)["state"]))
				rate.sleep()
			self.pub_state.publish("CONNECTED_DRONE")			
			rospy.loginfo("Connection to Anafi: " + str(self.drone.get_state(connection_state)["state"]))
		# Connect to the Anafi
		else:
			self.pub_state.publish("CONNECTED_DRONE")
			rospy.loginfo("Connection to Anafi: " + getattr(connection, 'message'))
			self.switch_offboard()
			
		self.frame_queue = queue.Queue()
		self.flush_queue_lock = threading.Lock()

		# Setup the callback functions to do some live video processing
		self.drone.set_streaming_callbacks(
			raw_cb=self.yuv_frame_cb,
			flush_raw_cb=self.flush_cb
		)
		self.drone.start_video_streaming()		
		
	def disconnect(self):
		self.pub_state.publish("DISCONNECTING")
		self.every_event_listener.unsubscribe()
		#self.drone.stop_video_streaming()
		self.drone.disconnect()
		self.pub_state.publish("DISCONNECTED")
		
	def stop(self):
		rospy.loginfo("AnafiBridge is stopping...")
		self.disconnect()
						
	def reconfigure_callback(self, config, level):
		if level == -1 or level == 1:
			self.drone(MaxTilt(config['max_tilt'])).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html?#olympe.messages.ardrone3.PilotingSettings.MaxTilt
			self.drone(MaxVerticalSpeed(config['max_vertical_speed'])).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettings.MaxVerticalSpeed
			self.drone(MaxRotationSpeed(config['max_yaw_rotation_speed'])).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettings.MaxRotationSpeed
			self.drone(MaxPitchRollRotationSpeed(config['max_pitch_roll_rotation_speed'])).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.SpeedSettings.MaxPitchRollRotationSpeed
			self.drone(MaxDistance(config['max_distance'])).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettings.MaxDistance
			self.drone(MaxAltitude(config['max_altitude'])).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettings.MaxAltitude
			self.drone(NoFlyOverMaxDistance(1)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettings.NoFlyOverMaxDistance
			self.drone(BankedTurn(int(config['banked_turn']))).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingSettings.BankedTurn
			self.max_tilt = config['max_tilt']
			self.max_vertical_speed = config['max_vertical_speed']
			self.max_rotation_speed = config['max_yaw_rotation_speed']
		if level == -1 or level == 2:
			self.gimbal_frame = 'absolute' if config['gimbal_compensation'] else 'relative'
			self.drone(gimbal.set_max_speed(
				gimbal_id=0,
				yaw=0, 
				pitch=config['max_gimbal_speed'], # [1 180] (deg/s)
				roll=config['max_gimbal_speed'] # [1 180] (deg/s)
				)).wait()
		return config
		
	# This function will be called by Olympe for each decoded YUV frame.
	def yuv_frame_cb(self, yuv_frame):      
		yuv_frame.ref()
		self.frame_queue.put_nowait(yuv_frame)

	def flush_cb(self):
		with self.flush_queue_lock:
			while not self.frame_queue.empty():
				self.frame_queue.get_nowait().unref()
		return True

	def yuv_callback(self, yuv_frame):
		# Use OpenCV to convert the yuv frame to RGB
		info = yuv_frame.info() # the VideoFrame.info() dictionary contains some useful information such as the video resolution
		rospy.logdebug_throttle(10, "yuv_frame.info = " + str(info))
		cv2_cvt_color_flag = {
			olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
			olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
		}[info["yuv"]["format"]] # convert pdraw YUV flag to OpenCV YUV flag
		cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
		
		# Publish image
		msg_image = self.bridge.cv2_to_imgmsg(cv2frame, "bgr8")
		self.pub_image.publish(msg_image)

		# yuv_frame.vmeta() returns a dictionary that contains additional metadata from the drone (GPS coordinates, battery percentage, ...)
		metadata = yuv_frame.vmeta()
		rospy.logdebug_throttle(10, "yuv_frame.vmeta = " + str(metadata))
				
		if metadata[1] != None:
			header = Header()
			header.stamp = rospy.Time.now()
			header.frame_id = '/body'
		
			frame_timestamp = metadata[1]['frame_timestamp'] # timestamp [millisec]
			msg_time = Time()
			msg_time.data = frame_timestamp # secs = int(frame_timestamp//1e6), nsecs = int(frame_timestamp%1e6*1e3)
			self.pub_time.publish(msg_time)

			drone_quat = metadata[1]['drone_quat'] # attitude
			msg_attitude = QuaternionStamped()
			msg_attitude.header = header
			msg_attitude.quaternion = Quaternion(drone_quat['x'], -drone_quat['y'], -drone_quat['z'], drone_quat['w'])
			self.pub_attitude.publish(msg_attitude)
					
			location = metadata[1]['location'] # GPS location [500.0=not available] (decimal deg)
			msg_location = PointStamped()
			if location != {}:			
				msg_location.header = header
				msg_location.header.frame_id = '/world'
				msg_location.point.x = location['latitude']
				msg_location.point.y = location['longitude']
				msg_location.point.z = location['altitude']
				self.pub_location.publish(msg_location)
				
			ground_distance = metadata[1]['ground_distance'] # barometer (m)
			self.pub_height.publish(ground_distance)

			speed = metadata[1]['speed'] # opticalflow speed (m/s)
			msg_speed = Vector3Stamped()
			msg_speed.header = header
			msg_speed.header.frame_id = '/world'
			msg_speed.vector.x = speed['north']
			msg_speed.vector.y = -speed['east']
			msg_speed.vector.z = -speed['down']
			self.pub_speed.publish(msg_speed)

			air_speed = metadata[1]['air_speed'] # air speed [-1=no data, > 0] (m/s)
			self.pub_air_speed.publish(air_speed)

			link_goodput = metadata[1]['link_goodput'] # throughput of the connection (b/s)
			self.pub_link_goodput.publish(link_goodput)

			link_quality = metadata[1]['link_quality'] # [0=bad, 5=good]
			self.pub_link_quality.publish(link_quality)

			wifi_rssi = metadata[1]['wifi_rssi'] # signal strength [-100=bad, 0=good] (dBm)
			self.pub_wifi_rssi.publish(wifi_rssi)

			battery_percentage = metadata[1]['battery_percentage'] # [0=empty, 100=full]
			self.pub_battery.publish(battery_percentage)

			state = metadata[1]['state'] # ['LANDED', 'MOTOR_RAMPING', 'TAKINGOFF', 'HOWERING', 'FLYING', 'LANDING', 'EMERGENCY']
			self.pub_state.publish(state)

			mode = metadata[1]['mode'] # ['MANUAL', 'RETURN_HOME', 'FLIGHT_PLAN', 'TRACKING', 'FOLLOW_ME', 'MOVE_TO']
			self.pub_mode.publish(mode)
			
			msg_pose = PoseStamped()
			msg_pose.header = header
			msg_pose.pose.position = msg_location.point
			msg_pose.pose.position.z = ground_distance
			msg_pose.pose.orientation = msg_attitude.quaternion
			self.pub_pose.publish(msg_pose)
			
			Rot = R.from_quat([drone_quat['x'], -drone_quat['y'], -drone_quat['z'], drone_quat['w']])
			drone_rpy = Rot.as_euler('xyz')
			
			msg_odometry = Odometry()
			msg_odometry.header = header
			msg_odometry.child_frame_id = '/body'
			msg_odometry.pose.pose = msg_pose.pose
			msg_odometry.twist.twist.linear.x =  math.cos(drone_rpy[2])*msg_speed.vector.x + math.sin(drone_rpy[2])*msg_speed.vector.y
			msg_odometry.twist.twist.linear.y = -math.sin(drone_rpy[2])*msg_speed.vector.x + math.cos(drone_rpy[2])*msg_speed.vector.y
			msg_odometry.twist.twist.linear.z = msg_speed.vector.z
			self.pub_odometry.publish(msg_odometry)
			
			# log battery percentage
			if battery_percentage >= 30:
				if battery_percentage%10 == 0:
					rospy.loginfo_throttle(100, "Battery level: " + str(battery_percentage) + "%")
			else:
				if battery_percentage >= 20:
					rospy.logwarn_throttle(10, "Low battery: " + str(battery_percentage) + "%")
				else:
					if battery_percentage >= 10:
						rospy.logerr_throttle(1, "Critical battery: " + str(battery_percentage) + "%")
					else:
						rospy.logfatal_throttle(0.1, "Empty battery: " + str(battery_percentage) + "%")		
					
			# log signal strength
			if wifi_rssi <= -60:
				if wifi_rssi >= -70:
					rospy.loginfo_throttle(100, "Signal strength: " + str(wifi_rssi) + "dBm")
				else:
					if wifi_rssi >= -80:
						rospy.logwarn_throttle(10, "Weak signal: " + str(wifi_rssi) + "dBm")
					else:
						if wifi_rssi >= -90:
							rospy.logerr_throttle(1, "Unreliable signal:" + str(wifi_rssi) + "dBm")
						else:
							rospy.logfatal_throttle(0.1, "Unusable signal: " + str(wifi_rssi) + "dBm")
		else:
			rospy.logwarn("Packet lost!")
		
	def takeoff_callback(self, msg):		
		self.drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.TakeOff
		rospy.logwarn("Takeoff")

	def land_callback(self, msg):		
		self.drone(Landing()).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.Landing
		rospy.loginfo("Land")

	def emergency_callback(self, msg):		
		self.drone(Emergency()).wait() # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.Emergency
		rospy.logfatal("Emergency!!!")
		
	def offboard_callback(self, msg):
		if msg.data == False:	
			self.switch_manual()
		else:
			self.switch_offboard()
						
	def rpyt_callback(self, msg):
		self.drone(PCMD( # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.PCMD
			flag=1,
			roll=int(self.bound_percentage(msg.roll/self.max_tilt*100)), # roll [-100, 100] (% of max tilt)
			pitch=int(self.bound_percentage(msg.pitch/self.max_tilt*100)), # pitch [-100, 100] (% of max tilt)
			yaw=int(self.bound_percentage(-msg.yaw/self.max_rotation_speed*100)), # yaw rate [-100, 100] (% of max yaw rate)
			gaz=int(self.bound_percentage(msg.gaz/self.max_vertical_speed*100)), # vertical speed [-100, 100] (% of max vertical speed)
			timestampAndSeqNum=0)) # for debug only

	# TODO: NOT USED YET	
	def moveBy_callback(self, msg):		
		assert self.drone(moveBy( # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.moveBy
			dX=msg.dx, # displacement along the front axis (m)
			dY=msg.dy, # displacement along the right axis (m)
			dZ=msg.dz, # displacement along the down axis (m)
			dPsi=msg.dyaw # rotation of heading (rad)
			) >> FlyingStateChanged(state="hovering", _timeout=1)
		).wait().success()
	
	# TODO: NOT USED YET	
	def moveTo_callback(self, msg):		
		assert self.drone(moveTo( # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.moveTo
			latitude=msg.latitude, # latitude (degrees)
			longitude=msg.longitude, # longitude (degrees)
			altitude=msg.altitude, # altitude (m)
			heading=msg.heading, # heading relative to the North (degrees)
			orientation_mode=HEADING_START # orientation mode {TO_TARGET, HEADING_START, HEADING_DURING}
			) >> FlyingStateChanged(state="hovering", _timeout=5)
		).wait().success()

	def camera_callback(self, msg):
		if msg.action & 0b001: # take picture
			self.drone(camera.take_photo(cam_id=0)) # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.take_photo
		if msg.action & 0b010: # start recording
			self.drone(camera.start_recording(cam_id=0)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.start_recording
		if msg.action & 0b100: # stop recording
			self.drone(camera.stop_recording(cam_id=0)).wait() # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.stop_recording
	
		self.drone(gimbal.set_target( # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.set_target
			gimbal_id=0,
			control_mode='position', # {'position', 'velocity'}
			yaw_frame_of_reference='none',
			yaw=0.0,
			pitch_frame_of_reference=self.gimbal_frame, # {'absolute', 'relative', 'none'}
			pitch=msg.pitch,
			roll_frame_of_reference=self.gimbal_frame, # {'absolute', 'relative', 'none'}
			roll=msg.roll))
			
		self.drone(camera.set_zoom_target( # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.set_zoom_target
			cam_id=0,
			control_mode='level', # {'level', 'velocity'}
			target=msg.zoom)) # [1, 3]

	def switch_manual(self):
		msg_rpyt = SkyControllerCommand()
		msg_rpyt.header.stamp = rospy.Time.now()
		msg_rpyt.header.frame_id = '/body'
		self.pub_skycontroller.publish(msg_rpyt)
		
		# button: 	0 = RTL, 1 = takeoff/land, 2 = back left, 3 = back right
		self.drone(mapper.grab(buttons=(0<<0|0<<1|0<<2|1<<3), axes=0)).wait() # bitfields
		self.drone(setPilotingSource(source="SkyController")).wait()
		rospy.loginfo("Control: Manual")
			
	def switch_offboard(self):
		# button: 	0 = RTL, 1 = takeoff/land, 2 = back left, 3 = back right
		# axis: 	0 = yaw, 1 = trottle, 2 = roll, 3 = pithch, 4 = camera, 5 = zoom
		if self.drone.get_state(pilotingSource)["source"] == PilotingSource_Source.SkyController:
			self.drone(mapper.grab(buttons=(1<<0|0<<1|1<<2|1<<3), axes=(1<<0|1<<1|1<<2|1<<3|0<<4|0<<5))) # bitfields
			self.drone(setPilotingSource(source="Controller")).wait()
			rospy.loginfo("Control: Offboard")
		else:
			self.switch_manual()
			
	def bound(self, value, value_min, value_max):
		return min(max(value, value_min), value_max)
		
	def bound_percentage(self, value):
		return self.bound(value, -100, 100)

	def run(self): 
		rate = rospy.Rate(100) # 100hz
		
		rospy.logdebug('MaxTilt = %f [%f, %f]', self.drone.get_state(MaxTiltChanged)["current"], self.drone.get_state(MaxTiltChanged)["min"], self.drone.get_state(MaxTiltChanged)["max"])
		rospy.logdebug('MaxVerticalSpeed = %f [%f, %f]', self.drone.get_state(MaxVerticalSpeedChanged)["current"], self.drone.get_state(MaxVerticalSpeedChanged)["min"], self.drone.get_state(MaxVerticalSpeedChanged)["max"])
		rospy.logdebug('MaxRotationSpeed = %f [%f, %f]', self.drone.get_state(MaxRotationSpeedChanged)["current"], self.drone.get_state(MaxRotationSpeedChanged)["min"], self.drone.get_state(MaxRotationSpeedChanged)["max"])
		rospy.logdebug('MaxPitchRollRotationSpeed = %f [%f, %f]', self.drone.get_state(MaxPitchRollRotationSpeedChanged)["current"], self.drone.get_state(MaxPitchRollRotationSpeedChanged)["min"], self.drone.get_state(MaxPitchRollRotationSpeedChanged)["max"])
		rospy.logdebug('MaxDistance = %f [%f, %f]', self.drone.get_state(MaxDistanceChanged)["current"], self.drone.get_state(MaxDistanceChanged)["min"], self.drone.get_state(MaxDistanceChanged)["max"])
		rospy.logdebug('MaxAltitude = %f [%f, %f]', self.drone.get_state(MaxAltitudeChanged)["current"], self.drone.get_state(MaxAltitudeChanged)["min"], self.drone.get_state(MaxAltitudeChanged)["max"])
		rospy.logdebug('NoFlyOverMaxDistance = %i', self.drone.get_state(NoFlyOverMaxDistanceChanged)["shouldNotFlyOver"])
		rospy.logdebug('BankedTurn = %i', self.drone.get_state(BankedTurnChanged)["state"])
		
		while not rospy.is_shutdown():
			connection = self.drone.connection_state()
			if getattr(connection, 'OK') == False:
				rospy.logfatal(getattr(connection, 'message'))
				self.disconnect()
				self.connect()
			
			# SLOW -- 5Hz			
			#attitude = self.drone.get_state(AttitudeChanged) # attitude
			#rospy.loginfo("Attitude: " + str(attitude))
			#msg_rpy = Vector3Stamped()
			#msg_rpy.header.seq = self.seq
			#msg_rpy.header.stamp = rospy.Time.now()
			#msg_rpy.header.frame_id = '/world'
			#msg_rpy.vector.x = attitude['roll']/math.pi*180
			#msg_rpy.vector.y = -attitude['pitch']/math.pi*180
			#msg_rpy.vector.z = -attitude['yaw']/math.pi*180
			#self.pub_rpy.publish(msg_rpy)
			
			with self.flush_queue_lock:
				try:					
					yuv_frame = self.frame_queue.get(timeout=0.01)
				except queue.Empty:
					continue
				
				try:
					self.yuv_callback(yuv_frame)
				except Exception:
					# Continue popping frame from the queue even if it fails to show one frame
					traceback.print_exc()
					continue
				finally:
					# Unref the yuv frame to avoid starving the video buffer pool
					yuv_frame.unref()
								
			rate.sleep()

class EveryEventListener(olympe.EventListener):
	def __init__(self, anafi):
		self.anafi = anafi
				
		self.msg_rpyt = SkyControllerCommand()
		
		super().__init__(anafi.drone)

	def print_event(self, event): # Serializes an event object and truncates the result if necessary before printing it
		if isinstance(event, olympe.ArsdkMessageEvent):
			max_args_size = 200
			args = str(event.args)
			args = (args[: max_args_size - 3] + "...") if len(args) > max_args_size else args
			rospy.logdebug("{}({})".format(event.message.fullName, args))
		else:
			rospy.logdebug(str(event))

	# RC buttons listener     
	@olympe.listen_event(mapper.grab_button_event()) # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_button_event
	def on_grab_button_event(self, event, scheduler):
		self.print_event(event)
		# button: 	0 = RTL, 1 = takeoff/land, 2 = back left, 3 = back right
		# axis_button:	4 = max CCW yaw, 5 = max CW yaw, 6 = max trottle, 7 = min trottle
		# 		8 = min roll, 9 = max roll, 10 = min pitch, 11 = max pitch
		# 		12 = max camera down, 13 = max camera up, 14 = min zoom, 15 = max zoom
		if event.args["event"] == button_event.press:
			if event.args["button"] == 0: # RTL
				self.anafi.drone(Emergency()).wait()
				rospy.logfatal("Emergency!!!")
				return
			if event.args["button"] == 2: # left back button
				self.anafi.switch_manual()
				return
			if event.args["button"] == 3: # right back button
				self.anafi.switch_offboard()
				self.msg_rpyt = SkyControllerCommand()
				return
        
      	# RC axis listener
	@olympe.listen_event(mapper.grab_axis_event()) # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_axis_event
	def on_grab_axis_event(self, event, scheduler):	
		# axis: 	0 = yaw, 1 = z, 2 = y, 3 = x, 4 = camera, 5 = zoom
		if event.args["axis"] == 0: # yaw
			self.msg_rpyt.yaw = event.args["value"]
		if event.args["axis"] == 1: # z
			self.msg_rpyt.z = event.args["value"]
		if event.args["axis"] == 2: # y/pitch
			self.msg_rpyt.y = event.args["value"]
		if event.args["axis"] == 3: # x/roll
			self.msg_rpyt.x = event.args["value"]
			
		self.msg_rpyt.header.stamp = rospy.Time.now()
		self.msg_rpyt.header.frame_id = '/body'
		self.anafi.pub_skycontroller.publish(self.msg_rpyt)
		          
      	# All other events
	@olympe.listen_event()
	def default(self, event, scheduler):
		#self.print_event(event)
		pass

if __name__ == '__main__':
	rospy.init_node('anafi_bridge', anonymous = False)
	rospy.loginfo("AnafiBridge is running...")
	anafi = Anafi()	
	try:
		anafi.run()
	except rospy.ROSInterruptException:
		traceback.print_exc()
		pass
