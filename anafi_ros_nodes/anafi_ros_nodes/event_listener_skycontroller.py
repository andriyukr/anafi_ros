#!/usr/bin/env python3

import rclpy
import math
import time
import olympe

from std_msgs.msg import Header
from geometry_msgs.msg import QuaternionStamped, Quaternion, Vector3Stamped
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from olympe.messages.mapper import grab_button_event, grab_axis_event, grab_state
from olympe.messages.skyctrl.SkyControllerState import AttitudeChanged
from olympe.enums.mapper import button_event

from anafi_ros_interfaces.msg import SkycontrollerCommand

from anafi_ros_nodes.utils import euler_from_quaternion


def print_event(event):
	if isinstance(event, olympe.ArsdkMessageEvent):
		print(f"{event.message.fullName}({str(event.args)})")
	else:
		print(str(event))


class EventListenerSkyController(olympe.EventListener):
	def __init__(self, skyctrl):
		self.skyctrl = skyctrl
		super().__init__(skyctrl.drone)

		self.pub_command = self.skyctrl.node.create_publisher(SkycontrollerCommand, 'skycontroller/command', qos_profile_system_default)
		self.pub_attitude = self.skyctrl.node.create_publisher(QuaternionStamped, 'skycontroller/attitude', qos_profile_sensor_data)
		self.pub_rpy = self.skyctrl.node.create_publisher(Vector3Stamped, 'skycontroller/rpy', qos_profile_sensor_data)

		self.msg_attitude = QuaternionStamped()
		self.msg_rpy = Vector3Stamped()
		self.msg_command = SkycontrollerCommand()
		self.msg_command.header.frame_id = '/body'
		self.header = Header()
		self.header.frame_id = '/world'

	##########  CALLBACKS  ##########
	# SkyController buttons listener
	@olympe.listen_event(grab_button_event(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_button_event
	def on_grab_button_event(self, event, scheduler):
		# button: 0 = return home, 1 = takeoff/land, 2 = reset camera, 3 = reset zoom
		# axis_button: 4 = max CCW yaw, 5 = max CW yaw, 6 = max throttle, 7 = min throttle
		#              8 = min roll, 9 = max roll, 10 = min pitch, 11 = max pitch
		#              12 = max camera down, 13 = max camera up, 14 = min zoom, 15 = max zoom
		pressed = (event.args["event"] == button_event.press)
		if event.args["button"] == 0:  # return home button
			self.msg_command.return_home = pressed
		elif event.args["button"] == 1:  # takeoff/land button
			self.msg_command.takeoff_land = pressed
		elif event.args["button"] == 2:  # reset camera button
			self.msg_command.reset_camera = pressed
		elif event.args["button"] == 3:  # reset zoom button
			self.msg_command.reset_zoom = pressed
		#self.pub_skyctrl_command.publish(self.msg_command)

	# SkyController axes listener
	@olympe.listen_event(grab_axis_event(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_axis_event
	def on_grab_axis_event(self, event, scheduler):
		# axis: 0 = yaw, 1 = z, 2 = y, 3 = x, 4 = camera, 5 = zoom
		if event.args["axis"] == 0:  # yaw
			self.msg_command.yaw = event.args["value"]
		if event.args["axis"] == 1:  # z
			self.msg_command.z = event.args["value"]
		if event.args["axis"] == 2:  # y/pitch
			self.msg_command.y = event.args["value"]
		if event.args["axis"] == 3:  # x/roll
			self.msg_command.x = event.args["value"]
		if event.args["axis"] == 4:  # camera
			self.msg_command.camera = event.args["value"]
		if event.args["axis"] == 5:  # zoom
			self.msg_command.zoom = event.args["value"]
		#self.pub_skyctrl_command.publish(self.msg_command)

	@olympe.listen_event(grab_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_state
	def on_grab_state(self, event, scheduler):
		# Reset command message
		self.msg_command = SkycontrollerCommand()
		self.msg_command.header.frame_id = '/body'

	def callback(self):  # publishes continuously commands from skycontroller
		self.msg_command.header.stamp = self.skyctrl.node.get_clock().now().to_msg()
		self.pub_command.publish(self.msg_command)

	##########  PUBLISHERS  ##########
	@olympe.listen_event(AttitudeChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_skyctrl_skycontrollerstate.html#olympe.messages.skyctrl.SkyControllerState.AttitudeChanged
	def onAttitudeChanged(self, event, scheduler):
		if rclpy.ok():
			attitude = event.args

			self.header.stamp = self.skyctrl.node.get_clock().now().to_msg()

			self.msg_attitude.header = self.header
			self.msg_attitude.quaternion.x = attitude['q1']
			self.msg_attitude.quaternion.y = -attitude['q2']
			self.msg_attitude.quaternion.z = -attitude['q3']
			self.msg_attitude.quaternion.w = attitude['q0']
			self.pub_attitude.publish(self.msg_attitude)

			(roll, pitch, yaw) = euler_from_quaternion(self.msg_attitude.quaternion)
			self.msg_rpy.header = self.header
			self.msg_rpy.vector.x = roll*180/math.pi
			self.msg_rpy.vector.y = pitch*180/math.pi
			self.msg_rpy.vector.z = yaw*180/math.pi
			self.pub_rpy.publish(self.msg_rpy)
			
	# All other events
	@olympe.listen_event()
	def default(self, event, scheduler):
		pass

