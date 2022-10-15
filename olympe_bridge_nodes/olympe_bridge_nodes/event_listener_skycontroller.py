#!/usr/bin/env python3

import rclpy
import math
import olympe

from std_msgs.msg import Header
from geometry_msgs.msg import QuaternionStamped, Quaternion, Vector3Stamped
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from olympe.messages.mapper import grab_button_event, grab_axis_event
from olympe.messages.skyctrl.SkyControllerState import AttitudeChanged
from olympe.enums.mapper import button_event

from olympe_bridge_nodes.utils import euler_from_quaternion


def print_event(event):
	# Here we're just serializing an event object and truncate the result if necessary before printing it.
	if isinstance(event, olympe.ArsdkMessageEvent):
		max_args_size = 60
		args = str(event.args)
		args = (args[: max_args_size - 3] + "...") if len(args) > max_args_size else args
		print(f"{event.message.fullName}({args})")
	else:
		print(str(event))


class EventListenerSkyController(olympe.EventListener):
	def __init__(self, skyctrl):
		self.skyctrl = skyctrl
		super().__init__(skyctrl.skyctrl)

		self.pub_skyctrl_attitude = \
			self.skyctrl.node.create_publisher(QuaternionStamped, 'skycontroller/attitude', qos_profile_sensor_data)
		self.pub_skyctrl_rpy = \
			self.skyctrl.node.create_publisher(Vector3Stamped, 'skycontroller/rpy', qos_profile_sensor_data)

		self.msg_attitude = QuaternionStamped()
		self.msg_rpy = Vector3Stamped()
		self.header = Header()
		self.header.frame_id = '/world'
	
	##########  CALLBACKS  ##########
	# SkyController buttons listener
	@olympe.listen_event(grab_button_event(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_button_event
	def on_grab_button_event(self, event, scheduler):
		# button: 	0 = return home, 1 = takeoff/land, 2 = reset camera, 3 = reset zoom
		# axis_button:	4 = max CCW yaw, 5 = max CW yaw, 6 = max throttle, 7 = min throttle
		# 		8 = min roll, 9 = max roll, 10 = min pitch, 11 = max pitch
		# 		12 = max camera down, 13 = max camera up, 14 = min zoom, 15 = max zoom
		if event.args["event"] == button_event.press:
			if event.args["button"] == 0:  # return home button
				self.skyctrl.msg_skycontroller.return_home = True
			elif event.args["button"] == 1:  # takeoff/land button
				self.skyctrl.msg_skycontroller.takeoff_land = True
			elif event.args["button"] == 2:  # reset camera button
				self.skyctrl.msg_skycontroller.reset_camera = True
			elif event.args["button"] == 3:  # reset zoom button
				self.skyctrl.msg_skycontroller.reset_zoom = True

	# SkyController axes listener
	@olympe.listen_event(grab_axis_event(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_axis_event
	def on_grab_axis_event(self, event, scheduler):
		# axis: 	0 = yaw, 1 = z, 2 = y, 3 = x, 4 = camera, 5 = zoom
		if event.args["axis"] == 0:  # yaw
			self.skyctrl.msg_skycontroller.yaw = event.args["value"]
		if event.args["axis"] == 1:  # z
			self.skyctrl.msg_skycontroller.z = event.args["value"]
		if event.args["axis"] == 2:  # y/pitch
			self.skyctrl.msg_skycontroller.y = event.args["value"]
		if event.args["axis"] == 3:  # x/roll
			self.skyctrl.msg_skycontroller.x = event.args["value"]
		if event.args["axis"] == 4:  # camera
			self.skyctrl.msg_skycontroller.camera = event.args["value"]
		if event.args["axis"] == 5:  # zoom
			self.skyctrl.msg_skycontroller.zoom = event.args["value"]

	##########  PUBLISHERS  ##########
	@olympe.listen_event(AttitudeChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_skyctrl_skycontrollerstate.html#olympe.messages.skyctrl.SkyControllerState.AttitudeChanged
	def onAttitudeChanged(self, event, scheduler):
		attitude = event.args

		self.header.stamp = self.skyctrl.node.get_clock().now().to_msg()

		self.msg_attitude.header = self.header
		self.msg_attitude.quaternion = Quaternion(
			x=attitude['q1'],
			y=-attitude['q2'],
			z=-attitude['q3'],
			w=attitude['q0'])
		self.pub_skyctrl_attitude.publish(self.msg_attitude)

		(roll, pitch, yaw) = euler_from_quaternion(self.msg_attitude.quaternion)
		self.msg_rpy.header = self.header
		self.msg_rpy.vector.x = roll*180/math.pi
		self.msg_rpy.vector.y = pitch*180/math.pi
		self.msg_rpy.vector.z = yaw*180/math.pi
		self.pub_skyctrl_rpy.publish(self.msg_rpy)
			
	# All other events
	@olympe.listen_event()
	def default(self, event, scheduler):
		pass
