#!/usr/bin/env python3

import rclpy
import os
import shlex
import subprocess
import tempfile
#import threading
import traceback
import time
import logging
import roslib
import sys
import olympe
import olympe_bridge

from rclpy.node import Node
from std_msgs.msg import String

from olympe.messages.drone_manager import connection_state
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.skyctrl.CoPilotingState import pilotingSource
from olympe.messages import mapper
from olympe.enums.mapper import button_event

from anafi_ros_interfaces.msg import SkyControllerCommand

olympe.log.update_config({"loggers": {"olympe": {"level": "ERROR"}}})

SKYCTRL_IP = "192.168.53.1"


class SkyController3(Node):
	def __init__(self):
		rclpy.init(args=sys.argv)
		self.node = rclpy.create_node('skycontroller')

		self.node.get_logger().info("skycontroller is running...")
		
		self.pub_state = self.node.create_publisher(String, "/skycontroller/state")
		self.pub_skycontroller = self.node.create_publisher(SkyControllerCommand, "/skycontroller/command")
		
		# Connect to the SkyController		
		self.skyctrl = olympe.SkyController(SKYCTRL_IP)
		
		# Create listener for RC events
		self.every_event_listener = anafi_autonomy.EventListener(self)
		
		self.msg_skycontroller = SkyControllerCommand()
		self.msg_skycontroller.header.frame_id = '/body'

		self.connect()
		
		self.skyctrl(mapper.grab(buttons=(1<<0|1<<1|1<<2|1<<3), axes=(1<<0|1<<1|1<<2|1<<3|1<<4|1<<5))).wait() # bitfields
		self.skyctrl(setPilotingSource(source="Controller")).wait()
		
	def connect(self):
		self.every_event_listener.subscribe()
		
		rate = rclpy.Rate(1) # 1hz
		while True:
			self.state_msg.data = "CONNECTING"
			self.pub_state.publish(self.state_msg)
			self.node.get_logger().info("Connecting to SkyController")
			connection = self.skyctrl.connect()
			if getattr(connection, 'OK'):
				break
			if rclpy.is_shutdown():
				exit()
			rate.sleep()
		
		# Connect to the SkyController
		self.state_msg.data = "CONNECTED_SKYCONTROLLER"
		self.pub_state.publish(self.state_msg)
		self.node.get_logger().info("Connection to SkyController: " + getattr(connection, 'message'))
			
	def connectAnafi(self):
		rate = rclpy.Rate(1) # 1hz
		# Connect to the drone
		while True:
			if self.skyctrl(connection_state(state="connected", _policy="check")):
				break				
			if rclpy.is_shutdown():
				exit()
			else:
				self.state_msg.data = "SERCHING_DRONE"
				self.pub_state.publish(self.state_msg)
				self.node.get_logger().info_once("Connection to Anafi: " +
												 str(self.skyctrl.get_state(connection_state)['state']))
			rate.sleep()
		self.state_msg.data = "CONNECTED_DRONE"
		self.pub_state.publish(self.state_msg)
		self.node.get_logger().info("Connection to Drone: " + str(self.skyctrl.get_state(connection_state)['state']))
		
	def disconnect(self):
		self.state_msg.data = "DISCONNECTING"
		self.pub_state.publish(self.state_msg)
		self.msg_skycontroller = SkyControllerCommand()
		self.msg_skycontroller.header.stamp = rclpy.Time.now()
		self.pub_skycontroller.publish(self.msg_skycontroller)
		
		self.every_event_listener.unsubscribe()
		#self.skyctrl.stop_video_streaming()
		self.skyctrl.disconnect()
		self.state_msg.data = "DISCONNECTED"
		self.pub_state.publish(self.state_msg)

	def run(self):
		rate = rclpy.Rate(10) # 10hz			
		while not rclpy.is_shutdown():
			connection = self.skyctrl.connect()
			if not getattr(connection, 'OK'):
				self.node.get_logger().fatal(getattr(connection, 'message'))
				self.disconnect()
				self.connect()
				
			self.msg_skycontroller.header.stamp = rclpy.Time.now()
			self.pub_skycontroller.publish(self.msg_skycontroller)
			
			# Publish button pressing event only once
			self.msg_skycontroller.return_home = False
			self.msg_skycontroller.takeoff_land = False
			self.msg_skycontroller.reset_camera = False
			self.msg_skycontroller.reset_zoom = False	
					
			rate.sleep()


if __name__ == '__main__':
	skycontroller = SkyController3()	
	try:
		skycontroller.run()
	except rclpy.ROSInterruptException:
		#traceback.print_exc()
		pass
