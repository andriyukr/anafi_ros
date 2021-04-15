#!/usr/bin/env python3

import olympe

# Event listener		
class EveryEventListener(olympe.EventListener):
	def __init__(self, anafi):
		self.anafi = anafi
		
		self.pub_skycontroller = rospy.Publisher("/skycontroller/command", TwistStamped, queue_size=1)	
				
		self.trottle = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		
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
				return
        
      	# RC axis listener
	@olympe.listen_event(mapper.grab_axis_event()) # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_axis_event
	def on_grab_axis_event(self, event, scheduler):
		self.print_event(event)
		# axis: 	0 = yaw, 1 = trottle, 2 = roll, 3 = pithch, 4 = camera, 5 = zoom
		if event.args["axis"] == 0: # yaw
			self.yaw = event.args["value"]
		if event.args["axis"] == 1: # trottle
			self.trottle = event.args["value"]
		if event.args["axis"] == 2: # roll
			self.roll = event.args["value"]
		if event.args["axis"] == 3: # pitch
			self.pitch = event.args["value"]
			
		msg_rpyt = TwistStamped()
		msg_rpyt.header.stamp = rospy.Time.now()
		msg_rpyt.header.frame_id = '/body'
		msg_rpyt.twist.linear.z = self.trottle
		msg_rpyt.twist.angular.x = self.roll
		msg_rpyt.twist.angular.y = self.pitch
		msg_rpyt.twist.angular.z = self.yaw
		self.pub_skycontroller.publish(msg_rpyt)
		          
      	# All other events
	@olympe.listen_event()
	def default(self, event, scheduler):
		pass
