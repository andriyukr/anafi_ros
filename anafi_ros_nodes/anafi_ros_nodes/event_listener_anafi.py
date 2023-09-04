#!/usr/bin/env python3

import rclpy
import math
import olympe

from std_msgs.msg import Float32, UInt8, UInt64
from geometry_msgs.msg import Vector3Stamped, PointStamped
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from olympe.messages.ardrone3.SettingsState import MotorErrorStateChanged
from olympe.messages.drone_manager import authentication_failed, connection_refused, connection_state
from olympe.enums.ardrone3.SettingsState import MotorErrorStateChanged_MotorError
from olympe.messages.battery import alert as battery_alert, voltage
from olympe.enums.battery import alert_level
from olympe.messages.gimbal import alert as gimbal_alert, calibration_result, calibration_state, attitude as gimbal_attitude
from olympe.enums import gimbal
from olympe.messages.user_storage import format_result, format_progress, info as user_storage_info, monitor as user_storage_monitor
from olympe.messages.user_storage_v2 import monitor as user_storage_monitor2
from olympe.enums.user_storage import formatting_result
from olympe.messages.ardrone3.PilotingState import AltitudeChanged, AttitudeChanged, GpsLocationChanged, AlertStateChanged, ForcedLandingAutoTrigger, HoveringWarning, VibrationLevelChanged, WindStateChanged, NavigateHomeStateChanged
from olympe.enums.ardrone3.PilotingState import AlertStateChanged_State, ForcedLandingAutoTrigger_Reason, VibrationLevelChanged_State, WindStateChanged_State
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
from olympe.messages.common.CommonState import LinkSignalQuality, MassStorageInfoStateListChanged, SensorsStatesListChanged
from olympe.messages.rth import home_reachability as home_reachability_message, rth_auto_trigger, state as rth_state
from olympe.enums.rth import home_reachability as home_reachability_enum, auto_trigger_reason 
from olympe.messages.common.MavlinkState import MavlinkFilePlayingStateChanged, MissionItemExecuted
from olympe.messages.follow_me import state as follow_me_state, target_trajectory
from olympe.enums.follow_me import mode
from olympe.messages.move import info as move_info
from olympe.messages.camera import zoom_level
from olympe.messages.camera2.Event import ZoomLevel as zoom_level2
from olympe.messages.ardrone3.GPSState import NumberOfSatelliteChanged

from anafi_ros_interfaces.msg import TargetTrajectory


def print_event(event):
	# Here we're just serializing an event object and truncate the result if necessary before printing it.
	if isinstance(event, olympe.ArsdkMessageEvent):
		max_args_size = 60
		args = str(event.args)
		args = (args[: max_args_size - 3] + "...") if len(args) > max_args_size else args
		print(f"{event.message.fullName}({args})")
	else:
		print(str(event))

def show_motors(motors):
	show = ' '
	for i in range(4):
		if motors & (1 << i):
			show += (chr(8598 + i) + ' ')
	return show


class EventListenerAnafi(olympe.EventListener):
	def __init__(self, drone):
		self.drone = drone
		super().__init__(drone.drone)

		self.pub_zoom = self.drone.node.create_publisher(Float32, 'camera/zoom', qos_profile_system_default)
		self.pub_gps_satellites = self.drone.node.create_publisher(UInt8, 'drone/gps/satellites', qos_profile_system_default)
		self.pub_altitude_above_to = self.drone.node.create_publisher(Float32, 'drone/altitude_above_to', qos_profile_sensor_data)
		self.pub_rpy_slow = self.drone.node.create_publisher(Vector3Stamped, 'drone/rpy_slow', qos_profile_sensor_data)
		self.pub_gps_location = self.drone.node.create_publisher(NavSatFix, 'drone/gps/location', qos_profile_sensor_data)
		self.pub_battery_voltage = self.drone.node.create_publisher(Float32, 'battery/voltage', qos_profile_system_default)
		self.pub_target_trajectory = self.drone.node.create_publisher(TargetTrajectory, 'target/trajectory', qos_profile_system_default)
		self.pub_gimbal_relative = self.drone.node.create_publisher(Vector3Stamped, 'gimbal/rpy_slow/relative', qos_profile_sensor_data)
		self.pub_gimbal_absolute = self.drone.node.create_publisher(Vector3Stamped, 'gimbal/rpy_slow/absolute', qos_profile_sensor_data)
		self.pub_storage_available = self.drone.node.create_publisher(UInt64, 'storage/available', qos_profile_system_default)
		self.pub_home_location = self.drone.node.create_publisher(PointStamped, 'home/location', qos_profile_system_default)  # TODO: change to Location message

	""" 
	FATAL ERRORS 
	"""
	@olympe.listen_event(MotorErrorStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_settings_state.html#olympe.messages.ardrone3.SettingsState.MotorErrorStateChanged
	def onMotorErrorStateChanged(self, event, scheduler):
		motor_error = event.args
		if motor_error['motorError'] is not MotorErrorStateChanged_MotorError.noError: # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_settings_state.html#olympe.enums.ardrone3.SettingsState.MotorErrorStateChanged_MotorError
			self.drone.node.get_logger().fatal('Motor Error: motors = %s, error = %s' % 
				    (show_motors(motor_error['motorIds']), motor_error['motorError'].name))

	@olympe.listen_event(authentication_failed(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.authentication_failed
	def onAuthenticationFailed(self, event, scheduler):
		self.drone.node.get_logger().fatal('Authentication failed because of a wrong key (passphrase)')

	@olympe.listen_event(connection_refused(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.connection_refused
	def onConnectionRefused(self, event, scheduler):
		self.drone.node.get_logger().fatal('Connection refused by the drone because another peer is already connected')

	""" 
	ERRORS 
	"""
	@olympe.listen_event(battery_alert(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.alert
	def onBatteryAlert(self, event, scheduler):
		alert = event.args
		if alert['level'] is alert_level.critical:
			self.drone.node.get_logger().error("Battery Allert: " + alert['alert'].name, throttle_duration_sec=1)
		if alert['level'] is alert_level.warning:
			self.drone.node.get_logger().warning("Battery Allert: " + alert['alert'].name, throttle_duration_sec=60)

	@olympe.listen_event(gimbal_alert(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.alert
	def onGimbalAlert(self, event, scheduler):
		alert = event.args
		for error in gimbal.error:
			if (1<<error.value) & alert['error']:
				self.drone.node.get_logger().error("Gimbal Allert: " + error.name)

	@olympe.listen_event(format_result(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.format_result
	def onFormatResult(self, event, scheduler):
		format_result = event.args['result']  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.enums.user_storage.formatting_result
		if format_result is formatting_result.error:
			self.drone.node.get_logger().error('Formatting failed')
		if format_result is formatting_result.denied:
			self.drone.node.get_logger().warning('Formatting was denied')
		if format_result is formatting_result.success:
			self.drone.node.get_logger().info('Formatting succeeded')

	""" 
	WARNINGS 
	"""
	@olympe.listen_event(AlertStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AlertStateChanged
	def onAlertStateChanged(self, event, scheduler):
		alert_state = event.args['state']
		if alert_state is not AlertStateChanged_State.none and alert_state is not AlertStateChanged_State.user:  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.AlertStateChanged_State
			self.drone.node.get_logger().warning("Alert: " + alert_state.name, throttle_duration_sec=60)

	@olympe.listen_event(ForcedLandingAutoTrigger(_policy="wait")) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.ForcedLandingAutoTrigger
	def onForcedLandingAutoTrigger(self, event, scheduler):
		forced_landing = event.args['reason']
		if forced_landing is not ForcedLandingAutoTrigger_Reason.NONE:
			self.drone.node.get_logger().warning("Forced Landing Reason: " + forced_landing.name, throttle_duration_sec=60)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.ForcedLandingAutoTrigger_Reason

	@olympe.listen_event(HoveringWarning(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.HoveringWarning
	def onHoveringWarning(self, event, scheduler):
		hovering_warning = event.args
		if (self.drone.state == "HOVERING" or self.drone.state == "FLYING"):
			if hovering_warning['no_gps_too_dark']:
				self.drone.node.get_logger().warning("Hovering Warning: the drone doesn’t have a GPS fix AND there is not enough light", throttle_duration_sec=60)
			if hovering_warning['no_gps_too_high']:
				self.drone.node.get_logger().warning("Hovering Warning: the drone doesn’t have a GPS fix AND is flying too high", throttle_duration_sec=60)

	@olympe.listen_event(VibrationLevelChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.VibrationLevelChanged
	def onVibrationLevelChanged(self, event, scheduler):
		vibration_level = event.args['state']
		if vibration_level is not VibrationLevelChanged_State.ok:  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.VibrationLevelChanged_State
			self.drone.node.get_logger().warning("Vibration Level: " + vibration_level.name, throttle_duration_sec=60)

	@olympe.listen_event(WindStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.WindStateChanged
	def onWindStateChanged(self, event, scheduler):
		wind_state = event.args['state']
		if wind_state is not WindStateChanged_State.ok:  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.WindStateChanged_State
			self.drone.node.get_logger().warning("Wind State: " + wind_state.name, throttle_duration_sec=60)

	@olympe.listen_event(LinkSignalQuality(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.LinkSignalQuality
	def onAlertStateChanged(self, event, scheduler):
		link_quality = event.args['value']
		if (link_quality>>6)&1:
			self.drone.node.get_logger().warning("4G interference coming from the smartphone", throttle_duration_sec=60)
		if (link_quality>>7)&1:
			self.drone.node.get_logger().warning("Radio link is perturbed by external elements", throttle_duration_sec=60)

	@olympe.listen_event(MassStorageInfoStateListChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.MassStorageInfoStateListChanged
	def onMassStorageInfoStateListChanged(self, event, scheduler):
		mass_storage = event.args
		self.drone.node.get_logger().info_once('Mass storage info: id = %i, size = %iMB, used = %iMB',
						   mass_storage['mass_storage_id'], mass_storage['size'], mass_storage['used_size'])
		if mass_storage['plugged'] == 0:
			self.drone.node.get_logger().warning('Mass storage is not plugged', throttle_duration_sec=60)
		if mass_storage['full'] == 1:
			self.drone.node.get_logger().warning('Mass storage is full', throttle_duration_sec=60)

	@olympe.listen_event(SensorsStatesListChanged())  # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.SensorsStatesListChanged
	def onSensorsStatesListChanged(self, event, scheduler):
		sensor = event.args
		if sensor['sensorState'] == 0:
			self.drone.node.get_logger().warning('%s is NOT OK' % sensor['sensorName'].name)

	@olympe.listen_event(home_reachability_message(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.home_reachability
	def onHomeReachability(self, event, scheduler):
		home_reachability = event.args['status']  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.home_reachability
		if home_reachability is home_reachability_enum.unknown:
			self.drone.node.get_logger().warning('Home reachability is unknown')
		if home_reachability is home_reachability_enum.reachable:
			self.drone.node.get_logger().info('Home is reachable')
		if home_reachability is home_reachability_enum.critical:
			self.drone.node.get_logger().warning('Home is still reachable but won’t be if rth is not triggered now')
		if home_reachability is home_reachability_enum.reachable:
			self.drone.node.get_logger().warning('Home is not reachable')

	@olympe.listen_event(rth_auto_trigger(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.rth_auto_trigger
	def onRTHAutoTrigger(self, event, scheduler):
		rth_auto_trigger = event.args['reason']  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.auto_trigger_reason
		if rth_auto_trigger is auto_trigger_reason.battery_critical_soon:
			self.drone.node.get_logger().warning('Battery will soon be critical', throttle_duration_sec=60)

	"""
	INFO
	"""
	@olympe.listen_event(MavlinkFilePlayingStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MavlinkFilePlayingStateChanged
	def onMissionItemExecuted(self, event, scheduler):
		self.drone.node.get_logger().info('FlightPlan state is %s' % event.args['state'].name)  # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MavlinkFilePlayingStateChanged

	@olympe.listen_event(MissionItemExecuted(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MissionItemExecuted
	def onMissionItemExecuted(self, event, scheduler):
		self.drone.node.get_logger().info('Mission item #%i executed' % event.args['idx'])

	@olympe.listen_event(follow_me_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.state
	def onFollowMeState(self, event, scheduler):
		follow_me = event.args
		if follow_me['mode'] != mode.none:
			self.drone.node.get_logger().info('FollowMe state: mode=%s, behavior=%s, animation=%s' %
										(follow_me['mode'].name, follow_me['behavior'].name,
										 follow_me['animation'].name))

	@olympe.listen_event(calibration_result(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.calibration_result
	def onGimbalCalibrationResult(self, event, scheduler):
		calibration_result = event.args
		self.drone.node.get_logger().info('Gimbal calibration result: ' + calibration_result['result'].name)

	@olympe.listen_event(move_info(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_move.html#olympe.messages.move.info
	def onMoveInfo(self, event, scheduler):
		move_info = event.args
		self.drone.node.get_logger().info('Move info: ' + str(move_info))

	@olympe.listen_event(format_progress(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.format_progress
	def onFormatProgress(self, event, scheduler):
		format_progress = event.args
		self.drone.node.get_logger().info('Formatting --> %s (%i%%)' %
									(format_progress['step'].name, format_progress['percentage']))

	@olympe.listen_event(user_storage_info(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.info
	def onUserStorageInfo(self, event, scheduler):
		capacity = event.args['capacity']
		available_bytes = self.drone.drone.get_state(user_storage_monitor)['available_bytes']  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.monitor
		self.drone.node.get_logger().info_once('Available space: %.1f/%.1fGB' % (available_bytes/(2**30), capacity/(2**30)))

	""" 
	DEBUG  
	"""
	@olympe.listen_event(NavigateHomeStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.NavigateHomeStateChanged
	def onNavigateHomeStateChanged(self, event, scheduler):
		navigate_home_state = event.args
		self.drone.node.get_logger().debug("Navigate Home State: state = %s, reason = %s" %
									(navigate_home_state['state'].name, navigate_home_state['reason'].name))

	@olympe.listen_event(rth_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.state
	def onRTHState(self, event, scheduler):
		state = event.args
		self.drone.node.get_logger().debug('RTH: state=%s, reason=%s' % (state['state'].name, state['reason'].name))

	@olympe.listen_event(calibration_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.calibration_state
	def onGimbalCalibrationState(self, event, scheduler):
		calibration_state = event.args
		self.drone.node.get_logger().debug('Gimbal calibration state: ' + calibration_state['state'].name)

	@olympe.listen_event(connection_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.connection_state
	def onConnectionState(self, event, scheduler):
		self.drone.node.get_logger().debug("connection_state: " + str(event.args))

	"""
	PUBLISHERS
	"""
	@olympe.listen_event(zoom_level(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.zoom_level
	def onZoomLevel(self, event, scheduler):  # for ANAFI 4K, Thermal, USA 
		msg_zoom = Float32()
		msg_zoom.data = event.args['level']
		self.pub_zoom.publish(msg_zoom)
		
	@olympe.listen_event(zoom_level2(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_camera_v2.html#olympe.messages.camera2.Event.ZoomLevel
	def onZoomLevel2(self, event, scheduler):  # for ANAFI Ai
		msg_zoom = Float32()
		msg_zoom.data = event.args['level']
		self.pub_zoom.publish(msg_zoom)

	@olympe.listen_event(NumberOfSatelliteChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_gps.html#olympe.messages.ardrone3.GPSState.NumberOfSatelliteChanged
	def onNumberOfSatelliteChanged(self, event, scheduler):
		msg_gps_satellites = UInt8()
		msg_gps_satellites.data = event.args['numberOfSatellite']
		self.pub_gps_satellites.publish(msg_gps_satellites)

	@olympe.listen_event(AltitudeChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AltitudeChanged
	def onAltitudeChanged(self, event, scheduler):
		msg_altitude = Float32()
		msg_altitude.data = event.args['altitude']
		self.pub_altitude_above_to.publish(msg_altitude)
		
	@olympe.listen_event(AttitudeChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AttitudeChanged
	def onAttitudeChanged(self, event, scheduler):  # publishes at lower rate (5Hz) than 'pub_rpy' (30Hz) but has higher reaction time (approx. 100ms faster)
		attitude = event.args
		msg_attitude = Vector3Stamped()
		msg_attitude.header.stamp = self.drone.node.get_clock().now().to_msg()
		msg_attitude.header.frame_id = '/world'
		msg_attitude.vector.x = attitude['roll']*180/math.pi
		msg_attitude.vector.y = -attitude['pitch']*180/math.pi
		msg_attitude.vector.z = -attitude['yaw']*180/math.pi
		self.pub_rpy_slow.publish(msg_attitude)

	@olympe.listen_event(GpsLocationChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.GpsLocationChanged
	def onGpsLocationChanged(self, event, scheduler):
		gps_location = event.args
		msg_gps_location = NavSatFix()
		msg_gps_location.header.stamp = self.drone.node.get_clock().now().to_msg()
		msg_gps_location.header.frame_id = '/world'
		msg_gps_location.status.status = \
			(msg_gps_location.status.STATUS_FIX
			 if self.drone.gps_fixed
			 else msg_gps_location.status.STATUS_NO_FIX) # https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatStatus.html
		if self.drone.model == "4k" or self.drone.model == "thermal":
			msg_gps_location.status.service = \
				msg_gps_location.status.SERVICE_GPS + \
				msg_gps_location.status.SERVICE_GLONASS
		if self.drone.model == "usa" or self.drone.model == "ai":
			msg_gps_location.status.service = \
				msg_gps_location.status.SERVICE_GPS + \
				msg_gps_location.status.SERVICE_GLONASS + \
				msg_gps_location.status.SERVICE_GALILEO
		if gps_location['latitude'] != 500.0 and gps_location['longitude'] != 500.0:
			msg_gps_location.latitude = gps_location['latitude']
			msg_gps_location.longitude = gps_location['longitude']
			msg_gps_location.altitude = gps_location['altitude']
		else:
			msg_gps_location.latitude = float('nan')
			msg_gps_location.longitude = float('nan')
			msg_gps_location.altitude = float('nan')
		msg_gps_location.position_covariance[0] = \
			(gps_location['latitude_accuracy']**2  # https://answers.ros.org/question/10310/calculate-navsatfix-covariance/
			 if (gps_location['latitude_accuracy'] >= 0 and gps_location['latitude'] != 500.0)
			 else float('nan'))
		msg_gps_location.position_covariance[4] = \
			(gps_location['longitude_accuracy']**2  # https://answers.ros.org/question/10310/calculate-navsatfix-covariance/
			 if (gps_location['longitude_accuracy'] >= 0 and gps_location['longitude'] != 500.0)
			 else float('nan'))
		msg_gps_location.position_covariance[8] = \
			(gps_location['altitude_accuracy']**2  # https://answers.ros.org/question/10310/calculate-navsatfix-covariance/
			 if gps_location['altitude_accuracy'] >= 0
			 else float('nan'))
		msg_gps_location.position_covariance_type = msg_gps_location.COVARIANCE_TYPE_DIAGONAL_KNOWN
		self.pub_gps_location.publish(msg_gps_location)

	@olympe.listen_event(voltage(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.voltage
	def onBatteryVoltage(self, event, scheduler):
		msg_battery_voltage = Float32()
		msg_battery_voltage.data = event.args['voltage']/1000
		self.pub_battery_voltage.publish(msg_battery_voltage)

	@olympe.listen_event(target_trajectory(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.target_trajectory
	def onTargetTrajectory(self, event, scheduler):
		trajectory = event.args
		self.drone.node.get_logger().info('target_trajectory: ' + str(trajectory))
		msg_trajectory = TargetTrajectory()
		msg_trajectory.header.stamp = self.drone.node.get_clock().now().to_msg()
		msg_trajectory.header.frame_id = '/world'
		msg_trajectory.latitude = trajectory['latitude']
		msg_trajectory.longitude = trajectory['longitude']
		msg_trajectory.altitude = trajectory['altitude']
		msg_trajectory.north_speed = trajectory['north_speed']
		msg_trajectory.east_speed = trajectory['east_speed']
		msg_trajectory.down_speed = trajectory['down_speed']
		self.pub_target_trajectory.publish(msg_trajectory)

	@olympe.listen_event(gimbal_attitude(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.attitude
	def onGimbalAttitude(self, event, scheduler):
		gimbal = event.args
		msg_gimbal = Vector3Stamped()
		msg_gimbal.header.stamp = self.drone.node.get_clock().now().to_msg()

		msg_gimbal.header.frame_id = '/world'
		msg_gimbal.vector.x = gimbal['roll_absolute']
		msg_gimbal.vector.y = -gimbal['pitch_absolute']
		msg_gimbal.vector.z = -gimbal['yaw_absolute']
		self.pub_gimbal_absolute.publish(msg_gimbal)

		msg_gimbal.header.frame_id = '/body'
		msg_gimbal.vector.x = gimbal['roll_relative']
		msg_gimbal.vector.y = -gimbal['pitch_relative']
		msg_gimbal.vector.z = -gimbal['yaw_relative']
		self.pub_gimbal_relative.publish(msg_gimbal)

	@olympe.listen_event(user_storage_monitor(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.monitor
	def onUserStorageMonitor(self, event, scheduler):  # for ANAFI 4K, Thermal, USA
		msg_storage_available = UInt64()
		msg_storage_available.data = event.args['available_bytes']
		self.pub_storage_available.publish(msg_storage_available)
		
	@olympe.listen_event(user_storage_monitor2(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage_v2.html#olympe.messages.user_storage_v2.monitor
	def onUserStorageMonitor2(self, event, scheduler):  # for ANAFI Ai
		msg_storage_available = UInt64()
		msg_storage_available.data = event.args['available_bytes']
		self.pub_storage_available.publish(msg_storage_available)

	@olympe.listen_event(HomeChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_gps.html#olympe.messages.ardrone3.GPSSettingsState.HomeChanged
	def onHomeChanged(self, event, scheduler):
		home = event.args
		self.drone.node.get_logger().info('Home: ' + str(home))
		if home['latitude'] != 500 and home['longitude'] != 500 and home['altitude'] != 500:
			msg_home_location = PointStamped()
			msg_home_location.header.stamp = self.drone.node.get_clock().now().to_msg()
			msg_home_location.header.frame_id = '/world'
			msg_home_location.point.x = home['latitude']
			msg_home_location.point.y = home['longitude']
			msg_home_location.point.z = home['altitude']
			self.pub_home_location.publish(msg_home_location)

	""" 
	All other events
	"""
	@olympe.listen_event()
	def default(self, event, scheduler):
		pass
