#!/usr/bin/env python3

import rospy
import math
import olympe

from std_msgs.msg import Float32, UInt8, UInt64
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix

from olympe.messages.ardrone3.SettingsState import MotorErrorStateChanged
from olympe.messages.drone_manager import authentication_failed, connection_refused
from olympe.enums.ardrone3.SettingsState import MotorErrorStateChanged_MotorError
from olympe.messages.battery import alert as battery_alert
from olympe.enums.battery import alert_level
from olympe.messages.gimbal import alert as gimbal_alert
from olympe.enums import gimbal
from olympe.messages.user_storage import format_result
from olympe.enums.user_storage import formatting_result
from olympe.messages.ardrone3.PilotingState import AlertStateChanged
from olympe.enums.ardrone3.PilotingState import AlertStateChanged_State
from olympe.messages.ardrone3.PilotingState import ForcedLandingAutoTrigger
from olympe.enums.ardrone3.PilotingState import ForcedLandingAutoTrigger_Reason
from olympe.messages.ardrone3.PilotingState import HoveringWarning
from olympe.messages.ardrone3.PilotingState import VibrationLevelChanged
from olympe.enums.ardrone3.PilotingState import VibrationLevelChanged_State
from olympe.messages.ardrone3.PilotingState import WindStateChanged
from olympe.enums.ardrone3.PilotingState import WindStateChanged_State
from olympe.messages.common.CommonState import LinkSignalQuality
from olympe.messages.common.CommonState import MassStorageInfoStateListChanged
from olympe.messages.common.CommonState import SensorsStatesListChanged
from olympe.messages.rth import home_reachability as home_reachability_message
from olympe.enums.rth import home_reachability as home_reachability_enum
from olympe.messages.rth import rth_auto_trigger
from olympe.enums.rth import auto_trigger_reason
from olympe.messages.ardrone3.PilotingState import NavigateHomeStateChanged
from olympe.messages.common.MavlinkState import MavlinkFilePlayingStateChanged
from olympe.messages.common.MavlinkState import MissionItemExecuted
from olympe.messages.drone_manager import connection_state
from olympe.messages.follow_me import state as follow_me_state
from olympe.enums.follow_me import mode
from olympe.messages.gimbal import calibration_result
from olympe.messages.move import info as move_info
from olympe.messages.rth import state as rth_state
from olympe.messages.user_storage import format_progress
from olympe.messages.user_storage import info as user_storage_info
from olympe.messages.user_storage import monitor as user_storage_monitor
from olympe.messages.gimbal import calibration_state
from olympe.messages.camera import zoom_level
from olympe.messages.ardrone3.GPSState import NumberOfSatelliteChanged
from olympe.messages.ardrone3.PilotingState import AltitudeChanged
from olympe.messages.ardrone3.PilotingState import AttitudeChanged
from olympe.messages.ardrone3.PilotingState import GpsLocationChanged
from olympe.messages.battery import voltage
from olympe.messages.follow_me import target_trajectory
from olympe.messages.gimbal import attitude as gimbal_attitude

from olympe_bridge.msg import TargetTrajectory


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
		
		# Publishers
		self.drone.pub_zoom = rospy.Publisher("camera/zoom", Float32, queue_size=1)
		self.drone.pub_gps_satellites = rospy.Publisher("drone/gps/satellites", UInt8, queue_size=1)
		self.drone.pub_altitude_above_TO = rospy.Publisher("drone/altitude_above_TO", Float32, queue_size=1)
		self.drone.pub_rpy_slow = rospy.Publisher("drone/rpy_slow", Vector3Stamped, queue_size=1)
		self.drone.pub_gps_location = rospy.Publisher("drone/gps/location", NavSatFix, queue_size=1)
		self.drone.pub_battery_voltage = rospy.Publisher("battery/voltage", Float32, queue_size=1)
		self.drone.pub_target_trajectory = rospy.Publisher("target/trajectory", TargetTrajectory, queue_size=1)
		self.drone.pub_gimbal_relative = rospy.Publisher("gimbal/relative", Vector3Stamped, queue_size=1)
		self.drone.pub_gimbal_absolute = rospy.Publisher("gimbal/absolute", Vector3Stamped, queue_size=1)
		self.drone.pub_media_available = rospy.Publisher("media/available", UInt64, queue_size=1)

		# Messages
		self.msg_attitude = Vector3Stamped()
		self.msg_gps_location = NavSatFix()
		self.msg_trajectory = TargetTrajectory()
		self.msg_gimbal = Vector3Stamped()

	""" 
	FATAL ERRORS 
	"""
	@olympe.listen_event(MotorErrorStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_settings_state.html#olympe.messages.ardrone3.SettingsState.MotorErrorStateChanged
	def onMotorErrorStateChanged(self, event, scheduler):
		motor_error = event.args
		if motor_error['motorError'] is not MotorErrorStateChanged_MotorError.noError: # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_settings_state.html#olympe.enums.ardrone3.SettingsState.MotorErrorStateChanged_MotorError
			rospy.logfatal('Motor Error: motors = %s, error = %s',
						   show_motors(motor_error['motorIds']), motor_error['motorError'].name)

	@olympe.listen_event(authentication_failed(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.authentication_failed
	def on_authentication_failed(self, event, scheduler):
		rospy.logfatal('Authentication failed because of a wrong key (passphrase)')

	@olympe.listen_event(connection_refused(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.connection_refused
	def on_connection_refused(self, event, scheduler):
		rospy.logfatal('Connection refused by the drone because another peer is already connected')

	""" 
	ERRORS 
	"""
	@olympe.listen_event(battery_alert(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.alert
	def onBatteryAlert(self, event, scheduler):
		alert = event.args
		if alert['level'] is alert_level.critical:
			rospy.logerr_throttle(1, "Battery Allert: " + alert['alert'].name)
		if alert['level'] is alert_level.warning:
			rospy.logwarn_throttle(60, "Battery Allert: " + alert['alert'].name)

	@olympe.listen_event(gimbal_alert(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.alert
	def onGimbalAlert(self, event, scheduler):
		alert = event.args
		for error in gimbal.error:
			if (1<<error.value) & alert['error']:
				rospy.logerr("Gimbal Allert: " + error.name)

	@olympe.listen_event(format_result(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.format_result
	def on_format_result(self, event, scheduler):
		format_result = event.args['result']  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.enums.user_storage.formatting_result
		if format_result is formatting_result.error:
			rospy.logerr('Formatting failed')
		if format_result is formatting_result.denied:
			rospy.logwarn('Formatting was denied')
		if format_result is formatting_result.success:
			rospy.loginfo('Formatting succeeded')

	""" 
	WARNINGS 
	"""
	@olympe.listen_event(AlertStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AlertStateChanged
	def onAlertStateChanged(self, event, scheduler):
		alert_state = event.args['state']
		if alert_state is not AlertStateChanged_State.none and alert_state is not AlertStateChanged_State.user:  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.AlertStateChanged_State
			rospy.logwarn_throttle(60, "Alert: " + alert_state.name)

	@olympe.listen_event(ForcedLandingAutoTrigger(_policy="wait")) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.ForcedLandingAutoTrigger
	def onForcedLandingAutoTrigger(self, event, scheduler):
		forced_landing = event.args['reason']
		if forced_landing is not ForcedLandingAutoTrigger_Reason.NONE:
			rospy.logwarn_throttle(60, "Forced Landing Reason: " + forced_landing.name)  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.ForcedLandingAutoTrigger_Reason

	@olympe.listen_event(HoveringWarning(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.HoveringWarning
	def onHoveringWarning(self, event, scheduler):
		hovering_warning = event.args
		if (self.drone.state == "HOVERING" or self.drone.state == "FLYING"):
			if hovering_warning['no_gps_too_dark']:
				rospy.logwarn_throttle(60, "Hovering Warning: the drone doesn’t have a GPS fix AND there is not enough light")
			if hovering_warning['no_gps_too_high']:
				rospy.logwarn_throttle(60, "Hovering Warning: the drone doesn’t have a GPS fix AND is flying too high")

	@olympe.listen_event(VibrationLevelChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.VibrationLevelChanged
	def onVibrationLevelChanged(self, event, scheduler):
		vibration_level = event.args['state']
		if vibration_level is not VibrationLevelChanged_State.ok:  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.VibrationLevelChanged_State
			rospy.logwarn_throttle(60, "Vibration Level: " + vibration_level.name)

	@olympe.listen_event(WindStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.WindStateChanged
	def onWindStateChanged(self, event, scheduler):
		wind_state = event.args['state']
		if wind_state is not WindStateChanged_State.ok:  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.WindStateChanged_State
			rospy.logwarn_throttle(60, "Wind State: " + wind_state.name)

	@olympe.listen_event(LinkSignalQuality(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.LinkSignalQuality
	def onAlertStateChanged(self, event, scheduler):
		link_quality = event.args['value']
		if (link_quality>>6)&1:
			rospy.logwarn_throttle(60, "4G interference coming from the smartphone")
		if (link_quality>>7)&1:
			rospy.logwarn_throttle(60, "Radio link is perturbed by external elements")

	@olympe.listen_event(MassStorageInfoStateListChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.MassStorageInfoStateListChanged
	def onMassStorageInfoStateListChanged(self, event, scheduler):
		mass_storage = event.args
		rospy.loginfo_once('Mass storage info: id = %i, size = %iMB, used = %iMB',
						   mass_storage['mass_storage_id'], mass_storage['size'], mass_storage['used_size'])
		if mass_storage['plugged'] == 0:
			rospy.logwarn_throttle(60, 'Mass storage is not plugged')
		if mass_storage['full'] == 1:
			rospy.logwarn_throttle(60, 'Mass storage is full')

	@olympe.listen_event(SensorsStatesListChanged())  # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.SensorsStatesListChanged
	def onSensorsStatesListChanged(self, event, scheduler):
		sensor = event.args
		if sensor['sensorState'] == 0:
			rospy.logwarn('%s is NOT OK', sensor['sensorName'].name)

	@olympe.listen_event(home_reachability_message(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.home_reachability
	def on_home_reachability(self, event, scheduler):
		home_reachability = event.args['status']  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.home_reachability
		if home_reachability is home_reachability_enum.unknown:
			rospy.logwarn('Home reachability is unknown')
		if home_reachability is home_reachability_enum.reachable:
			rospy.loginfo('Home is reachable')
		if home_reachability is home_reachability_enum.critical:
			rospy.logwarn('Home is still reachable but won’t be if rth is not triggered now')
		if home_reachability is home_reachability_enum.reachable:
			rospy.logwarn('Home is not reachable')

	@olympe.listen_event(rth_auto_trigger(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.rth_auto_trigger
	def on_home_reachability(self, event, scheduler):
		rth_auto_trigger = event.args['reason']  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.auto_trigger_reason
		if rth_auto_trigger is auto_trigger_reason.battery_critical_soon:
			rospy.logwarn_throttle(60, 'Battery will soon be critical')

	"""
	INFO
	"""
	@olympe.listen_event(NavigateHomeStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.NavigateHomeStateChanged
	def onNavigateHomeStateChanged(self, event, scheduler):
		navigate_home_state = event.args
		rospy.loginfo("Navigate Home State: state = %s, reason = %s",
					  navigate_home_state['state'].name, navigate_home_state['reason'].name)

	@olympe.listen_event(MavlinkFilePlayingStateChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MavlinkFilePlayingStateChanged
	def onMissionItemExecuted(self, event, scheduler):
		rospy.loginfo('FlightPlan state is %s', event.args['state'].name)  # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MavlinkFilePlayingStateChanged

	@olympe.listen_event(MissionItemExecuted(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MissionItemExecuted
	def onMissionItemExecuted(self, event, scheduler):
		rospy.loginfo('Mission item #%i executed', event.args['idx'])

	@olympe.listen_event(connection_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.connection_state
	def on_connection_state(self, event, scheduler):
		rospy.loginfo("connection_state: " + str(event.args))

	@olympe.listen_event(follow_me_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.state
	def on_follow_me_state(self, event, scheduler):
		follow_me = event.args
		if follow_me['mode'] != mode.none:
			rospy.loginfo('FollowMe state: mode=%s, behavior=%s, animation=%s',
						  follow_me['mode'].name, follow_me['behavior'].name, follow_me['animation'].name)

	@olympe.listen_event(calibration_result(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.calibration_result
	def on_gimbal_calibration_result(self, event, scheduler):
		calibration_result = event.args
		rospy.loginfo('Gimbal calibration result: ' + calibration_result['result'].name)

	@olympe.listen_event(move_info(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_move.html#olympe.messages.move.info
	def on_move_info(self, event, scheduler):
		move_info = event.args
		rospy.loginfo('Move info: ' + str(move_info))

	@olympe.listen_event(rth_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.state
	def on_rth_state(self, event, scheduler):
		state = event.args
		rospy.loginfo('RTH: state=%s, reason=%s', state['state'].name, state['reason'].name)

	@olympe.listen_event(format_progress(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.format_progress
	def on_format_progress(self, event, scheduler):
		format_progress = event.args
		rospy.loginfo('Formatting --> %s (%i%%)', format_progress['step'].name, format_progress['percentage'])

	@olympe.listen_event(user_storage_info(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.info
	def on_user_storage_info(self, event, scheduler):
		capacity = event.args['capacity']
		available_bytes = self.drone.drone.get_state(user_storage_monitor)['available_bytes']  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.monitor
		rospy.loginfo_once('Available space: %.1f/%.1fGB', available_bytes/(2**30), capacity/(2**30))

	""" 
	DEBUG  
	"""
	@olympe.listen_event(calibration_state(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.calibration_state
	def on_gimbal_calibration_state(self, event, scheduler):
		calibration_state = event.args
		rospy.logdebug('Gimbal calibration state: ' + calibration_state['state'].name)

	"""
	PUBLISHERS
	"""
	@olympe.listen_event(zoom_level(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.zoom_level
	def onZoomLevel(self, event, scheduler):
		self.drone.pub_zoom.publish(event.args['level'])

	@olympe.listen_event(NumberOfSatelliteChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_gps.html#olympe.messages.ardrone3.GPSState.NumberOfSatelliteChanged
	def onNumberOfSatelliteChanged(self, event, scheduler):
		self.drone.pub_gps_satellites.publish(event.args['numberOfSatellite'])

	@olympe.listen_event(AltitudeChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AltitudeChanged
	def onAltitudeChanged(self, event, scheduler):
		self.drone.pub_altitude_above_TO.publish(event.args['altitude'])

	@olympe.listen_event(AttitudeChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AttitudeChanged
	def onAttitudeChanged(self, event, scheduler):  # publishes at lower rate (5Hz) than 'pub_rpy' (30Hz) but has higher reaction time (approx. 100ms faster)
		attitude = event.args
		self.msg_attitude.header.stamp = rospy.Time.now()
		self.msg_attitude.header.frame_id = '/world'
		self.msg_attitude.vector.x = attitude['roll']*180/math.pi
		self.msg_attitude.vector.y = -attitude['pitch']*180/math.pi
		self.msg_attitude.vector.z = -attitude['yaw']*180/math.pi
		self.drone.pub_rpy_slow.publish(self.msg_attitude)

	@olympe.listen_event(GpsLocationChanged(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.GpsLocationChanged
	def onGpsLocationChanged(self, event, scheduler):
		gps_location = event.args
		self.msg_gps_location.header.stamp = rospy.Time.now()
		self.msg_gps_location.header.frame_id = '/world'
		self.msg_gps_location.status.status = \
			(self.msg_gps_location.status.STATUS_FIX if self.drone.gps_fixed else self.msg_gps_location.status.STATUS_NO_FIX) # https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatStatus.html
		if self.drone.model == "4k" or self.drone.model == "thermal":
			self.msg_gps_location.status.service = \
				self.msg_gps_location.status.SERVICE_GPS + \
				self.msg_gps_location.status.SERVICE_GLONASS;
		if self.drone.model == "usa" or self.drone.model == "ai":
			self.msg_gps_location.status.service = \
				self.msg_gps_location.status.SERVICE_GPS + \
				self.msg_gps_location.status.SERVICE_GLONASS + \
				self.msg_gps_location.status.SERVICE_GALILEO;
		self.msg_gps_location.latitude = (gps_location['latitude'] if gps_location['latitude'] != 500 else float('nan'))
		self.msg_gps_location.longitude = (gps_location['longitude'] if gps_location['longitude'] != 500 else float('nan'))
		self.msg_gps_location.altitude = (gps_location['altitude'] if gps_location['altitude'] != 500 else float('nan'))
		self.msg_gps_location.position_covariance[0] = \
			(math.sqrt(gps_location['latitude_accuracy']) if gps_location['latitude_accuracy'] > 0 else float('nan'))
		self.msg_gps_location.position_covariance[4] = \
			(math.sqrt(gps_location['longitude_accuracy']) if gps_location['longitude_accuracy'] > 0 else float('nan'))
		self.msg_gps_location.position_covariance[8] = \
			(math.sqrt(gps_location['altitude_accuracy']) if gps_location['altitude_accuracy'] > 0 else float('nan'))
		self.msg_gps_location.position_covariance_type = msg_gps_location.COVARIANCE_TYPE_DIAGONAL_KNOWN
		self.drone.pub_gps_location.publish(self.msg_gps_location)

	@olympe.listen_event(voltage(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.voltage
	def onBatteryVoltage(self, event, scheduler):
		self.drone.pub_battery_voltage.publish(event.args['voltage']/1000)

	@olympe.listen_event(target_trajectory(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.target_trajectory
	def on_target_trajectory(self, event, scheduler):
		trajectory = event.args
		rospy.loginfo('target_trajectory: ' + str(trajectory))
		self.msg_trajectory.header.stamp = rospy.Time.now()
		self.msg_trajectory.header.frame_id = '/world'
		self.msg_trajectory.latitude = trajectory['latitude']
		self.msg_trajectory.longitude = trajectory['longitude']
		self.msg_trajectory.altitude = trajectory['altitude']
		self.msg_trajectory.north_speed = trajectory['north_speed']
		self.msg_trajectory.east_speed = trajectory['east_speed']
		self.msg_trajectory.down_speed = trajectory['down_speed']
		self.drone.pub_target_trajectory.publish(self.msg_trajectory)

	@olympe.listen_event(gimbal_attitude(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.attitude
	def onGimbalAttitude(self, event, scheduler):
		gimbal = event.args
		self.msg_gimbal.header.stamp = rospy.Time.now()
		self.msg_gimbal.header.frame_id = '/world'
		self.msg_gimbal.vector.x = gimbal['roll_absolute']
		self.msg_gimbal.vector.y = -gimbal['pitch_absolute']
		self.msg_gimbal.vector.z = -gimbal['yaw_absolute']
		self.drone.pub_gimbal_absolute.publish(self.msg_gimbal)
		self.msg_gimbal.header.frame_id = 'body'
		self.msg_gimbal.vector.x = gimbal['roll_relative']
		self.msg_gimbal.vector.y = -gimbal['pitch_relative']
		self.msg_gimbal.vector.z = -gimbal['yaw_relative']
		self.drone.pub_gimbal_relative.publish(self.msg_gimbal)

	@olympe.listen_event(user_storage_monitor(_policy="wait"))  # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.monitor
	def on_user_storage_monitor(self, event, scheduler):
		self.drone.pub_media_available.publish(event.args['available_bytes'])

	""" 
	All other events
	"""
	@olympe.listen_event()
	def default(self, event, scheduler):
		pass
