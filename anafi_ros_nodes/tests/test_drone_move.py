#test/test_drone_move.py

import rclpy

import unittest
from termcolor import colored
import time
from timeit import default_timer as timer
import numpy as np
import math

import drone_utils

from anafi_ros_nodes.anafi_tester import AnafiTester
from anafi_ros_interfaces.msg import MoveByCommand, MoveToCommand, PilotingCommand
from anafi_ros_nodes.utils import euler_from_quaternion


class TestAnafiMove(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        namespace = '/anafi'
        self.test_node = AnafiTester(namespace=namespace)

        self.start_time_test = timer()
        self.delta_speed = 0.05
        self.delta_degree = 6.0
        self.delta_radians = 0.2
        self.delta_gps = 0.000005

    def tearDown(self):
        self.test_node.destroy_node()

    def test_attitude(self):
        print("\n***** ATTITUDE TOPIC TEST *****")

        # get current state
        drone_utils.init_drone_state(self.test_node)
        # set initial state to LANDED
        drone_utils.set_drone_initial_state(self.test_node, "LANDED")

        #get current attitude
        drone_utils.init_drone_attitude(self.test_node)
        roll, pitch, yaw = euler_from_quaternion(self.test_node.drone_attitude)

        roll_threshold = 5.0
        pitch_threshold = 5.0

        try:
            self.assertTrue(abs(math.degrees(roll)) < roll_threshold)
            self.assertTrue(abs(math.degrees(pitch)) < pitch_threshold)

            print(colored("Test passed: duration {:.3f}s, drone_state: {}, attitude:roll {} pitch {} yaw {})".format(
                timer() - self.start_time_test, self.test_node.drone_state,
                roll, pitch, yaw),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration {:.3f}s, drone_state: {}, attitude:r{:.3f} p{:.3f} y{:.3f})".format(
                    timer() - self.start_time_test, self.test_node.drone_state,
                    roll, pitch, yaw),
                    'red'))
            raise e

    def test_piloting_command(self):
        print("\n***** PILOTING COMMAND TEST *****")
        #set to HOVERING

        drone_utils.init_drone_state(self.test_node)
        # Initial state must be TAKING OFF, HOVERING or FLYING
        # HOVERING gets ignored by setDroneInitialState
        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        #send piloting command
        drone_command_msg = PilotingCommand()
        drone_command_msg.roll = 0.8
        drone_command_msg.pitch = 0.8
        drone_command_msg.yaw = 0.0
        drone_command_msg.gaz = 0.0


        # get current rpy
        drone_utils.init_drone_rpy(self.test_node)
        roll = self.test_node.drone_rpy.vector.x
        pitch = self.test_node.drone_rpy.vector.y

        is_correct_degree = False
        for _ in range(100):
            self.test_node.pub_drone_command.publish(drone_command_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            if ( (abs(pitch - drone_command_msg.pitch) < self.delta_radians) and
               (abs(roll - drone_command_msg.roll) < self.delta_radians)):
                is_correct_degree = True
                break
            drone_utils.check_timeout(self.test_node, self.start_time_test)
            roll = self.test_node.drone_rpy.vector.x
            pitch = self.test_node.drone_rpy.vector.y
            time.sleep(0.2)

        #check if is_flying
        is_flying = False
        for _ in range(100):
            self.test_node.pub_drone_command.publish(drone_command_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            if self.test_node.drone_state == "FLYING":
                is_flying = True
                break
            drone_utils.check_timeout(self.test_node, self.start_time_test)
            time.sleep(0.1)

        try:
            self.assertTrue(is_flying)
            self.assertTrue(is_correct_degree)
            print(colored("Test passed: duration {:.3f}s, drone_state: {})".format(
                timer() - self.start_time_test, self.test_node.drone_state),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration {:.3f}s, drone_state: {})".format(
                    timer() - self.start_time_test, self.test_node.drone_state),
                    'red'))
            raise e

    def test_emergency(self):
        print("\n***** EMERGENCY TEST *****")

        drone_utils.init_drone_state(self.test_node)
        # Initial state must be TAKING OFF, HOVERING or FLYING
        # HOVERING gets ignored by setDroneInitialState
        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        starting_state = self.test_node.drone_state

        self.test_node.send_emergency()
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            if self.test_node.drone_state == "EMERGENCY":
                break
            time.sleep(0.5)

        final_state = self.test_node.drone_state

        try:
            self.assertEqual(final_state, "EMERGENCY")

            print(colored("Test passed: starting state {}, final state {}".format(
                starting_state, final_state), 'green'))
        except AssertionError as e:
            print(colored("Test Failed: starting state {}, final state {}".format(
                starting_state, final_state), 'red'))
            raise e

    def test_altitude(self):
        print("\n***** ALTITUDE TOPIC TEST *****")

        # get current state
        drone_utils.init_drone_state(self.test_node)
        # set initial state to LANDED
        drone_utils.set_drone_initial_state(self.test_node, "LANDED")

        #get altitude
        altitude_landed = drone_utils.init_drone_altitude(self.test_node)

        #take off
        self.test_node.drone_takeoff()
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            if self.test_node.drone_state == "HOVERING":
                break
            time.sleep(1)

        #get altitude after take off
        altitude_hovering = self.test_node.drone_altitude

        try:
            self.assertAlmostEqual(altitude_landed, 0.0)
            self.assertGreater(altitude_hovering, 0.0)

            print(colored("Test passed: altitude landed {:.3f}, altitude after takeoff {:.3f}".format(
                altitude_landed, altitude_hovering), 'green'))
        except AssertionError as e:
            print(colored("Test Failed: altitude landed {:.3f}, altitude after takeoff {:.3f}".format(
                altitude_landed, altitude_hovering), 'red'))
            raise e

    def test_drone_rpy(self):
        print("\n***** RPY TOPIC TEST *****")

        # get current state
        drone_utils.init_drone_state(self.test_node)
        # set initial state to LANDED
        drone_utils.set_drone_initial_state(self.test_node, "LANDED")

        #get current rpy
        drone_utils.init_drone_rpy(self.test_node)
        roll = self.test_node.drone_rpy.vector.x
        pitch = self.test_node.drone_rpy.vector.y
        yaw = self.test_node.drone_rpy.vector.z

        roll_threshold = 6.0
        pitch_threshold = 6.0

        try:
            self.assertLess(abs(math.degrees(roll)), roll_threshold)
            self.assertLess(abs(math.degrees(pitch)), pitch_threshold)

            print(colored("Test passed: duration {:.3f}s, drone_state: {}, attitude:r{:.3f} p{:.3f} y{:.3f})".format(
                timer() - self.start_time_test, self.test_node.drone_state,
                roll, pitch, yaw),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration {:.3f}s, drone_state: {}, attitude:r{:.3f} p{:.3f} y{:.3f})".format(
                    timer() - self.start_time_test, self.test_node.drone_state,
                    roll, pitch, yaw),
                    'red'))
            raise e

    def test_drone_slow_rpy(self):
        print("\n***** SLOW RPY TOPIC TEST *****")
        #published at lower rate (5Hz) than 'pub_rpy' (30Hz) but has higher reaction time (approx. 100ms faster)
        # get current state
        drone_utils.init_drone_state(self.test_node)
        # set initial state to LANDED
        drone_utils.set_drone_initial_state(self.test_node, "LANDED")

        # get current rpy slow
        drone_utils.init_drone_rpy_slow(self.test_node)
        roll = self.test_node.drone_rpy_slow.vector.x
        pitch = self.test_node.drone_rpy_slow.vector.y
        yaw = self.test_node.drone_rpy_slow.vector.z

        roll_threshold = 6.0
        pitch_threshold = 6.0

        try:
            self.assertLess(abs(math.degrees(roll)), roll_threshold)
            self.assertLess(abs(math.degrees(pitch)), pitch_threshold)

            print(colored("Test passed: duration {:.3f}s, drone_state: {}, attitude:r{} p{} y{})".format(
                timer() - self.start_time_test, self.test_node.drone_state,
                roll, pitch, yaw),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration {:.3f}s, drone_state: {}, attitude:r{:.3f} p{:.3f} y{:.3f})".format(
                    timer() - self.start_time_test, self.test_node.drone_state,
                    roll, pitch, yaw),
                    'red'))
            raise e

    def test_drone_speed(self):
        print("\n***** SPEED TOPIC TEST *****")

        # get current state
        drone_utils.init_drone_state(self.test_node)
        # set initial state to LANDED
        drone_utils.set_drone_initial_state(self.test_node, "LANDED")

        drone_utils.init_drone_speed(self.test_node)

        speed_landed = self.test_node.drone_speed.vector
        #init speed_max with 0
        speed_max = speed_landed
        max_speed_vertical_reached = 0.0

        #send drone command with positive gaz
        drone_command_msg = PilotingCommand()
        drone_command_msg.roll = 0.0
        drone_command_msg.pitch = 0.0
        drone_command_msg.yaw = 0.0
        drone_command_msg.gaz = 50.0

        drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        self.test_node.pub_drone_command.publish(drone_command_msg)
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            if self.test_node.drone_speed.vector != speed_landed:
                while abs(self.test_node.drone_speed.vector.z) > self.delta_speed:
                    rclpy.spin_once(self.test_node, timeout_sec=0.5)
                    if max_speed_vertical_reached < self.test_node.drone_speed.vector.z:
                        max_speed_vertical_reached = self.test_node.drone_speed.vector.z
                        speed_max = self.test_node.drone_speed.vector
                break
            time.sleep(0.1)

        try:
            self.assertAlmostEqual(speed_landed.x, 0.0)
            self.assertAlmostEqual(speed_landed.y, 0.0)
            self.assertAlmostEqual(speed_landed.z, 0.0)
            self.assertAlmostEqual(speed_max.x, 0.0, delta=self.delta_speed)
            self.assertAlmostEqual(speed_max.y, 0.0, delta=self.delta_speed)
            self.assertGreater(max_speed_vertical_reached, speed_landed.z)

            print(colored("Test passed: duration {:.3f}s, drone_state: {}, maximum vertical speed:{:.3f})".format(
                timer() - self.start_time_test, self.test_node.drone_state,
                max_speed_vertical_reached),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration {:.3f}s, drone_state: {}, maximum vertical speed:{:.3f})".format(
                    timer() - self.start_time_test, self.test_node.drone_state,
                    max_speed_vertical_reached),
                    'red'))
            raise e

    def test_takeoff(self):
        print("\n***** TAKEOFF TEST *****")

        #get current state
        drone_utils.init_drone_state(self.test_node)
        #set initial state to LANDED
        drone_utils.set_drone_initial_state(self.test_node, "LANDED")

        #get starting altitude
        to_altitude = drone_utils.init_drone_altitude(self.test_node)
        print("\n starting altitude: {}".format(to_altitude))

        #start test
        self.test_node.drone_takeoff()
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            if self.test_node.drone_state == "HOVERING" and (self.test_node.drone_altitude - to_altitude > 0.0):
                break
            time.sleep(1)

        try:
            self.assertEqual(self.test_node.drone_state, 'HOVERING')
            if to_altitude != np.nan:
                self.assertGreater(self.test_node.drone_altitude, to_altitude)
            print(colored("Test passed: duration takeoff: {:.3f}s, drone_state: {}, delta altitude: {:.3f})".format(
                timer() - self.start_time_test, self.test_node.drone_state,
                self.test_node.drone_altitude - to_altitude),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration takeoff: {:.3f}s, drone_state: {}, delta altitude: {:.3f})".format(
                    timer() - self.start_time_test, self.test_node.drone_state,
                    self.test_node.drone_altitude - to_altitude),
                    'red'))
            raise e

    def test_land(self):
        print("\n***** LAND TEST *****")

        drone_utils.init_drone_state(self.test_node)
        # Initial state must be TAKING OFF, HOVERING or FLYING
        # HOVERING gets ignored by setDroneInitialState
        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        # get starting altitude
        to_altitude = drone_utils.init_drone_altitude(self.test_node)
        print("\n starting altitude: {}".format(to_altitude))

        # start test
        self.test_node.drone_land()
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            if self.test_node.drone_state == "LANDED" and (self.test_node.drone_altitude - to_altitude < 0.0):
                break
            time.sleep(1)

        try:
            self.assertEqual(self.test_node.drone_state, 'LANDED')
            if to_altitude != np.nan:
                self.assertLess(self.test_node.drone_altitude, to_altitude)
            print(colored("Test passed: duration landing: {:.3f}s, drone_state: {}, delta altitude: {:.3f})".format(
                timer() - self.start_time_test, self.test_node.drone_state,
                self.test_node.drone_altitude - to_altitude),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration landing: {:.3f}s, drone_state: {}, delta altitude: {:.3f})".format(
                    timer() - self.start_time_test, self.test_node.drone_state,
                    self.test_node.drone_altitude - to_altitude),
                    'red'))
            raise e

    def test_moveby(self):
        print("\n***** MOVE BY TEST *****")

        drone_utils.init_drone_state(self.test_node)
        # Initial state must be TAKING OFF, HOVERING or FLYING
        # HOVERING gets ignored by setDroneInitialState
        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        #set moveby command
        msg_moveby_command = MoveByCommand()
        msg_moveby_command.dx = 1.0
        msg_moveby_command.dy = 0.0
        msg_moveby_command.dz = -1.0
        msg_moveby_command.dyaw = 0.0

        # start test
        self.test_node.pub_drone_moveby.publish(msg_moveby_command)
        """while not self.is_moved_by(to_location, msg_moveby_command):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            drone_utils.check_timeout(self.test_node, self.start_time_test)
            time.sleep(0.1)"""

        distance_to_move = (msg_moveby_command.dx, msg_moveby_command.dy, msg_moveby_command.dz)
        result = drone_utils.is_moved_by(self.test_node, distance_to_move, 0.01)

        try:
            #self.assertTrue(self.is_moved_by(to_location, msg_moveby_command))
            self.assertTrue(result)
            print(colored("Test passed: duration moving: {:.3f}s, drone_state: {})".format(
                timer() - self.start_time_test, self.test_node.drone_state),
                'green'))

        except AssertionError as e:
            print(colored("Assertion Failed: duration moving: {:.3f}s, drone_state: {})".format(
                timer() - self.start_time_test, self.test_node.drone_state),
                'red'))
            raise e

    def test_moveto(self):
        print("\n***** MOVE TO TEST *****")

        drone_utils.init_drone_state(self.test_node)

        # Initial state must be TAKING OFF, HOVERING or FLYING
        # HOVERING gets ignored by setDroneInitialState
        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        #set target altitude
        to_altitude = 2.8 #meters
        #set displacement
        displacement = (3.0, 3.0, 0.0) #north, east, altitude (meters)
        #convert delta meters to delta GPS
        delta_lat, delta_lon = drone_utils.convert_delta_meters_to_gps(self.test_node, displacement[0], displacement[1])

        #get drone location
        drone_utils.init_drone_gps_position(self.test_node)
        to_location = self.test_node.drone_gps_location
        #calculate new position
        new_lat = to_location[0] + delta_lat
        new_lon = to_location[1] + delta_lon

        #set moveto command
        msg_moveto_command = MoveToCommand()
        msg_moveto_command.latitude = new_lat
        msg_moveto_command.longitude = new_lon
        msg_moveto_command.altitude = to_altitude + displacement[2]
        msg_moveto_command.heading = 0.0  #used for orientation mode 2 and 3
        msg_moveto_command.orientation_mode = 0  #no change on orientation

        # start test
        self.test_node.pub_drone_moveto.publish(msg_moveto_command)
        is_moved_to = False
        curr_location = self.test_node.drone_gps_location
        for _ in range(1000):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            curr_location = self.test_node.drone_gps_location
            if (abs(curr_location[0] - new_lat) < self.delta_gps) and (abs(curr_location[1] - new_lon) < self.delta_gps):
                is_moved_to = True
                break
            drone_utils.check_timeout(self.test_node, self.start_time_test)
            time.sleep(0.1)

        try:
            self.assertTrue(is_moved_to)
            print(colored("Test passed: duration moving: {:.3f}s, current position: {})".format(
                timer() - self.start_time_test, curr_location),
                'green'))

        except AssertionError as e:
            print(colored("Assertion Failed: duration moving: {:.3f}s, current position: {})".format(
                timer() - self.start_time_test, curr_location),
                'red'))
            raise e

    def test_calibrate(self):
        print("\n***** DRONE CALIBRATE TEST *****")

        future = self.test_node.calibrate_drone()
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        try:
            self.assertNotEqual(future.result(), None)
            self.assertTrue(future.result().success)
            print(colored("Test passed: duration calibration of drone: {:.3f}s with message: {}".format(
                timer() - self.start_time_test, future.result().message),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration calibration of drone: {:.3f}s with message: {}".format(
                    timer() - self.start_time_test, future.result().message),
                    'red'))
            raise e

    def test_halt(self):
        print("\n***** DRONE HALT TEST *****")

        drone_utils.init_drone_state(self.test_node)
        # Initial state must be TAKING OFF, HOVERING or FLYING
        # HOVERING gets ignored by setDroneInitialState
        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        # set moveby command
        msg_moveby_command = MoveByCommand()
        msg_moveby_command.dx = 1.0
        msg_moveby_command.dy = 1.0
        msg_moveby_command.dz = 1.0
        msg_moveby_command.dyaw = 0.0

        #make the drone start moving
        self.test_node.pub_drone_moveby.publish(msg_moveby_command)
        time.sleep(5)
        #halt the drone
        future = self.test_node.halt_drone()
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        try:
            self.assertNotEqual(future.result(), None)
            self.assertTrue(future.result().success)
            #TODO add check for hovering state??
            print(colored("Test passed: duration halt of drone: {:.3f}s".format(
                timer() - self.start_time_test),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration calibration of drone: {:.3f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_steady(self):
        #TODO fix anafi node
        print("\n***** DRONE STEADY TEST *****")

        drone_utils.init_attribute(self.test_node, "drone_steady")

        try:
            self.assertTrue(isinstance(self.test_node.drone_steady, bool))

            print(colored("Test passed: drone steady {}".format("Yes" if self.test_node.drone_steady else "No"), 'green'))
        except AssertionError as e:
            print(colored("Assertion failed: drone steady {}".format("Yes" if self.test_node.drone_steady else "No"),
                          'red'))
            raise e

    def test_get_max_altitude(self):
        print("\n***** DRONE GET MAX ALTITUDE TEST *****")

        max_altitude = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_altitude,'double')

        try:
            self.assertNotEqual(max_altitude, np.nan)
            self.assertGreaterEqual(max_altitude, 0.5)
            self.assertLessEqual(max_altitude, 4000)
            print(colored("Test passed: duration get of max altitude: {:.3f}, max_altitude: {:.3f}m".format(
                timer() - self.start_time_test, max_altitude),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration get of max altitude: {:.3f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_set_max_altitude(self):
        print("\n***** DRONE SET MAX ALTITUDE TEST *****")
        drone_utils.init_drone_altitude(self.test_node)

        starting_max_altitude = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_altitude,'double')
        #set new max altitude
        new_max_altitude = drone_utils.increment_within_range(starting_max_altitude, 10, 0.5, 4000)
        future = self.test_node.set_max_altitude(new_max_altitude)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)
        #get updated altitude
        updated_max_altitude = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_altitude,'double')

        #try to fly above max altitude
        #set max altitude to small value
        small_max_altitude = 5.0
        future = self.test_node.set_max_altitude(small_max_altitude)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)
        #start from landed state
        drone_utils.init_drone_state(self.test_node)
        # set initial state to LANDED
        drone_utils.set_drone_initial_state(self.test_node, "LANDED")
        drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        # send drone command with positive gaz
        drone_command_msg = PilotingCommand()
        drone_command_msg.roll = 0.0
        drone_command_msg.pitch = 0.0
        drone_command_msg.yaw = 0.0
        drone_command_msg.gaz = 80.0

        delta_altitude = 1.0
        exceeded = False
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)
            #print(self.test_node.drone_altitude)
            self.test_node.pub_drone_command.publish(drone_command_msg)
            fence_altitude = small_max_altitude + delta_altitude
            if self.test_node.drone_altitude > fence_altitude:
                exceeded = True
            time.sleep(1)

        try:
            self.assertEqual(updated_max_altitude, new_max_altitude)
            self.assertFalse(exceeded)
            print(colored("Test passed: duration set of max altitude : {:.3f}s, current altitude: {:.3f}".format(
                timer() - self.start_time_test, self.test_node.drone_altitude),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration set of max altitude: {:.3f}s, current altitude: {:.3f}".format(
                    timer() - self.start_time_test, self.test_node.drone_altitude),
                    'red'))
            raise e

    def test_get_max_distance(self):
        print("\n***** DRONE GET MAX DISTANCE TEST *****")

        max_distance = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_distance,'double')

        try:
            self.assertNotEqual(max_distance, np.nan)
            self.assertGreaterEqual(max_distance, 10)
            self.assertLessEqual(max_distance, 4000)
            print(colored("Test passed: duration get of max distance: {:.3f}, max_distance: {:.3f}m".format(
                timer() - self.start_time_test, max_distance),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration get of max distance: {:.3f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_set_max_distance(self):
        print("\n***** DRONE SET MAX DISTANCE TEST *****")
        starting_max_distance = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_distance,'double')
        # set new max distance
        new_max_distance = drone_utils.increment_within_range(starting_max_distance, 10, 10, 4000)
        print(new_max_distance)
        future = self.test_node.set_max_distance(new_max_distance)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        # get updated distance
        updated_max_distance = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_distance,'double')
        try:
            self.assertEqual(updated_max_distance, new_max_distance)
            print(colored(f"Max distance successfully updated to {updated_max_distance:.2f} meters",
                          'green'))
        except AssertionError:
            print(colored(
                f"Test failed: Expected max distance {new_max_distance:.2f} but got {updated_max_distance:.2f}", 'red'
            ))
            raise

        #try to fly over geofence
        future = self.test_node.set_max_pitch_roll(40.0)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)
        #set max distance to min
        future = self.test_node.set_max_distance(10.0)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)
        updated_max_distance = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_distance,'double')
        print(updated_max_distance)
        # get current state
        drone_utils.init_drone_state(self.test_node)

        # set initial state to LANDED to reset takeoff point (distance refers to takeoff point)
        drone_utils.set_drone_initial_state(self.test_node, "LANDED")

        #start flying
        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        drone_command_msg = PilotingCommand()
        drone_command_msg.roll = 40.0
        drone_command_msg.pitch = 40.0
        drone_command_msg.yaw = 0.0
        drone_command_msg.gaz = 0.0

        distance_to_move = (15.0, 15.0, 0.0)
        exceeded = drone_utils.is_moved_by(self.test_node, distance_to_move, 0.01, max_duration=30.0, piloting_msg=drone_command_msg)

        try:
            self.assertFalse(exceeded)
            print(colored("Test passed: duration set of max distance : {:.3f}s, new max distance: {:.3f}".format(
                timer() - self.start_time_test, updated_max_distance),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration set of max distance: {:.3f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_get_max_pitch_roll(self):
        print("\n***** DRONE GET MAX PITCH AND ROLL DEGREE TEST *****")

        max_rp = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_pitch_roll,'double')

        try:
            self.assertNotEqual(max_rp, np.nan)
            self.assertGreaterEqual(max_rp, 1)
            self.assertLessEqual(max_rp, 40.0)
            print(colored("Test passed: duration get of max pitch and roll: {:.3f}, max pitch and roll: {:.3f}m".format(
                timer() - self.start_time_test, max_rp),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration get of max pitch and  roll: {:.3f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_set_max_pitch_roll(self):
        print("\n***** DRONE SET MAX PITCH AND ROLL TEST *****")
        starting_max_pitch_roll = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_pitch_roll,'double')
        # set new max distance
        new_max_pitch_roll = drone_utils.increment_within_range(starting_max_pitch_roll, 10.0, 1.0, 40.0)
        print(new_max_pitch_roll)
        future = self.test_node.set_max_pitch_roll(new_max_pitch_roll)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        # get updated tilt
        updated_max_pitch_roll = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_pitch_roll,'double')

        #test that max tilt is not exceeded with max possible piloting command degree
        drone_utils.init_drone_state(self.test_node)
        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        drone_command_msg = PilotingCommand()
        drone_command_msg.roll = 40.0
        drone_command_msg.pitch = 40.0
        drone_command_msg.yaw = 0.0
        drone_command_msg.gaz = 0.0
        self.test_node.pub_drone_command.publish(drone_command_msg)
        max_tilt_reached_piloting = drone_utils.store_max_tilt(self.test_node, 60)
        #print(max_tilt_reached_piloting)

        try:
            self.assertEqual(updated_max_pitch_roll, new_max_pitch_roll)
            self.assertLess(max_tilt_reached_piloting, new_max_pitch_roll)
            print(colored("Test passed: duration set of max pitch and roll : {:.3f}s, new max pitch and roll: {:.3f}, "
                          "max tilt reached piloting: {:.3f}".format(
                timer() - self.start_time_test, updated_max_pitch_roll, max_tilt_reached_piloting),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration set  of max pitch and roll: {:.3f}s, max pitch and roll: {:.3f}".format(
                    timer() - self.start_time_test, updated_max_pitch_roll),
                    'red'))
            raise e

    def test_get_max_horizontal_speed(self):
        print("\n***** DRONE GET MAX HORIZONTAL SPEED TEST *****")

        max_horiz_speed = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_horizontal_speed,'double')

        try:
            self.assertNotEqual(max_horiz_speed, np.nan)
            self.assertGreaterEqual(max_horiz_speed, 0.1)
            self.assertLessEqual(max_horiz_speed, 15.0)
            print(colored("Test passed: duration get of max horizontal speed: {:.3f}, max horizontal speed: {:.3f}m".format(
                timer() - self.start_time_test, max_horiz_speed),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration get of max horizontal speed: {:.3f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_set_max_horizontal_speed(self):
        print("\n***** DRONE SET MAX HORIZONTAL SPEED TEST *****")
        drone_utils.init_drone_state(self.test_node)

        starting_max_horizontal_speed = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_horizontal_speed,'double')
        # set new max distance
        new_max_horizontal_speed = drone_utils.increment_within_range(starting_max_horizontal_speed, 1.0, 0.1, 15.0)
        future = self.test_node.set_max_horizontal_speed(new_max_horizontal_speed)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        # get updated distance
        updated_max_horizontal_speed = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_horizontal_speed,'double')

        #test if horizontal speed limited is not exceeded
        #first change max tilte angle to max possible
        max_tilt = 40.0
        future = self.test_node.set_max_pitch_roll(max_tilt)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        #send piloting command with max tilt on x and y-axis and store max horizontal speed
        drone_command_msg = PilotingCommand()
        drone_command_msg.roll = 40.0
        drone_command_msg.pitch = 40.0
        drone_command_msg.yaw = 0.0
        drone_command_msg.gaz = 0.0
        self.test_node.pub_drone_command.publish(drone_command_msg)
        max_h_speed_reached_piloting = drone_utils.store_max_horizontal_speed(self.test_node, 60)
        print(max_h_speed_reached_piloting)

        #send moveby command and store max horizontal speed
        msg_moveby_command = MoveByCommand()
        msg_moveby_command.dx = 3.0
        msg_moveby_command.dy = 3.0
        msg_moveby_command.dz = 0.0
        msg_moveby_command.dyaw = 0.0
        self.test_node.pub_drone_moveby.publish(msg_moveby_command)
        max_h_speed_reached_moveby = drone_utils.store_max_horizontal_speed(self.test_node, 60)
        print(max_h_speed_reached_moveby)

        try:
            self.assertEqual(updated_max_horizontal_speed, new_max_horizontal_speed)
            self.assertLess(max_h_speed_reached_piloting, updated_max_horizontal_speed)
            self.assertLess(max_h_speed_reached_moveby, updated_max_horizontal_speed)
            print(colored("Test passed: duration set of max horizontal speed : {:.3f}s, new max horizontal speed: {:.3f}".format(
                timer() - self.start_time_test, updated_max_horizontal_speed),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration set of max horizontal speed: {:.3f}s, new max horizontal speed: {:.3f}".format(
                    timer() - self.start_time_test, updated_max_horizontal_speed),
                    'red'))
            raise e

    def test_get_max_pitch_roll_rate(self):
        print("\n***** DRONE GET MAX PITCH AND ROLL RATE TEST *****")

        max_rp_rate = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_pitch_roll_rate,'double')

        try:
            self.assertNotEqual(max_rp_rate, np.nan)
            self.assertGreaterEqual(max_rp_rate, 40)
            self.assertLessEqual(max_rp_rate, 300.0)
            print(colored("Test passed: duration get of max pitch and roll: {:.3f}, max pitch and roll rate: {:.0f}d/s".format(
                timer() - self.start_time_test, max_rp_rate),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration get of max pitch and  roll rate: {:.0f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_set_max_pitch_roll_rate(self):
        print("\n***** DRONE SET MAX PITCH AND ROLL RATE TEST *****")
        starting_max_pitch_roll_rate = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_pitch_roll_rate,'double')
        # set new max distance
        new_max_pitch_roll_rate = drone_utils.increment_within_range(starting_max_pitch_roll_rate, 40.0, 40.0, 300.0)
        future = self.test_node.set_max_pitch_roll_rate(new_max_pitch_roll_rate)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        # get updated rate
        updated_max_pitch_roll_rate = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_pitch_roll_rate,'double')

        try:
            self.assertEqual(updated_max_pitch_roll_rate, new_max_pitch_roll_rate)
            print(colored("Test passed: duration set of max pitch and roll rate : {:.3f}s, new max pitch and roll rate: {:.0f}d/s".format(
                timer() - self.start_time_test, updated_max_pitch_roll_rate),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration set  of max pitch and roll rate: {:.3f}s, max pitch and roll rate: {:.0f}".format(
                    timer() - self.start_time_test, updated_max_pitch_roll_rate),
                    'red'))
            raise e

    def test_get_max_yaw_rate(self):
        print("\n***** DRONE GET MAX YAW RATE TEST *****")

        max_yaw_rate = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_yaw_rate,'double')

        try:
            self.assertNotEqual(max_yaw_rate, np.nan)
            self.assertGreaterEqual(max_yaw_rate, 3.0)
            self.assertLessEqual(max_yaw_rate, 200.0)
            print(colored("Test passed: duration get of max yaw rate: {:.3f}, max yaw rate: {:.0f}d/s".format(
                timer() - self.start_time_test, max_yaw_rate),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration get of max yaw rate: {:.0f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_set_max_yaw_rate(self):
        print("\n***** DRONE SET MAX YAW RATE TEST *****")
        starting_max_yaw_rate = drone_utils.get_drone_parameter(self.test_node,
                                                                self.test_node.request_max_yaw_rate,'double')
        # set new max distance
        new_max_yaw_rate = drone_utils.increment_within_range(starting_max_yaw_rate, 40.0, 3.0, 200.0)
        future = self.test_node.set_max_yaw_rate(new_max_yaw_rate)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        # get updated rate
        updated_max_yaw_rate = drone_utils.get_drone_parameter(self.test_node,
                                                               self.test_node.request_max_yaw_rate,'double')

        try:
            self.assertEqual(updated_max_yaw_rate, new_max_yaw_rate)
            print(colored(
                "Test passed: duration set of max yaw rate : {:.3f}s, new max yaw rate: {:.0f}d/s".format(
                    timer() - self.start_time_test, updated_max_yaw_rate),
                'green'))

        except AssertionError as e:
            print(
                colored(
                    "Assertion Failed: duration set  of max yaw rate: {:.3f}s, max pitch and roll rate: {:.0f}".format(
                        timer() - self.start_time_test, updated_max_yaw_rate),
                    'red'))
            raise e

    def test_get_max_vertical_speed(self):
        print("\n***** DRONE GET MAX VERTICAL SPEED TEST *****")

        max_vertical_speed = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_vertical_speed,'double')

        try:
            self.assertNotEqual(max_vertical_speed, np.nan)
            self.assertGreaterEqual(max_vertical_speed, 0.1)
            self.assertLessEqual(max_vertical_speed, 4.0)
            print(colored(
                "Test passed: duration get of max pitch and roll: {:.3f}, max vertical speed: {:.0f}m/s".format(
                    timer() - self.start_time_test, max_vertical_speed),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration get of max vertical speed: {:.0f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_set_max_vertical_speed(self):
        print("\n***** DRONE SET MAX VERTICAL SPEED TEST *****")
        drone_utils.init_drone_state(self.test_node)

        starting_max_vertical_speed = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_vertical_speed,'double')
        # set new max distance
        new_max_vertical_speed = drone_utils.increment_within_range(starting_max_vertical_speed, 1, 0.1, 4.0)
        future = self.test_node.set_max_vertical_speed(new_max_vertical_speed)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        # get updated distance
        updated_max_vertical_speed = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_max_vertical_speed,'double')

        #test if vertical speed limited is not exceeded

        if self.test_node.drone_state != "TAKINGOFF" and self.test_node.drone_state != "FLYING":
            drone_utils.set_drone_initial_state(self.test_node, "HOVERING")

        #send piloting command with max gaz
        drone_command_msg = PilotingCommand()
        drone_command_msg.roll = 0.0
        drone_command_msg.pitch = 0.0
        drone_command_msg.yaw = 0.0
        drone_command_msg.gaz = 100.0
        self.test_node.pub_drone_command.publish(drone_command_msg)
        max_v_speed_reached_piloting = drone_utils.store_max_vertical_speed(self.test_node, 60)
        print(max_v_speed_reached_piloting)

        #send moveby command and store max vertical speed
        msg_moveby_command = MoveByCommand()
        msg_moveby_command.dx = 0.0
        msg_moveby_command.dy = 0.0
        msg_moveby_command.dz = 3.0
        msg_moveby_command.dyaw = 0.0
        self.test_node.pub_drone_moveby.publish(msg_moveby_command)
        max_v_speed_reached_moveby = drone_utils.store_max_vertical_speed(self.test_node, 60)
        print(max_v_speed_reached_moveby)

        try:
            self.assertEqual(updated_max_vertical_speed, new_max_vertical_speed)
            self.assertLess(max_v_speed_reached_piloting, updated_max_vertical_speed)
            self.assertLess(max_v_speed_reached_moveby, updated_max_vertical_speed)
            print(colored("Test passed: duration set of max vertical speed : {:.3f}s, new max vertical speed: {:.3f}".format(
                timer() - self.start_time_test, updated_max_vertical_speed),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration set of max vertical: {:.3f}s, new max vertical speed: {:.3f}".format(
                    timer() - self.start_time_test, updated_max_vertical_speed),
                    'red'))
            raise e

    def test_get_offboard(self):
        print("\n***** DRONE GET IS OFFBOARD TEST *****")

        is_offboard = drone_utils.get_drone_parameter(self.test_node, self.test_node.request_offboard,'bool')

        try:
            self.assertNotEqual(is_offboard, np.nan)
            self.assertTrue(isinstance(is_offboard, bool))
            print(colored(
                "Test passed: duration get of offboard information: {:.3f}, is offboard: {}".format(
                    timer() - self.start_time_test, is_offboard),
                'green'))

        except AssertionError as e:
            print(
                colored("Assertion Failed: duration get of offboard information {:.0f}s".format(
                    timer() - self.start_time_test),
                    'red'))
            raise e

    def test_set_offboard(self):
        print("\n***** DRONE SET OFFBOARD TEST *****")

        starting_offboard_state = drone_utils.get_drone_parameter(self.test_node,
                                                                  self.test_node.request_offboard,'bool')
        #Switch offboard state
        new_offboard_state = not starting_offboard_state
        future = self.test_node.set_offboard(new_offboard_state)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        # get updated offboard state
        updated_offboard_state = drone_utils.get_drone_parameter(self.test_node,
                                                                 self.test_node.request_offboard,'bool')

        try:
            if drone_utils.is_simulated(self.test_node):
                self.assertEqual(updated_offboard_state, True)
            else:
                self.assertEqual(updated_offboard_state, new_offboard_state)
            print(colored(
                "Test passed: duration set of offboard state : {:.3f}s, new offboard state: {}".format(
                    timer() - self.start_time_test, updated_offboard_state),
                'green'))

        except AssertionError as e:
            print(
                colored(
                    "Assertion Failed: duration set  of offboard state: {:.3f}s, offboard state: {}".format(
                        timer() - self.start_time_test, updated_offboard_state),
                    'red'))
            raise e

    def test_set_offboard_parameter(self):
        print("\n***** DRONE SET OFFBOARD PARAMETER TEST *****")

        starting_offboard_state = drone_utils.get_drone_parameter(self.test_node,
                                                                  self.test_node.request_offboard,'bool')
        # Switch offboard state parameter
        new_offboard_state = not starting_offboard_state
        future = self.test_node.set_offboard_parameter(new_offboard_state)
        drone_utils.spin_until_done(self.test_node, future, self.start_time_test)

        # get updated offboard state
        updated_offboard_state = drone_utils.get_drone_parameter(self.test_node,
                                                                 self.test_node.request_offboard, 'bool')

        try:
            if drone_utils.is_simulated(self.test_node):
                self.assertEqual(updated_offboard_state, True)
            else:
                self.assertEqual(updated_offboard_state, new_offboard_state)
            print(colored(
                "Test passed: duration set of offboard state : {:.3f}s, new offboard state: {}".format(
                    timer() - self.start_time_test, updated_offboard_state),
                'green'))

        except AssertionError as e:
            print(
                colored(
                    "Assertion Failed: duration set  of offboard state: {:.3f}s, offboard state: {}".format(
                        timer() - self.start_time_test, updated_offboard_state),
                    'red'))
            raise e
