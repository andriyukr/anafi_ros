#!/usr/bin/env python3

import numpy, math

def bound(value, value_min, value_max):
	return min(max(value, value_min), value_max)


def bound_percentage(value):
	return bound(value, -100, 100)

# https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434?permalink_comment_id=3850488#file-euler_from_quaternion-py
# Replace with: https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py#L1227
def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = {quaternion.x, quaternion.y, quaternion.z, quaternion.w}
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = numpy.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = numpy.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = numpy.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434?permalink_comment_id=3850488#file-quaternion_from_euler-py
# Replace with: https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py#L1238
def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    quaternion = [0] * 4  
    quaternion[0] = cy * cp * sr - sy * sp * cr
    quaternion[1] = sy * cp * sr + cy * sp * cr
    quaternion[2] = sy * cp * cr - cy * sp * sr
    quaternion[3] = cy * cp * cr + sy * sp * sr

    return quaternion

# https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py#L1445
def quaternion_multiply(quaternion1, quaternion0):
	"""
	Return multiplication of two quaternions.
	"""
	w0, x0, y0, z0 = quaternion0
	w1, x1, y1, z1 = quaternion1
	return numpy.array([
		-x1*x0 - y1*y0 - z1*z0 + w1*w0,
		x1*w0 + y1*z0 - z1*y0 + w1*x0,
		-x1*z0 + y1*w0 + z1*x0 + w1*y0,
		x1*y0 - y1*x0 + z1*w0 + w1*z0,
	], dtype=numpy.float64,	)

# https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py#L1466
def quaternion_conjugate(quaternion):
	"""
	Return conjugate of quaternion.
	"""
	return numpy.array([
		quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]
	], dtype=numpy.float64,	)

# https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py#L1480
def quaternion_inverse(quaternion):
	"""
	Return inverse of quaternion.
	"""
	return quaternion_conjugate(quaternion)/numpy.dot(quaternion, quaternion)

# https://answers.ros.org/question/196149/how-to-rotate-vector-by-quaternion-in-python/?answer=196155#post-id-196155
def rotate_vector(quaternion, vector):
	"""
	Return rotated vector by quaternion.
	"""
	v = [0.0, vector[0], vector[1], vector[2]]
	return quaternion_multiply(quaternion_multiply(quaternion, v), quaternion_conjugate(quaternion))[1:]

# 
def rotate_quaternion(quaternion1, quaternion2):
	"""
	Return rotated quaternion.
	"""
	return quaternion_multiply(quaternion1, quaternion2)
