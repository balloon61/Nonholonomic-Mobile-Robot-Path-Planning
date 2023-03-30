#!/usr/bin/env python

import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
   
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qx, qy, qz, qw]

def get_rotation (orientation_q):
    """
    Input: ROS geometry msgs orientation 
    Output: roll, pitch, yaw
    """
    roll, pitch, yaw =  euler_from_quaternion ([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    return roll, pitch, yaw

def polar(x: float, y: float):
    """
    Return the polar coordinates (r, theta) of the point (x, y).
    """
    r = math.sqrt(x*x + y*y)
    theta = math.atan2(y, x)
    return r, theta


def mod2pi(var:float) -> float:
    
	value = var % (2 * math.pi)
	if(value < -math.pi):
		value += 2 * math.pi
	if(value >= math.pi):
		value -= 2 * math.pi
	return value

def tau_Omega(phi, xi, eta, u, v):

    delta = mod2pi(u - v)
    A = math.sin(u) - math.sin(delta)
    B = math.cos(u) - math.cos(delta) - 1
    tau = 0
    if 2 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3 < 0:
        tau = mod2pi(math.atan2(eta * A - xi * B, xi * A + eta * B) + math.pi)
    else:
        tau = mod2pi(math.atan2(eta * A - xi * B, xi * A + eta * B))
    
    omega = mod2pi(tau - u + v - phi)

    return tau, omega
