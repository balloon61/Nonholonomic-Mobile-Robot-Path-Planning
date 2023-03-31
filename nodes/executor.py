#!/usr/bin/env python

import numpy as np
from math import pi

import rospy
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float64
import sympy as sp
from sympy.physics.mechanics import dynamicsymbols
import time
from pyquaternion import Quaternion


def __yaw_to_quat(yaw):
    """
      Computing corresponding quaternion q to angle yaw [rad]
      :param yaw
      :return: q
      """
    q = Quaternion(axis=[0, 0, 1], angle=yaw)
    return q.elements



class ROBOT:
    def __init__(self) -> None:
        # publisher
        self.pub_linear_slide = rospy.Publisher('/mmrobot/linear_joint/command', Float64, queue_size=10)

        self.pub_left_arm_j1 = rospy.Publisher('/mmrobot/l_arm_joint1/command', Float64, queue_size=10)
        self.pub_left_arm_j2 = rospy.Publisher('/mmrobot/l_arm_joint2/command', Float64, queue_size=10)
        self.pub_left_arm_j3 = rospy.Publisher('/mmrobot/l_arm_joint3/command', Float64, queue_size=10)
        self.pub_left_gripper = rospy.Publisher('/mmrobot/l_gripper_joint/command', Float64, queue_size=10)

        self.pub_right_arm_j1 = rospy.Publisher('/mmrobot/r_arm_joint1/command', Float64, queue_size=10)
        self.pub_right_arm_j2 = rospy.Publisher('/mmrobot/r_arm_joint2/command', Float64, queue_size=10)
        self.pub_right_arm_j3 = rospy.Publisher('/mmrobot/r_arm_joint3/command', Float64, queue_size=10)
        self.pub_right_gripper = rospy.Publisher('/mmrobot/r_gripper_joint1/command', Float64, queue_size=10)

        self.arm_reach_goal = rospy.Publisher('/arm_goal', String, queue_size=10)

        self.pose = np.array([0., 0., 0.])
        time.sleep(1)

        self.slide_height = -0.3

        self.left_q = np.array([self.slide_height, 0, pi/8, -pi/4])
        self.right_q = np.array([self.slide_height, 0, pi/8, -pi/4])
        self.l_offset = np.array([0, 0.11, 0.14, 1.21])
        self.r_offset = np.array([0, -0.11, -1.79, 1.19])


        self.init_robot()

        # self.l_offset1 = 0.11
        # self.l_offset2 = 0.14
        # self.l_offset3 = 1.21

        # self.r_offset1 = -0.11
        # self.r_offset2 = -1.79
        # self.r_offset3 = 1.19
    

    def init_robot(self):

        rospy.loginfo("initial robot")
        self.pub_linear_slide.publish(self.slide_height)
        self.pub_left_arm_j1.publish(self.left_q[1] + self.l_offset[1])
        self.pub_left_arm_j2.publish(self.left_q[2] + self.l_offset[2])
        self.pub_left_arm_j3.publish(self.left_q[3] + self.l_offset[3])

        self.pub_right_arm_j1.publish(self.right_q[1] + self.r_offset[1])
        self.pub_right_arm_j2.publish(self.right_q[2] + self.r_offset[2])
        self.pub_right_arm_j3.publish(self.right_q[3] + self.r_offset[3])
        # rospy.loginfo("initial robot")


    def update_arm_pose(self, slide_height:float, left_joint, right_joint):
        self.slide_height = slide_height
        self.left_q[1:-1] = left_joint
        self.right_q[1:-1] = right_joint
        self.init_robot()

    def __yaw_to_quat(self, yaw: float):
        """
        Computing corresponding quaternion q to angle yaw [rad]
        :param yaw
        :return: q
        """
        q = Quaternion(axis=[0, 0, 1], angle=yaw)
        return q.elements
    

    def movebase_client(self, x:float, y:float, yaw:float): # x:float, y:float, yaw:float
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        q = self.__yaw_to_quat(yaw)
    
        goal.target_pose.pose.orientation.w = q[0]
        goal.target_pose.pose.orientation.x = q[1]
        goal.target_pose.pose.orientation.y = q[2]
        goal.target_pose.pose.orientation.z = q[3]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()












