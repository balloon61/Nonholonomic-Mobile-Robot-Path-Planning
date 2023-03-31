#!/usr/bin/env python
import rospy
import numpy as np

from transform import *
import std_msgs.msg

from nav_msgs.msg import Path
from nav_msgs.srv import GetPlanRequest, GetPlan
from geometry_msgs.msg import PoseStamped
from calculation import *
import string

# 1. Subscriber for the move_base global plan
class Global_Path:
    def __init__(self, path_topic:string) -> None:
        """
        Global path object
        """
        self.map = None

        # self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        # self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback)
        self.path = None
        self.wait_for_path = True
        self.way_points = list()
        self.header = std_msgs.msg.Header()
        # self.Wait_for_map_update()

    def generate_path(self, start_pos:np.ndarray, goal_pos:np.ndarray):
        rospy.wait_for_service('/move_base/NavfnROS/make_plan')
        rospy.sleep(1.0)
        xs, ys, thetas = start_pos
        xg, yg, thetag = goal_pos
        try:

            start = PoseStamped()

            start.header.frame_id = 'map'
            start.pose.position.x = xs
            start.pose.position.y = ys
            qs = get_quaternion_from_euler(0, 0, thetas)
            # print(qs)
            start.pose.orientation.x = qs[0]
            start.pose.orientation.y = qs[1]
            start.pose.orientation.z = qs[2]
            start.pose.orientation.w = qs[3]

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = xg
            goal.pose.position.y = yg
            qg = get_quaternion_from_euler(0, 0, thetag)
            # print(qg)
            goal.pose.orientation.x = qg[0]
            goal.pose.orientation.y = qg[1]
            goal.pose.orientation.z = qg[2]
            goal.pose.orientation.w = qg[3]
        
            generate_plan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
            
            rospy.sleep(1.0)
            req = GetPlanRequest()
            rospy.sleep(1.0)

            req.start = start
            req.goal = goal
            req.tolerance = 10.0

            generated_plan = generate_plan(req)
            self.path = generated_plan.plan.poses
            # print("plan:\n", self.path)
            # return generated_plan.plan, generated_plan.plan.poses

        except rospy.ServiceException as e:
            print("Service call failed: %s", e)

    def path_callback(self, data):
        
        rospy.loginfo("Reading Path.")
        self.path = data.poses
        self.header = data.header
        self.wait_for_path = False


    def Wait_for_path_update(self):
        while(self.wait_for_path):
            rospy.loginfo("Waiting for Path Update.")
            rospy.sleep(1.0)
        rospy.loginfo("Path Update.")

        self.wait_for_path = True

    def Get_Waypoints(self, interval:int):
        """
        Return interval + 1 points from the starting point to the end point
        """
        i = 0
        num = len(self.path)

        while i < num - 1:
            print("waypoints:", self.path[i])
            self.way_points.append(self.path[i])
            i = i + int(num / interval)
        
        self.way_points.append(self.path[-1])
        print(self.way_points[-1])
        # print(self.way_points)

    def Clear_Waypoints(self):
        """
        Reset the list of waypoints
        """
        self.way_points = []