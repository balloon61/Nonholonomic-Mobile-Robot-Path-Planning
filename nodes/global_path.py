#!/usr/bin/env python
import rospy
import numpy as np

from transform import *

from nav_msgs.msg import Path

import string

# 1. Subscriber for the move_base global plan
class Global_Path:
    def __init__(self, path_topic:string) -> None:
        self.map = None

        # self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback)
        self.path = None
        self.wait_for_path = True
        self.way_points = list()
        # self.Wait_for_map_update()

    def path_callback(self, data):
        
        rospy.loginfo("Reading Path.")
        self.path = data.poses
        self.wait_for_path = False


    def Wait_for_path_update(self):
        while(self.wait_for_path):
            rospy.loginfo("Waiting for Path Update.")
        rospy.loginfo("Path Update.")

        self.wait_for_path = True

    def Get_Waypoints(self, interval:int):
        """
        Return interval + 1 points from the starting point to the end point
        """
        i = 0
        num = len(self.path)

        while i < num - 1:
            
            self.way_points.append(self.path[i])
            i = i + int(num / interval)
        
        self.way_points.append(self.path[-1])


    def Clear_Waypoints(self):
        """
        Reset the list of waypoints
        """
        self.way_points = []