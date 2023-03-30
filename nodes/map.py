#!/usr/bin/env python
import rospy
import numpy as np

import sys
from transform import *
# sys.path.append('../')

from nav_msgs.msg import OccupancyGrid, MapMetaData
import string



class Map:
    def __init__(self, map_topic:string="map", map_info_topic:string="map_metadata") -> None:
        """
        map object
        """
        self.map = None

        # self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        self.map_meta_data_sub = rospy.Subscriber(map_info_topic, MapMetaData, self.mapmetadata_callback)
        self.resolution = None
        self.origin = None
        self.negate = None
        self.occupied_thresh = None
        self.width = 0
        self.height = 0
        self.map = None
        # self.free_thresh = rospy.get_param("free_thresh", default=0.196)        
        self.wait_for_readmap = True
        self.wait_for_readmapinfo = True
        # self.Wait_for_map_update()


    def mapmetadata_callback(self, metadata):
        """
        map info callback function
        """
        rospy.loginfo("Reading MapData.")

        self.width = metadata.width
        self.height = metadata.height
        self.origin = metadata.origin
        self.resolution = metadata.resolution

        self.wait_for_readmapinfo = False

    def map_callback(self, data):
        """
        map callback function
        """
        rospy.loginfo("Reading Map.")

        self.map = data
        # self.height = data.height
        # self.width = data.width
        self.wait_for_readmap = False


    def Wait_for_map_update(self):
        """
        Wait until we get the map
        """
        while(self.wait_for_readmap and self.wait_for_readmapinfo):
            rospy.loginfo("Waiting for Map Update.")
            rospy.sleep(1.0)
        rospy.loginfo("Map and Map info Update.")
    
    def getmap_value(self, pose:list()):
        """
        The input of this function is the pose: (x, y) of the robot
        """
        x, y, _ = pose
        pose_width, pose_height = int((x - self.origin.position.x) / self.resolution), int((y - self.origin.position.y) / self.resolution)
        map_array_position = self.width * pose_height + pose_width
        # Transfer the pose to the grid (2D)

        return self.map.data[map_array_position]




















    # def get_line(self, length:float, interval:float, pos:np.ndarray, dir:int) -> list():

    #     path = np.array([0., 0., 0., 1.])
    #     for i in range(int(length / interval)):
    #         x = i * interval * dir
    #         path = np.append(path, [[x, 0., 0., 1.]], axis=0)

    #     # return np.array(l).dot(tf.get_matrix())
    #     start_point = tf(3).set_translation([pos[0], pos[1], 0.]) # [x, y, z]
    #     start_point.set_rotation(rotvec=[0., 0., pos[2]])  # [roll, pitch, yaw]

    #     points = start_point.get_matrix()@path.T
    #     # points = points.T
    #     return points.T
    


    # def get_curve(self, length:float, interval:float, pos:np.ndarray, direction:np.ndarray, steering:int, radius:float) -> np.ndarray:

    #     """
    #     Input:
    #     Output: a list of points 
    #     """
    #     path = np.array([0., 0., 0., 1.])
    #     center = np.array([steering * radius, 0., 0., 0.])
        
    #     for theta in range(int(length / interval)):
    #         t = theta * interval * direction
    #         new_point = center + [radius * np.cos(t) * (-steering), radius * np.sin(t), 0., 1.] 
    #         path.append(path, [new_point], axis=0)
       
    #     start_point = tf(3).set_translation([pos[0], pos[1], 0.]) # [x, y, z]
    #     start_point.set_rotation(rotvec=[0., 0., pos[2]])  # [roll, pitch, yaw]

    #     points = start_point.get_matrix()@path.T
    #     points = points.T

    #     return 

# class Line:

#     def __init__(self, length:float, interval:float, pos:np.ndarray, direction:int) -> None:
#         self.length = length
#         self.interval = interval 
#         self.pos = pos
#         self.direction = direction
    
#     def get_line(self) -> np.ndarray:
        
#         path = np.array([0., 0., 0., 1.])
#         for i in range (self.length / self.interval):
#             x = i * self.interval * self.direction
#             path = np.append(path, [[x, 0., 0., 1.]], axis=0)

#         # return np.array(l).dot(tf.get_matrix())
#         start_point = tf(3).set_translation([self.pos[0], self.pos[1], 0.]) # [x, y, z]
#         start_point.set_rotation(rotvec=[0., 0., self.pos[2]])  # [roll, pitch, yaw]

#         points = start_point.get_matrix()@path.T
#         points = points.T
#         return points

# class Curve:

#     def __init__(self, length:float, interval:float, pos:np.ndarray, direction:np.ndarray, steering:int, radius:float) -> None:        
        
#         self.length = length
#         self.interval = interval
#         self.pos = pos
#         self.direction = direction
#         self.steering = steering
#         self.r = radius


#     def get_center(self) -> list:
#         # if self.direction 'Left':
#         return [self.steering * self.r, 0., 0., 0.] # Left:1, Right:-1
#         # return np.array([self.steering * self.r, 0., 0., 0.]) # Left:1, Right:-1

#     def get_curve(self) -> np.ndarray:

#         path = np.array([0., 0., 0., 1.])
#         center = self.get_center()
        
#         for theta in range(self.length / self.interval):
#             t = theta * self.interval * self.direction
#             new_point = center + [self.r * np.cos(t) * (-self.steering), self.r * np.sin(t), 0., 1.] 
#             np.append(path, [new_point], axis=0)
       
#         start_point = tf(3).set_translation([self.pos[0], self.pos[1], 0.]) # [x, y, z]
#         start_point.set_rotation(rotvec=[0., 0., self.pos[2]])  # [roll, pitch, yaw]

#         points = start_point.get_matrix()@path.T
#         points = points.T

#         return points
