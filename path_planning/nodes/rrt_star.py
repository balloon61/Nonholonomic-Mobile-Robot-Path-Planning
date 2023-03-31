#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import argparse
import sys
import random
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from decimal import Decimal

resolution = 0
width, height = 0, 0
map = list()
origin = Pose()


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

    def set_node(self, x: float, y: float):
        self.x = x
        self.y = y

    def set_parent(self, parent: Node):
        self.parent = parent


class RRT_Star:

    def __init__(self, gmap: list, mapres: float, width: int, height: int, origin: Pose, step_len: float,
                 radius: float):
        # map parameters

        self.__gridmap = gmap
        self.__resolution = mapres
        self.__width = width
        self.__height = height
        self.__origin = origin

        # RRT star parameters
        self.__start_node = Node(origin.position.x, origin.position.y)
        self.__goal_node = Node(0, 0)
        self.__iterations_times = 2000
        self.cartesian_to_grid_dict = self.create_cartesian_to_grid()
        self.__radius = radius
        self.__vertex = [np.array([origin.position.x, origin.position.y])]
        self.__step_len = step_len

    #######################################################################################
    def Get_RRT_path_planner(self, goal: Pose) -> np.ndarray:
        self.__goal_node.set_node(goal.position.x, goal.position.y)

        for i in range(self.__iterations_times):
            new_node = self.get_feasible_random_point()
            nearest_node = self.near_neighbor()
            if node_new and not self.utils.is_collision(node_near, node_new):
                neighbor_index = self.find_near_neighbor(node_new)
                self.vertex.append(node_new)

                if neighbor_index:
                    self.choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)

        index = self.search_goal_parent()
        self.path = self.extract_path(self.vertex[index])
        return

    #######################################################################################
    def create_cartesian_to_grid(self, x, y) -> ndarray:
        return [round(x / self.__resolution) - self.__origin[0] / self.__resolution, round(y / self.__resolution) - self.__origin[0] / self.__resolution]
      #  return [round(x / 0.05) + 1000, round(y / 0.05) + 1000]

    def grid_to_cartesian(self, gx, gy) -> list:
        return [gx * self.__resolution + self.__origin[0], gy * self.__resolution - self.__origin[1]]
       # return [gx * self.__resolution - 50, gy * self.__resolution - 50]

    def get_feasible_random_point(self) -> list:
        x, y = random.random * self.__width - 1, random.random * self.__height - 1
        while self.__gridmap[x, y] != 0 and [x, y] not in self.__replicated_point:
            x, y = random.random * self.__width - 1, random.random * self.__height - 1
        self.__replicated_point[[x, y]] = 'selected'
        return np.array([x, y])

    #######################################################################################
    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not self.utils.is_collision(self.vertex[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertex) - 1

    #######################################################################################
    #######################################################################################
    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]
    #######################################################################################

    def get_distance(self, p1: ndarray, p2: ndarray) -> float:
        return np.linalg.norm(p1 - p2)

    def check_path_valid(self) -> bool:
        return False

    #######################################################################################
    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    #######################################################################################
    def near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.utils.is_collision(node_new, self.vertex[ind])]

        return dist_table_index


def map_callback(map_callback):
    global map
    map = map_callback.data


# print(len(map_callback.data), map_callback.info)  # , len(map.data[0]))


def map_info_callback(data):
    global resolution, width, height, origin
    resolution, width, height = data.resolution, data.width, data.height
    origin = data.origin


if __name__ == "__main__":
    rospy.init_node("rrt_star")

    rospy.Subscriber('/map_metadata', MapMetaData, map_info_callback)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    Path_planner = RRT_Star(map, resolution, width, height, origin)
    path = Path_planner.RRT_path_planner()
    rospy.spin()
