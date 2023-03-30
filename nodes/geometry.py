#!/usr/bin/env python

import rospy
import numpy as np


def get_line(start:np.ndarray, length:float, number_of_points:int, direction: int):
    """
    Input: start pose (np.array), length: this is provided by RS curve, num_of_points: how many point you want to check if there have collision with obstacle
    Output: a list of points, from the begining pose to the end
    This function using basic triangular function to calculate the intermediate point.
    """
    x, y, theta = start
    line = list()
    # direciotn = -1 (backward), 1 (forward)
    point_distance = length / number_of_points * direction
    for i in range(number_of_points + 1):
        # There have totally NoP + 1 of points, including the begining point and the end point 
        line.append([x + np.cos(theta) * i * point_distance, y + np.sin(theta) * i * point_distance, theta])

    return line
    
def get_curve(start:np.ndarray, length:float, steering:int, dir:int, number_of_points:int=100, radius:float=1.):
    """
    Input: start pose (np.array), length: this is provided by RS curve, num_of_points: how many point you want to check if there have collision with obstacle
           steering: 1 (left), -1 (right)
    Output: a list of points, from the begining pose to the end
    This function using basic triangular function to calculate the intermediate point.
    """

    x, y, theta = start
    curve = list()
    point_distance = length / (number_of_points * radius) 
    center = np.array([-np.sin(theta) * steering * radius + x, np.cos(theta) * steering * radius + y, 0])
    # if the radius is 1, then the length is the radian you turn.
    for i in range(number_of_points + 1):
        t = i * point_distance * dir * steering + theta 
        new_point = center + np.array([radius * np.cos(t - np.pi / 2 * steering), radius * np.sin(t - np.pi / 2 * steering), t]) 
            
        curve.append(new_point.tolist())

    return curve








    # def get_line(self, start:np.ndarray, length:float, number_of_points:int, direction: int):
    #     """
    #     Input: start pose (np.array), length: this is provided by RS curve, num_of_points: how many point you want to check if there have collision with obstacle
    #     Output: a list of points, from the begining pose to the end
    #     This function using basic triangular function to calculate the intermediate point.
    #     """
    #     x, y, theta = start
    #     line = list()
    #     # direciotn = -1 (backward), 1 (forward)
    #     point_distance = length / number_of_points * direction
    #     for i in range(number_of_points + 1):
    #         # There have totally NoP + 1 of points, including the begining point and the end point 
    #         line.append([x + np.cos(theta) * i * point_distance, y + np.sin(theta) * i * point_distance, theta])

    #     return line
    
    # def get_curve(self, start:np.ndarray, length:float, steering:int, dir:int, number_of_points:int=100, radius:float=1.):
    #     """
    #     Input: start pose (np.array), length: this is provided by RS curve, num_of_points: how many point you want to check if there have collision with obstacle
    #            steering: 1 (left), -1 (right)
    #     Output: a list of points, from the begining pose to the end
    #     This function using basic triangular function to calculate the intermediate point.
    #     """

    #     x, y, theta = start
    #     curve = list()
    #     point_distance = length / (number_of_points * radius) 
    #     center = np.array([-np.sin(theta) * steering * radius + x, np.cos(theta) * steering * radius + y, 0])
    #     # if the radius is 1, then the length is the radian you turn.
    #     for i in range(number_of_points + 1):
    #         t = i * point_distance * dir * steering + theta 
    #         new_point = center + np.array([radius * np.cos(t - np.pi / 2 * steering), radius * np.sin(t - np.pi / 2 * steering), t]) 
            
    #         curve.append(new_point.tolist())

    #     return curve
