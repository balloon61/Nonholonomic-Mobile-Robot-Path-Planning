#!/usr/bin/env python3

import numpy as np
import rospkg
import rospy

import sys, os
from RS import *





def test_tf(start: tf, goal: tf)->list:
    # Transofrmation
    t = start.transformation(goal)
    # T = start.inverse_matrix().get_matrix() @ goal.get_matrix()
    # print(T)
    # x, y, theta = T[0, 3], T[1, 3], math.atan2(T[0, 1], T[0, 0])
    x, y= t.t_vec[:2]
    theta = t.r_vec[2]
    print(x, y, theta) 
    print(f"rotation vector: {t.r_vec}\n translation vector: {t.t_vec}")


check = False
if __name__ == '__main__':
    rospy.init_node("RS_test")

    rospy.loginfo("Building the list of observer positions.")

    # Test map subscirver
    # test_map_sub = Map("map")
    # test_map_sub.Wait_for_map_update()
    
    # Test Path subscriber
    # test_path = Global_Path(path_topic="/move_base/DWAPlannerROS/global_plan")
    # test_path.Wait_for_path_update()

    # test_path.Get_Waypoints(interval=5)

    # print(test_path.way_points)

    # Test collision
    # if check == False:
    #     RPCC = RS_Path_Collision_Check()
    #     rospy.sleep(5.0)
    #     path_list = list()

    #     while True:
    #         if RPCC.map.wait_for_readmapinfo == True and RPCC.map.wait_for_readmap == True:
    #             continue
    #         else:
    #             for x in np.arange(-14, 14, 0.5):
    #                 path_list = list()
                
    #                 for y in np.arange(-14, 14, 0.5):
    #                     path_list.append([x, y])
                
    #                     result = RPCC.Collision_detect(path_point=path_list)
    #                     if result == True:
    #                         rospy.loginfo("Collide %f %f", x, y)
    #                     path_list.clear()

    #             break    
    # test_path_sub = Global_Path


    RS_path = ReedShepp_Path(constraint=1.0)
    paths = RS_path.find_RSpath(1.5, 1.5, 1)
    RPCC = RS_Path_Collision_Check()
    RPCC.get_RSPath(paths[0])
    pp = RPCC.Get_path(interval=4)
    print(pp)
    # points = RPCC.Get_path(interval=100)
    # points = RPCC.get_line(np.array([1.0, 2.0, np.pi/2]), 1.0, 100, 1)
    # points1 = RPCC.get_curve(np.array([1.0, 2.0, np.pi / 2]), length=2 * np.pi, steering=1, dir=1, number_of_points=4, radius=2.0)
    # points2 = RPCC.get_curve(np.array([1.0, 2.0, np.pi / 2]), length=2 * np.pi, steering=-1, dir=1, number_of_points=4, radius=4.0)
    # points3 = RPCC.get_curve(np.array([1.0, 2.0, np.pi / 2]), length=2 * np.pi, steering=1, dir=-1, number_of_points=4, radius=1.0)
    # points4 = RPCC.get_curve(np.array([1.0, 2.0, np.pi / 2]), length=2 * np.pi, steering=-1, dir=-1, number_of_points=4, radius=0.5)
    
    # print("st: 1 dir: 1", points1)
    # print("st: -1 dir: 1", points2)
    # print("st: 1 dir: -1", points3)
    # print("st: -1 dir: -1", points4)

    # change length
    # points1 = RPCC.get_curve(np.array([1.0, 2.0, np.pi]), length=np.pi / 2, steering=1, dir=1, number_of_points=4, radius=2.0)
    # points2 = RPCC.get_curve(np.array([1.0, 2.0, np.pi]), length=np.pi / 3, steering=-1, dir=1, number_of_points=4, radius=2.0)
    # points3 = RPCC.get_curve(np.array([1.0, 2.0, np.pi]), length=np.pi / 4, steering=1, dir=-1, number_of_points=4, radius=2.0)
    # points4 = RPCC.get_curve(np.array([1.0, 2.0, np.pi]), length=np.pi / 5, steering=-1, dir=-1, number_of_points=4, radius=2.0)
    
    # print("length", np.pi / 2, points1)
    # print("length", np.pi / 3, points2)
    # print("length", np.pi / 4, points3)
    # print("length", np.pi / 5, points4)

    # change theta
    # points1 = RPCC.get_curve(np.array([1.0, 2.0, np.pi / 2]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=2.0)
    # points2 = RPCC.get_curve(np.array([1.0, 2.0, np.pi / 3]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=2.0)
    # points3 = RPCC.get_curve(np.array([1.0, 2.0, np.pi / 4]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=2.0)
    # points4 = RPCC.get_curve(np.array([1.0, 2.0, np.pi / 5]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=2.0)
    
    # print(points1)
    # print(points2)
    # print(points3)
    # print(points4)

    # change radius
    # points1 = RPCC.get_curve(np.array([1.0, 2.0, np.pi]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=2.0)
    # points2 = RPCC.get_curve(np.array([1.0, 2.0, np.pi]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=1.0)
    # points3 = RPCC.get_curve(np.array([1.0, 2.0, np.pi]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=0.5)
    # points4 = RPCC.get_curve(np.array([1.0, 2.0, np.pi]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=2.5)
    
    # print(points1)
    # print(points2)
    # print(points3)
    # print(points4)


    # change x, y
    # points1 = RPCC.get_curve(np.array([4.0, 3.0, np.pi / 2]), length=np.pi, steering=1, dir=1, number_of_points=4, radius=2.0)
    # points2 = RPCC.get_curve(np.array([1.0, -2.0, np.pi / 4]), length=np.pi, steering=-1, dir=1, number_of_points=4, radius=2.0)
    # points3 = RPCC.get_curve(np.array([-11.0, 2.0, np.pi / 8]), length=np.pi, steering=1, dir=-1, number_of_points=4, radius=2.0)
    # points4 = RPCC.get_curve(np.array([1.0, -22.0, np.pi / 16]), length=np.pi, steering=-1, dir=-1, number_of_points=4, radius=2.0)
    
    # print(points1)
    # print(points2)
    # print(points3)
    # print(points4)
    # points = RPCC.get_curve(np.array([1.0, 2.0, 3 * np.pi / 2]), np.pi / 2, 1, 1, 100, 2.0)

    # test_points0 = [[2.5, 2.5, 1.3], [2.5, -2.5, -1], [-2.5, 2.5, -1], [-2.5, -2.5, 1], [0.5, 0.5, 1], [0.5, -0.5, -1], [-0.5, 0.5, -1], [-0.5, -0.5, 1]]
    # test_points1 = [[0.5, 0.5, -3], [2.5, -2.5, -1.5], [-2.5, 2.5, -1.5], [-2.5, -2.5, 1.5], [0.5, 0.5, -1.5], [0.5, -0.5, -1.5], [-0.5, 0.5, -1.5], [-0.5, -0.5, 1.5]]
    # test_points2 = [[2.5, 2.5, 0.4], [2.5, -2.5, 0], [-2.5, 2.5, 0.2], [-2.5, -2.5, 0], [0.5, 0.5, 0.2], [0.5, -0.5, 0], [-0.5, 0.5, 0], [-0.5, -0.5, 0]]
    # test_points3 = [[2.5, 2.5, -1], [2.5, -2.5, 1], [-2.5, 2.5, 1], [-2.5, -2.5, -1], [0.5, 0.5, -1], [0.5, -0.5, 1], [-0.5, 0.5, 1], [-0.5, -0.5, -1]]




    # frame1 = tf(3).set_translation([10., 1., 0.])
    # frame1.set_rotation(rotvec=[0., 0., math.pi/2])
    # frame2 = tf(3).set_translation([1., 5., 0.])
    # frame2.set_rotation(rotvec=[0., 0., 0.])
    # test_tf(frame1, frame2)
    # print(test_map_sub.map)


    rospy.spin()


 




















