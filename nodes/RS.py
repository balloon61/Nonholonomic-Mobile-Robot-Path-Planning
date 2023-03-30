#!/usr/bin/env python

import tf
import rospy
import sys
sys.path.append('/utils')

from calculation import *
from transform import *
from map import *
from map import *
from global_path import *
from executor import *
from geometry import *
# import nodes.tf_broadcaster as tf_broadcaster
# from htransform import *
# from enum import Enum

from bidict import bidict
from itertools import chain
from geometry_msgs.msg import PoseStamped


EPS = 1e-6

    
class SubPath:
    """
    Sub path for the RS curve. 
    """
    def __init__(self, length: float, steering: string, direction: string):
        self.Steering_dict = bidict({
            'Right': -1,
            'Straight': 0,
            'Left': 1
        })
        self.Direction_dict = bidict({
            'Forward': 1,
            'Backward': -1
        })
        self.steering = steering
        self.direction = direction
        self.distance = length
        self.r = 1.

    def change_direction(self):
        """
        Reverse the direction forward -> backward, backward -> forward
        """
        self.direction = self.Direction_dict.inverse[self.Direction_dict[self.direction] * -1]

    def change_steering(self):
        """
        Change the steering left -> right, right -> right
        """
        self.steering = self.Steering_dict.inverse[self.Steering_dict[self.steering] * -1]

    def set_radius(self, r:float):
        """
        change the minimum turning radius.
        """
        self.r = r

# A class that describes a line
class RS_Path_Collision_Check:
    """
    A class for collision checking
    use map object and path object
    """

    def __init__(self, map_topic:string="map", path_topic:string="/move_base/DWAPlannerROS/global_plan"):
        self.RSpath = None
        self.map = Map(map_topic=map_topic)
        self.map.Wait_for_map_update()
        self.global_path = Global_Path(path_topic=path_topic) 

    # def __init__(self):
    # This init is used for sub function testing
    #     self.RSpath = None
    #     # self.map = Map(map_topic="map")
    #     # self.Global_Path(path_topic="/move_base/DWAPlannerROS/global_plan") 


    def get_RSPath(self, RS):
        """
        RS: a list of sub path, which is generated from the RS curve
        """
        self.RSpath = RS

    def Get_path_points(self, number_of_points:int=100) -> list:
        """
        Input: The number of points
        Output: A list of points
        Return a list of points along the Reed-Shepp path 
        """        
        path_point = list()
        theta = 0.
        pos = np.array([0., 0., theta])
        for sub in self.RSpath: # SubPath(t, 'Left', 'Forward')    
            steering = sub.Steering_dict[sub.steering]
            dir = sub.Direction_dict[sub.direction]

            if sub.steering == 'Straight':
                # path_point.append(self.get_line(pos, sub.distance, interval, dir))
                path_point = path_point + get_line(np.array(pos), sub.distance, number_of_points, dir)
            else:
                # path_point.append(self.get_curve(pos, sub.distance, steering, dir, interval, sub.r))
                path_point = path_point + get_curve(np.array(pos), sub.distance, steering, dir, number_of_points, sub.r)
                # Get the next theta
                # theta = theta + sub.distance * dir * steering / sub.r

            print(path_point[-1])
            pos = path_point[-1]
            
        return path_point
    
    def Collision_detect(self, path_point:list()):
        """
        Input: A list of points, you can get the points from the Get_path_points(), or (get_line() and get_curve function)
        Output: bool
        This function used the occupancy (map.data) in the map topic, check if there have any collision between the map and the path, 
        since we have inflate the map, we do not need to check a lotf of points. 
        """

        for pt in path_point:

            occupancy = self.map.getmap_value(pt)
            if occupancy == 100 or occupancy == -1: # -1: unknown, 0: free, 100: occupied 
                return True

        return False
    


class ReedShepp_Path:

    def __init__(self, constraint:float = 1.):
        self.path_publisher = rospy.Publisher("RS_path", Path, queue_size=10)
        rospy.sleep(1.0)
        self.path = []
        self.best_path = None
        self.rad = constraint
        self.collision_checker = RS_Path_Collision_Check()
        self.robot = ROBOT()
        self.RSPath_topic = Path()


    def Publish_Path(self, total_path):
        """
        publish path topic 
        """
        self.show_path(total_path)
        poses = []
        points = self.collision_checker.Get_path_points(number_of_points=1050)
        for pt in points:

            newPose = PoseStamped()

            x, y, yaw = pt
            # rospy.loginfo(f"path points x: {x}, y: {y}, theta: {yaw}")

            newPose.header = self.RSPath_topic.header
            newPose.pose.position.x = x
            newPose.pose.position.y = y
            
            quaternion = get_quaternion_from_euler(0, 0, yaw)
            newPose.pose.orientation.x = quaternion[0]
            newPose.pose.orientation.y = quaternion[1]
            newPose.pose.orientation.z = quaternion[2]
            newPose.pose.orientation.w = quaternion[3]

            poses.append(newPose)

        self.RSPath_topic.poses = poses
        rospy.loginfo("publishing path")

        # self.path_publisher.publish(self.RSPath_topic)

    def change_constraint(self, new_constraint:float):
        self.rad = new_constraint

    def show_path(self, path):
        """
        Print the detail of the path.
        """
        for p in path:
            print(f"Direction: {p.direction}, Steering: {p.steering}, Length: {abs(p.distance)}")
        print("\n")

    def reset_path(self):
        """
        reset the RS path
        """
        self.path.clear()

    def get_path_length(self, path: list()) -> float:
        """
        Get the length of the RS path
        """
        length = 0.
        if path == None:
            return 0
        for sub in path:
            if sub == None:
                return 0
            length += abs(sub.distance)
        return length
    
    def find_path(self) -> list():
        """
        This function find the RS paths from the global path get from moveit
        Gets a list of waypoint and design the RS curve from waypointt to next waypoint
        """
        # wait for global path update
        rospy.loginfo("please set your goal point via move base client or Rviz")
        self.collision_checker.global_path.Wait_for_path_update()
        self.RSPath_topic.header = self.collision_checker.global_path.header
        # get way points
        number_of_waypoints = 4
        self.collision_checker.global_path.Get_Waypoints(interval=number_of_waypoints)
        
        total_path = list()
        # rospy.loginfo("waypoints:", self.collision_checker.global_path.way_points)
        start = self.collision_checker.global_path.way_points[0]
        # initialize two frame
        frame1 = tf(3)
        frame2 = tf(3)
        # set up frame1
        frame1.set_translation([start.pose.position.x, start.pose.position.y, 0])
        _, _, yaw = get_rotation(start.pose.orientation)
        frame1.set_rotation(rotvec=[0., 0., yaw])
        for way_pts in self.collision_checker.global_path.way_points[1:-1]:

            # setup frame2
            frame2.set_translation([way_pts.pose.position.x, way_pts.pose.position.y, 0])
            _, _, yaw2 = get_rotation(way_pts.pose.orientation)
            frame2.set_rotation(rotvec=[0., 0., yaw2])      

            # Transofrmation
            GoalfromStart = frame1.transformation(frame2)
            x, y = GoalfromStart.t_vec[:2]
            theta = GoalfromStart.r_vec[2]
            # divide by radius
            x = x / self.rad
            y = y / self.rad
            # find Reed-Shepp path
            paths = self.find_RSpath(x, y, theta)
        
            for path in paths:
                # check collision

                # the distance in path is actually radian so the straight line need to multiply by self.rad 
                # PSCC = RS_Path_Collision_Check(path)
                self.collision_checker.get_RSPath(path)
                pts = self.collision_checker.Get_path_points(number_of_points=20)
                # true if no collision, since the paths is already sorted, it return the shortest path without collision
                if self.collision_checker.Collision_detect(pts) == False:
                    total_path.append(path)
                    break
            rospy.loginfo("collision free RS path found")
            self.show_path(total_path[-1])
            # update frame1 to next frane

            frame1.set_translation([way_pts.pose.position.x, way_pts.pose.position.y, 0])
            frame1.set_rotation(rotvec=[0., 0., yaw2])  
            

        rospy.loginfo("Solution found")
        for p in total_path:
            self.show_path(p)
        
        flatten_total_path = list(chain.from_iterable(total_path))


        self.Publish_Path(flatten_total_path)
        return total_path


    def find_RSpath(self, x: float, y: float, theta: float) -> list:
        """
        Find the Nonholonomic path
        Input: goal: (x, y, theta), after transformation
        output: all path available in RS algorithm (without collision checking)
        """
        # transformation between origin and goal
        # s = HTransform(2).set_translation(start[:2]).set_rotation(start[3])
        # g = HTransform(2).set_translation(goal[:2]).set_rotation(goal[3])

        # get all path function
        # 
        path_fns = [self.path1, self.path2, self.path3, self.path4, self.path7, self.path8, self.path9, self.path10, self.path11, self.path12]
        # test_path_fns = [self.path9]

        paths = list()
        # run all path function include original, timeflip, reflection, timeflip before reflection
        for path in path_fns:

            # run all path including timeflip and reflection and find the shortest
            p = path(x, y, theta)
            tf_p = self.timeflip(path(-x, y, -theta))
            rf_p = self.reflection(path(x, -y, -theta))
            rt_p = self.reflection(self.timeflip(path(-x, -y, theta)))

            paths.append(p)
            paths.append(tf_p)
            paths.append(rf_p)
            paths.append(rt_p)

        # remove empty paths
        paths = list(filter(None, paths))  
            
        # get the shortest path, but have not check the collision
        paths.sort(key=lambda x: self.get_path_length(x))

        for p in paths:
            self.show_path(p)
        return paths

    def timeflip(self, path: list()) -> list():
        """
        Helper function for calculate all 48 RS path
        """
        if path is None:
            return None
        for sp in path:
            sp.change_direction()
        return path

    def reflection(self, path: list()):
        """
        Helper function for calculate all 48 RS path
        """
        if path is None:
            return None
        for sp in path:
            sp.change_steering()
        return path

    def path_length(path):
        """
        Calculate the total length of a RS path.
        """
        if path is None:
            return None
        if len(path) == 0: # if list is empty
            return
        return sum([e.param for e in path])

    # Check, Correct
    def path1(self, x, y, phi):
        """
        Formula 8.1: CSC (same turns)
        """
        u, t = polar(x - math.sin(phi), y - 1 + math.cos(phi))
        if t >= 0:
            v = mod2pi(phi - t)
            if v >= 0:
                # Check whether the result is correct, only compare to the theta
                if (abs(mod2pi(t + v - phi)) < EPS):
                    return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Straight', 'Forward'), SubPath(v, 'Left', 'Forward')]
                    

    # Check, Correct
    def path2(self, x, y, phi):
        """
        Formula 8.2: CSC (opposite turns)
        """
        phi = mod2pi(phi)
        rho, t1 = polar(x + math.sin(phi), y - 1 - math.cos(phi))

        if rho * rho >= 4:
            u = math.sqrt(rho * rho - 4)
            t = mod2pi(t1 + math.atan2(2, u))
            v = mod2pi(t - phi)
            if t >= 0 and v >= 0:
                # Check whether the result is correct, only compare to the theta
                if (abs(mod2pi(t - v - phi)) < EPS):
                    return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Straight', 'Forward'), SubPath(v, 'Right', 'Forward')]


    # Check, Correct
    def path3(self, x, y, phi):
        """
        Formula 8.3: C|C|C
        """
        xi = x - math.sin(phi)
        eta = y - 1 + math.cos(phi)
        rho, theta = polar(xi, eta)

        if rho <= 4:

            u = -2 * math.asin(rho / 4)
            t = mod2pi(theta + u / 2 + math.pi)
            v = mod2pi(phi - t + u)

            if t >= 0 and u <= 0:
                # Check whether the result is correct, only compare to the theta
                if abs(t - u + v - phi) < EPS:
                    return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Right', 'Backward'), SubPath(v, 'Left', 'Forward')]


    # Check Correct but we can ignore path4 since it never be found as the optimal solution
    def path4(self, x, y, phi):
        """
        Formula 8.4: C|CC
        """
        xb = x * math.cos(phi) + y * math.sin(phi)
        yb = x * math.sin(phi) - y * math.cos(phi)
        
        xi = xb - math.sin(phi)
        eta = yb - 1 + math.cos(phi)
        rho, theta = polar(xi, eta)

        if rho <= 4:

            u = -2 * math.asin(rho / 4)
            t = mod2pi(theta + u / 2 + math.pi)
            v = mod2pi(phi - t + u)
            if t >= 0 and u <= 0:
                if t + abs(u) - abs(v) - phi < EPS:
                    return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Right', 'Backward'), SubPath(v, 'Left', 'Backward')]

      #  return None

    # never found a solution
    def path5(self, x, y, phi):

        xi = x - math.sin(phi)
        eta = y - 1 + math.cos(phi)
        rho, theta = polar(xi, eta)

        if rho <= 4:
            u = math.acos(1 - rho * rho / 8)
            A = math.asin(2 * math.sin(u) / rho)
            t = mod2pi(theta + math.pi / 2 - A)
            v = mod2pi(t - u - phi)

            if t >= 0 and u >= 0 and v <= 0:
                if t - u - v - phi < EPS:   
                    return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Right', 'Forward'), SubPath(v, 'Left', 'Backward')]
 
    # Check, Correct
    def path6(self, x, y, phi):
        """
        Type: Lp t|(RnLn) u|Rp v
        """
        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho, theta = polar(xi, eta)
        u1 = (20 - rho * rho) / 16

        if rho <= 6 and 0 <= u1 <= 1:
            u = math.acos(u1)
            A = math.asin(2 * math.sin(u) / rho)
            t = mod2pi(theta + math.pi / 2 + A)
            v = mod2pi(t - phi)

            if t >= 0  and v >= 0:
                return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Right', 'Backward'), SubPath(u, 'Left', 'Backward'), SubPath(v, 'Right', 'Forward')]
       
    # Check, Correct
    def path7(self, x, y, phi):
        """
        Formula 8.7
        """
        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho = (2 + math.sqrt(xi * xi + eta * eta)) / 4

        if rho <= 1:
            u = math.acos(rho)
            t, v = tau_Omega(phi, xi, eta, u, -u)

            if t >= 0 and v <= 0:
                return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Right', 'Forward'), SubPath(u, 'Left', 'Backward'), SubPath(v, 'Right', 'Backward')]

       # return None


    # check,  Correct
    def path8(self, x, y, phi):
        """

        """
        xi = x - math.sin(phi)
        eta = y - 1 + math.cos(phi)
        rho, theta = polar(xi, eta)

        if rho >= 2:
            u1 = math.sqrt(rho * rho - 4)
            u = 2 - u1
            t = mod2pi(theta + math.atan2(u1, -2))
            v = mod2pi(phi - math.pi / 2 - t)

            if t >= 0 and u <= 0 and v <= 0:
                return [SubPath(t, 'Left', 'Forward'), SubPath(math.pi / 2, 'Right', 'Backward'), SubPath(u, 'Straight', 'Backward'), SubPath(v, 'Left', 'Backward')]
       # return None

    # never found a solution
    def path9(self, x, y, phi):
        """

        """
        xb = x * math.cos(phi) + y * math.sin(phi)
        yb = x * math.sin(phi) - y * math.cos(phi)

        xi = xb - math.sin(phi)
        eta = yb - 1 + math.cos(phi)
        rho, theta = polar(xi, eta)

        if rho >= 2:
            u1 = math.sqrt(rho * rho - 4)
            u = 2 - u1
            t = mod2pi(theta + math.atan2(u1, -2))
            v = mod2pi(phi - math.pi / 2 - t)

            if t >= 0 and u >= 0 and v <= 0:
                return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Straight', 'Forward'), SubPath(math.pi / 2, 'Right', 'Forward'), SubPath(v, 'Left', 'Backward')]

    # Check Correct
    def path10(self, x, y, phi):
        """
        Formula 8.10 (1): C|C[pi/2]SC
        """
        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho, theta = polar(xi, eta)

        if rho >= 2:
            t = theta
            u = 2 - rho
            v = mod2pi(-phi + t + math.pi / 2)

            if t >= 0 and u <= 0 and v <= 0:
                return [SubPath(t, 'Left', 'Forward'), SubPath(math.pi / 2, 'Right', 'Backward'), SubPath(u, 'Straight', 'Backward'), SubPath(v, 'Right', 'Backward')]
      #  return None

    # never found a solution
    def path11(self, x, y, phi):
        """
        Formula 8.10 (2): CSC[pi/2]|C
        """
        xb = x * math.cos(phi) + y * math.sin(phi)
        yb = x * math.sin(phi) - y * math.cos(phi)
        
        xi = xb + math.sin(phi)
        eta = yb - 1 - math.cos(phi)
        rho, theta = polar(xi, eta)

        if rho >= 2:
            t = mod2pi(theta)
            u = rho - 2
            v = mod2pi(phi - t - math.pi/2)

            if t >= 0 and v <= 0 and u >= 0:
                return [SubPath(t, 'Left', 'Forward'), SubPath(u, 'Straight', 'Forward'), SubPath(math.pi / 2, 'Left', 'Forward'), SubPath(v, 'Right', 'Backward')]

       # return None

    # Check, result is correct
    def path12(self, x, y, phi):
        """
        Formula 8.11: C|C[pi/2]SC[pi/2]|C
        """
        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho, theta = polar(xi, eta)

        if rho >= 4:
            u = math.sqrt(rho * rho - 4) - 4
            A = math.atan2(2, u + 4)
            t = mod2pi(theta + math.pi / 2 + A)
            v = mod2pi(t - phi)

            if t >= 0 and u <= 0 and v >= 0:
                if abs(t - abs(v) - phi) < EPS:  
                    return [SubPath(t, 'Left', 'Forward'), SubPath(math.pi / 2, 'Right', 'Backward'), SubPath(u, 'Straight', 'Backward'), SubPath(math.pi / 2, 'Left', 'Backward'), SubPath(v, 'Right', 'Forward')]

      #  return None




# class Collision_detection:
#     def __init__(self) -> None:
#         self.map = None
#         self.res = []

#     def Get_map(self, map: list, res:float = 0.05):
#         self.map = map
#         self.res = res
    
#     def collision_check(self, path:ReedShepp_Path):
#         for x, y in path:
#             if map[x, y] == 0:
#                 return False
#         return True

#     def cloest_grid(self, x, y):
#         return x, y
