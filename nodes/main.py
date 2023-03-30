#!/usr/bin/env python3

import numpy as np
import rospkg
import rospy

from RS import *


if __name__ == '__main__':

    rospy.init_node("RS")

    RS_path = ReedShepp_Path(constraint=1.0)

    # Algorithm test
    rospy.loginfo("Find Collision free RS path")
    RS_path.find_path()

    rospy.spin()










