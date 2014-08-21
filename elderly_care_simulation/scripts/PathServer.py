#!/usr/bin/env python

import roslib; roslib.load_manifest('elderly_care_simulation')
import sys
import rospy
from geometry_msgs.msg import Point
from elderly_care_simulation import *

def find_path(req):
    from_point = req.from_point;
    to_point = req.to_point;
    pass


def find_path_server():
    rospy.init_node('find_path_server')
    service = rospy.Service('find_path', find_path)
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) < 1:
        rospy.logwarn("No world file given")
        return

    find_path_server()