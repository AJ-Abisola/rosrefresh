#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from july_tutorials.srv import *

def area_client(x, y):
    rospy.wait_for_service('rectangle_area')
    try:
        get_area = rospy.ServiceProxy('rectangle_area', rectangle)
        resp1 = get_area(x, y)
        return resp1.area
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, area_client(x, y)))