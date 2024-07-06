#!/usr/bin/env python

from __future__ import print_function

from july_tutorials.srv import rectangle,rectangleResponse
import rospy

def find_area(req):
    print("Returning [%s + %s = %s]"%(req.width, req.height, (req.width * req.height)))
    return rectangleResponse(req.width * req.height)

def area_server():
    rospy.init_node('find_area_server')
    s = rospy.Service('rectangle_area', rectangle, find_area)
    print("Ready to find the area.")
    rospy.spin()

if __name__ == "__main__":
    area_server()