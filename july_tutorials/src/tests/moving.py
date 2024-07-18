#!/usr/bin/env python

import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

# x_pos = 0

#subscriber's callback for Pose
def callback(data):
    global x_pos
    x_pos = data.x


def straight_line(publisher, speed:int, distance:int, direction:bool):

    global x_pos
    start_pos = x_pos
    rate = rospy.Rate(50) # 10hz
    twist = Twist()

    if direction == False:
        twist.linear.x = -abs(speed)
        end_pos = x_pos - distance
    else:
        twist.linear.x = abs(speed)
        end_pos = x_pos + distance
    print(f"Starting position: {start_pos}")

    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()
        print(f"Current position: {x_pos}")
        print(f"Current speed: {speed} m/s \n")
        

        if abs(x_pos - start_pos) > distance: 
            print(f"End point {round(end_pos, 2)} reached")
            twist.linear.x = 0
            pub.publish(twist)
            break


def spiral(publisher, linear_speed, angular_turn):

    twist = Twist()
    loop_rate = rospy.Rate(1)

    while True:
        
        twist.linear.x = linear_speed
        twist.angular.z = angular_turn
        pub.publish(twist)
        loop_rate.sleep()
        linear_speed = linear_speed + 0.2

        



if __name__ == '__main__':
    try:
        rospy.init_node('robot', anonymous=True)
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/turtle1/pose", Pose, callback)
        time.sleep(2)


        # straight_line(pub,0.5, 2, True)
        spiral(pub,1,2)
    except rospy.ROSInterruptException:
        pass