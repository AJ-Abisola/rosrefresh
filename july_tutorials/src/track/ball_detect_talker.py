#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time

def main():

    video_file = "/video/tennis-ball-video.mp4"
    cap = cv2.VideoCapture(video_file)
    pub = rospy.Publisher('/tennis_ball_image', Image, queue_size=10)
    rospy.init_node('ball_detection_publisher', anonymous=True)
    rate = rospy.Rate(5) 
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()
    
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        frame = cv2.resize(frame, (320, 240))
        cv2.imshow("Original Image", frame)
        image_message = bridge.cv2_to_imgmsg(frame, "bgr8")

        pub.publish(image_message)
        if cv2.waitKey(10) == ord('q'):
            break
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        time.sleep(2)
        main()
    except rospy.ROSInterruptException:
        pass
