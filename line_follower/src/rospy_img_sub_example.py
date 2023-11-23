#!/usr/bin/env python
# Use this node in combination with the play_rosbag.launch file, which loads a recorded drive over the lane you need to follow
# or/later on with the live image of the simulated robot


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class LineFollower(object):
    
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_object = Twist()


    def image_callback(self, msg):
        cv_image = None  # create empty variable of no spezific type

        # OpenCV uses a different definition of an image, therefore we need to convert it
        try:
            crop_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerror(e)

        height, width, channels = crop_frame.shape

        # Crop the image 
        #descentre = 220
        #rows_to_watch = 20
        #crop_frame = frame[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

        # Convert BGR to HSV
        hsv = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV)
        # define range of color in HSV
        #blue
        lower = np.array([20,100,100])
        upper = np.array([30,255,255])

        # Threshold the HSV image to get only colors
        mask = cv2.inRange(hsv, lower, upper)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_frame,crop_frame, mask= mask)

        search_top = 3*height/4
        search_bot = 3*height/4 + 20
        mask[0:search_top, 0:width] = 0
        mask[search_bot:height, 0:width] = 0

        M = cv2.moments(mask)
        try:
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # put text and highlight the center
            cv2.circle(crop_frame, (cX, cY), 5, (255, 255, 255), -1)
            #cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            error_x = cX - width / 2
            self.twist_object.linear.x = 0.2
            self.twist_object.angular.z = -error_x *0.01
            #if 2.84
            #self.twist_object.angular.z = -error_x *0.01775
            self.cmd_vel_pub.publish(self.twist_object)
            rospy.loginfo(error_x)
            rospy.loginfo("Angular: {}m".format(self.twist_object.angular.z))
        except ZeroDivisionError: # When no line is found - Recovery Behaviour
            rospy.loginfo("No line is found")
            self.twist_object = Twist()
            self.twist_object.angular.z = 1
            self.cmd_vel_pub.publish(self.twist_object)
        
        # Display the opencv image
        #cv2.imshow("Image stream", crop_frame)
        #cv2.waitKey(0)


if __name__ == "__main__":
    rospy.init_node("cv_bridge_example")
    line_follower = LineFollower()

    rospy.spin()
