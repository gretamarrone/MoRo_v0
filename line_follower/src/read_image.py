#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from rospkg import RosPack
from geometry_msgs.msg import Twist
from move_robot import MoveRobot


package_path = RosPack().get_path("fhtw_line_follower")
image_path = package_path+"/Lane.png" 

# Take each frame
frame = cv2.imread(image_path)

height, width, channels = frame.shape

# Convert BGR to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# define range of color in HSV
#blue
lower = np.array([20,100,100])
upper = np.array([30,255,255])

# Threshold the HSV image to get only colors
mask = cv2.inRange(hsv, lower, upper)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(frame,frame, mask= mask)
""" 
cv2.imshow('frame',frame)
cv2.imshow('mask',mask)
cv2.imshow('res',res)
cv2.waitKey(0) """

M = cv2.moments(mask)
# calculate x,y coordinate of center
cX = int(M["m10"] / M["m00"])
cY = int(M["m01"] / M["m00"])
 
# put text and highlight the center
cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
#cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
 
error_x = cX - width / 2
twist_object = Twist()
twist_object.linear.x = 0.2
twist_object.angular.z = -error_x / 100

rospy.init_node('line_following_node')
moverobot_object = MoveRobot()

moverobot_object.move_robot(twist_object)

# display the image
cv2.imshow("Image", frame)
cv2.waitKey(0)
