#!/usr/bin/env python3
import cv2

import rospy

import numpy as np

from rospkg import RosPack

from geometry_msgs.msg import Twist

from move_robot import MoveRobot


package_path = RosPack().get_path("fhtw_line_follower")
image_path = package_path+"/Lane.png" 


frame = cv2.imread(image_path)

height, width, channels = frame.shape


hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


lower = np.array([20,100,100])
upper = np.array([30,255,255])


mask = cv2.inRange(hsv, lower, upper)


res = cv2.bitwise_and(frame,frame, mask= mask)
""" 
cv2.imshow('frame',frame)
cv2.imshow('mask',mask)
cv2.imshow('res',res)
cv2.waitKey(0) """

M = cv2.moments(mask)

cX = int(M["m10"] / M["m00"])
cY = int(M["m01"] / M["m00"])
 

cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)

 
error_x = cX - width / 2
twist_object = Twist()
twist_object.linear.x = 0.2
twist_object.angular.z = -error_x / 100

rospy.init_node('line_following_node')
moverobot_object = MoveRobot()

moverobot_object.move_robot(twist_object)


cv2.imshow("Image", frame)
cv2.waitKey(0)