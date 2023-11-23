#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as numpy


class Unit2_hw:
    def __init__(self):
        rospy.init_node("Unit2_hw")
        self.sub_scan = rospy.Subscriber("/scan", data_class=LaserScan, callback=self.callback_scan)
        self.sub_odom = rospy.Subscriber("/odom", data_class=Odometry, callback=self.callback_odom)
        self.pub = rospy.Publisher("/n_smallest", Float32MultiArray, queue_size=10)
        self.n_smallest = 5 #Number of smallest points to find

    @staticmethod
    def callback_odom(msg):
        rospy.logdebug(msg)

    def callback_scan(self, msg):
        ranges = numpy.asarray(msg.ranges)

        idx = numpy.argpartition(ranges, self.n_smallest)
        pub_msg = Float32MultiArray(data=ranges[idx[:self.n_smallest]])
        self.pub.publish(pub_msg)


Hw_2= Unit2_hw()

while not rospy.is_shutdown():
    rospy.Rate(1).sleep()

