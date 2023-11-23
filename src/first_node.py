#!/usr/bin/python3

import rospy

if __name__ == '__main__':
    rospy.init_node("Hagrid")
    rate = rospy.Rate(10)              # We create a Rate object of 10Hz
    while not rospy.is_shutdown():     # Endless loop until Ctrl + C
       print("You're a wizard, Harry")
       rate.sleep()                    # We sleep the needed time to maintain the Rate fixed above

    # This program creates an endless loop, while the roscore (more about this later) is not shutdown, that repeats itself 10 times per second (10Hz) until somebody presses Ctrl + C
    # in the Shell or kills the node using the ROS CLI (rosnode kill /Hagrid)
