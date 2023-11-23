#!/usr/bin/env python3 
import rospy

import actionlib

import subprocess

from rospkg import RosPack

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

def news(msg):
	msg = MoveBaseActionFeedback() 
	rospy.loginfo(msg.status.status)
	if msg.status.status == 3:
		
		rospy.loginfo("Goal is reached!")
		execfile('rospy_img_sub_example.py')
	else:
		rospy.loginfo("Goal is not reached!")

def reach_goal():

	reach = actionlib.SimpleActionClient('reach_goal', MoveBaseAction)
	reach.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = 7.0
	goal.target_pose.pose.position.y = 0.3
	goal.target_pose.pose.orientation.w = 0.7
	goal.target_pose.pose.orientation.z = -0.7
	
	reach.send_goal(goal, news_cb=news)

	tempo = reach.wait_for_result()
	
	if not tempo :
		rospy.logerr("Not available")
		rospy.signal_shutdown("Not available")
	else:
		return reach.get_result()

if __name__=='__main__':
	try:
		rospy.init_node('move_robot')
		result = reach_goal()
		if result:
			rospy.loginfo("Result= {}".format(result))
			rospy.loginfo("Maze is solved!")
			package_path = RosPack().get_path("fhtw_line_follower")
			bash_path = package_path+"/scripts/run_project.sh"
			a = subprocess.Popen(bash_path, stdout=subprocess.PIPE, shell=True)
			(output, err) = a.communicate()
			rospy.loginfo("Output={}\nerr={}".format(output, err))
	except rospy.ROSInterruptException:
		rospy.loginfo("Maze is not solved!") 