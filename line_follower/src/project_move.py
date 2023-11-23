#!/usr/bin/env python3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import rospy
import actionlib
import subprocess
from rospkg import RosPack


def info(inp):
	msg = MoveBaseActionFeedback() 
	rospy.loginfo(inp.status.status)
	if inp.status.status == 3:
		rospy.loginfo("Goal is reached!")
	else:
		rospy.loginfo("Goal is not reached!")

def move_base():

	guide= actionlib.SimpleActionClient('move_base', MoveBaseAction)
	guide.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = 7.0
	goal.target_pose.pose.position.y = 0.3
	goal.target_pose.pose.orientation.w = 0.7
	goal.target_pose.pose.orientation.z = -0.7
	
	guide.send_goal(goal, feedback_cb=info)

	wait = guide.wait_for_result()
	
	if not wait:
		rospy.logerr("Not available")
		rospy.signal_shutdown("Not available")
	else:
		return guide.get_result()

if __name__=='__main__':
	try:
		rospy.init_node('move_robot')
		result = move_base()
		if result:
			rospy.loginfo("Result= {}".format(result))
			rospy.loginfo("Labirint in done!")
			path_firstpkg = RosPack().get_path("first_pkg")
			bash = path_firstpkg+"/scripts/run_project.sh"
			k = subprocess.Popen(bash, stdout=subprocess.PIPE, shell=True)
			(output, err) = k.communicate()
			rospy.loginfo("Output={}\nerr={}".format(output, err))
	except rospy.ROSInterruptException:
		rospy.loginfo("Labirint is not solved!")