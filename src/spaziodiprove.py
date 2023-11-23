#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, hypot

class SimplePose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        rospy.wait_for_message("/odom", Odometry, 10)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
    def go_to(self, goal_point):
        speed : Twist = Twist()
        rho = 9999999999999999

        while rho > 0.2:
            delta_x = goal_point.x - self.x
            delta_y = goal_point.y - self.y
            rho = sqrt(delta_x**2 + delta_y**2)
            rospy.loginfo_throttle_identical(1, "Distance to goal= {}m".format(rho))
            angle_to_goal = atan2(delta_y, delta_x)

            if abs(angle_to_goal - self.theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.22
                speed.angular.z = 0.0
            self.pub.publish(speed)
            rospy.sleep(0.01)

    def stop_robot(self):
        speed : Twist = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pub.publish(speed)



if __name__ == '__main__':
    rospy.init_node("speed_controller")
    simple_pose_mover = SimplePose()

    goal = Point()
    goal.x = 5
    goal.y = 5
    
    try:
        simple_pose_mover.go_to(goal)
    except (KeyboardInterrupt, rospy.ROSException) as e:
        rospy.logerr(e)
    finally:
        simple_pose_mover.stop_robot()
        position_error=hypot(goal.x - simple_pose_mover.x, goal.y-simple_pose_mover.y)
        rospy.loginfo("Final position error: {}m".format(position_error))