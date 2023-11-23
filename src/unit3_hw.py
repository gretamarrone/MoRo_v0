#!/usr/bin/python3


import rospy
from numpy import array
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, hypot
   

class SimplePose:
    def __init__(self) :
        self.x = 0.00
        self.y = 0.00
        self.theta = 0.00
        self.goal_tolerance = 0.010  #When the robot  get close to the goal, it slow down.

        self.max_vel=0.40
        self.max_omega = 2.84
         
        #The next are the wheel left and right,if we change the number ,the robot chage the direction.
        self.k_rho = 0.40     
        self.k_alpha = 0.90      
        

        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # we wait for a message on the odom topic to make sure the simulation is started
        rospy.wait_for_message("/odom", Odometry, 10)

    def odom_callback(self, msg) :
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
    def go_to(self, goal_point) :
    
        speed = Twist()
        rho = float("inf")

        while rho > self.goal_tolerance:
            delta_x = goal_point.x - self.x
            delta_y = goal_point.y - self.y
            rho = sqrt(delta_x**2 + delta_y**2)
            rospy.loginfo_throttle_identical(1,"Distance to goal= {}m".format(rho))
            angle_to_goal = atan2(delta_y, delta_x)

            alpha = angle_to_goal - self.theta

            v = self.k_rho * rho            #calculation of linear velocity
            omega = self.k_alpha * alpha    #angular velocity

            speed.linear.x = v if v <= self.max_vel else self.max_vel
            speed.angular.z = omega if omega <= self.max_omega else self.max_omega
            self.pub.publish(speed)
            rospy.sleep(0.01)

              
    def stop_robot(self) :#-> None:
        speed = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pub.publish(speed)


if __name__=='__main__':

    rospy.init_node("speed controller")
    simple_pose_mover = SimplePose()

     

    goal_points = array([[1,1],[2,2],[3,3],[4,4],[5,5],[6,6]])
    dim = goal_points.shape
    n = int(dim[0])
    
    for i in range (n) : 
        
   
        goal = Point()
        goal.x = int(goal_points[i][0])
        goal.y = int(goal_points[i][1])

        try:
            simple_pose_mover.go_to(goal)
        except(KeyboardInterrupt,rospy.ROSException) as e:
            rospy.logerr(e)
        
      
    
    simple_pose_mover.stop_robot()
    position_error=hypot(goal.x - simple_pose_mover.x,goal.y-simple_pose_mover.y)
    rospy.loginfo("Final position error: {}m".format(position_error))

