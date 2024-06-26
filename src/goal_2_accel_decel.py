#!/usr/bin/env python3

from math import atan2, sqrt, pow, pi
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
from common_utils import CommonFunctions
kp = 1
kd = 0.1
ki = 0.0001

class turtlebot():
    
    def __init__(self):
        rospy.init_node('Make grid', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.__posecallback)

        self.pose = Pose()
        self.__common_functions = CommonFunctions()
        self.rate = rospy.Rate(10)

    def __posecallback(self, msg):
        self.pose = msg	

    def __moveGoal(self, goal):

        vel_msgs = Twist()
        last_vel = Twist()

        I_error = 0
        last_error = sqrt(pow((goal.x - self.pose.x), 2) + pow((goal.y - self.pose.y), 2))

        t_final = time.time()
        while(1):
            I_error = I_error + last_error
            error = sqrt(pow((goal.x - self.pose.x), 2) + pow((goal.y - self.pose.y), 2))
            D_error = error - last_error            

            vel_msgs.linear.x = kp*(error) + kd * (D_error) + ki * (I_error)
            vel_msgs.linear.y = 0
            vel_msgs.linear.z = 0
            vel_msgs.angular.x = 0
            vel_msgs.angular.y = 0
            rospy.loginfo("Error is %f ", error)

            vel_msgs.angular.z = 6*kp * ((atan2(goal.y - self.pose.y, goal.x - self.pose.x)) - self.pose.theta)
            last_vel,t_final = self.__common_functions.step_vel(self.velocity_publisher,
                                                                vel_msgs,last_vel,t_final)

            self.velocity_publisher.publish(vel_msgs)
            t_initial = t_final

            self.rate.sleep()
            last_error = error

            if(error<=0.3):
                vel_msgs.angular.z = 0
                vel_msgs.linear.x = 0
                self.velocity_publisher.publish(vel_msgs)
                rospy.loginfo("Goal Reached")
                break

    def __rotate(self, angle):

        vel_angle = Twist()
        while((abs((angle*pi/180)-(self.pose.theta))) > 0.03):
            vel_angle.angular.z=(angle*pi/180)-(self.pose.theta)
            self.velocity_publisher.publish(vel_angle)
    
    def grid(self):
        goal = Pose()
        grid_corner_points=[(1,1,0),(10,1,90),(10,2,180),(1,2,90),(1,4,0),(10,4,90),
                            (10,6,180),(1,6,90),(1,8,0),(10,8,90),(10,10,180),(1,10,0)]
        for i in range(len(grid_corner_points)):
            goal.x=grid_corner_points[i][0]
            goal.y=grid_corner_points[i][1]
            self.__moveGoal(goal)
            self.__rotate(grid_corner_points[i][2])


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            x = turtlebot()
            x.grid()

    except rospy.ROSInterruptException: pass