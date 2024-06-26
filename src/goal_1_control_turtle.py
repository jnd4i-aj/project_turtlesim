#!/usr/bin/env python3

from math import atan2, sqrt, pow
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
kp = 2
kd = 1
ki = 0.0001

class turtlebot():
    
    def __init__(self):
        rospy.init_node('Go to goal', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.__posecallback)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def __posecallback(self, msg):
        self.pose = msg

    def moveGoal(self):

        vel_msgs = Twist()

        goal_pose = Pose()                                     # goal_pose = Goal Pose

        goal_pose.x = float(input('Enter Goal X coordinate '))
        goal_pose.y = float(input('Enter Goal Y coordinate '))

        I_error = 0
        I_angle_error = 0
        last_error = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
        last_angle_error =((atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)) - self.pose.theta)

        while(1):
            I_error = I_error + last_error
            error = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            D_error = error - last_error            

            vel_msgs.linear.x = kp*(error) + kd * (D_error) + ki * (I_error)
            vel_msgs.linear.y = 0
            vel_msgs.linear.z = 0
            vel_msgs.angular.x = 0
            vel_msgs.angular.y = 0
            rospy.loginfo("Error is %f ", error)

            angle_error = ((atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)) - self.pose.theta)
            D_angle_error = angle_error - last_angle_error
            vel_msgs.angular.z = 6*kp * (angle_error) + kd * (D_angle_error) + ki * I_angle_error

            self.velocity_publisher.publish(vel_msgs)

            self.rate.sleep()
            last_error = error
            last_angle_error = angle_error
            I_angle_error = I_angle_error + last_angle_error

            if(error<=0.3):
                vel_msgs.angular.z = 0
                vel_msgs.linear.x = 0
                self.velocity_publisher.publish(vel_msgs)
                rospy.loginfo("Goal Reached")
                break

 
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            x = turtlebot()
            x.moveGoal()

    except rospy.ROSInterruptException: pass