#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
from common_utils import CommonFunctions
kp = 1
kd = 0.1
ki = 0.0001

class RotateInCircleOfRadiusFive():
    
    def __init__(self):
        rospy.init_node('Rotating in a Circle', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.real_pose_publisher = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)
        self.noisy_pose_publisher = rospy.Publisher('/rt_noisy_pose', Pose, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.__posecallback)

        self.__common_functions = CommonFunctions()
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def __posecallback(self, msg):
        self.pose = msg

    def circles(self, radius, velocity):
        new_vel_msg = Twist()
        last_vel = Twist()
        new_vel_msg.linear.x = velocity
        new_vel_msg.angular.z = velocity/radius
        t_initial = time.time()
        while(not rospy.is_shutdown()):
            t_final = time.time()
            last_vel,t_final = self.__common_functions.step_vel(self.velocity_publisher,
                                                                new_vel_msg,
                                                                last_vel,t_final)
            self.rate.sleep()
            if(time.time() - t_initial >= 5):
                self.real_pose_publisher.publish(self.pose)
                real_pose = ', '.join(map(str, [self.pose.x, self.pose.y, self.pose.theta,
                        self.pose.linear_velocity, self.pose.angular_velocity]))
                rospy.loginfo("Publishing real pose %s", real_pose)
                t_initial = time.time()
                self.__common_functions.gaussian_noise(self.noisy_pose_publisher, self.pose)

            caught_status = rospy.get_param('/caughtStatus')
            rospy.loginfo("Caught Status of Robber Turtle is %s", caught_status)
            if caught_status == True:
                new_vel_msg.linear.x = 0
                new_vel_msg.angular.z = 0
                self.velocity_publisher.publish(new_vel_msg)
                break
 
if __name__ == '__main__':
    try:
         radius = 5
         velocity = 7
         RotateInCircleOfRadiusFive().circles(radius, velocity)

    except rospy.ROSInterruptException: pass