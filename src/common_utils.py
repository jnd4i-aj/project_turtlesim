#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import numpy as np


class CommonFunctions():

    def __init__(self) -> None:
        pass

    def gaussian_noise(self, noisy_pose_publisher, pose):
        # Pose Published with Gaissian Noise
        # Mean = 0
        # Standard Deviation = 10 units 

        noisy_pose = Pose()
        noisy_pose.x = pose.x + np.random.normal(0,10)
        noisy_pose.y = pose.y + np.random.normal(0,10)
        noisy_pose.theta = pose.theta + np.random.normal(0,0.4)
        noisy_pose.linear_velocity = pose.linear_velocity + np.random.normal(0,0.4)
        noisy_pose.angular_velocity = pose.angular_velocity + np.random.normal(0,0.4)
        noisy_pose_list =', '.join(map(str, [noisy_pose.x, noisy_pose.y, noisy_pose.theta,
                          noisy_pose.linear_velocity, noisy_pose.angular_velocity]))
        rospy.loginfo("Publishing pose with Gaussian Noise %s", noisy_pose_list)
        noisy_pose_publisher.publish(noisy_pose)

    def step_vel(self,velocity_publisher, final_vel, initial_vel, time_sent):
        K_accel = 0.1
        K_decel = 0.3
        max_linear_accel = 2000
        max_linear_decel = -2000
        max_angular_accel = 5500
        max_angular_decel = -5500
        
        if(final_vel.linear.x-initial_vel.linear.x > 0):
            k = K_accel
            max_linear_steps = max_linear_accel
            max_angular_steps = max_angular_accel

        else:
            k = K_decel
            max_linear_steps = max_linear_decel
            max_angular_steps = max_angular_decel

        step_vel = Twist()
        step = k*(final_vel.linear.x - initial_vel.linear.x)
        time_now = time.time()
        dt = time_now - time_sent
        
        if(abs(step/dt) > abs(max_linear_steps)):
            rospy.loginfo("Task completed: Linear")
            step = dt*max_linear_steps

        stepangular = 4*k*(final_vel.angular.z - initial_vel.angular.z)

        if(abs(stepangular/dt) > abs(max_angular_steps)):
            rospy.loginfo("Task completed: Angular")
            stepangular = dt*max_angular_steps
        initial_vel.linear.x = initial_vel.linear.x + step
        initial_vel.angular.z = initial_vel.angular.z + stepangular
        velocity_publisher.publish(initial_vel)
        timer = time.time()
        return(initial_vel,timer)