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

class ChaseRT():
    
    def __init__(self):
        rospy.init_node('chasing RT', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.real_pose_subscriber = rospy.Subscriber('/rt_real_pose', Pose, self.chase)
        self.rate = rospy.Rate(10)
        self.pose_subscriber = rospy.Subscriber('/turtle2/pose', Pose, self.__posecallback)

        self.__common_functions = CommonFunctions()
        self.pose = Pose()

    def __posecallback(self, msg):
        self.pose = msg

    def __moveGoal(self, target):

        vel_msgs = Twist()
        last_vel = Twist()
        I_error = 0
        last_error = sqrt(pow((self.pose.x-target.x), 2) + pow((self.pose.y - target.y), 2))
        last_angle_err = 0
        i_angle_err = 0
        t_final = time.time()

        while(not rospy.is_shutdown()):
            I_error = I_error + last_error
            rospy.loginfo("I Error is %f ", I_error)
            error = sqrt(pow((self.pose.x-target.x), 2) + pow((self.pose.y - target.y), 2))
            rospy.loginfo("Error between current pose and the target pose %f", error)
            D_error = error - last_error
            angle_error = atan2(target.y - self.pose.y, target.x - self.pose.x)

            if angle_error > pi:
                angle_error -= 2 * pi
            elif angle_error < -pi:
                angle_error += 2 * pi

            d_angle_err = angle_error - last_angle_err
            i_angle_err = i_angle_err + last_angle_err
            rospy.loginfo("Angle Error %f", angle_error)
            rospy.loginfo("Derivative angle error %f", d_angle_err)
            # I_error = I_error + error

            if(error<=0.3):
                vel_msgs.linear.x = 0
                vel_msgs.angular.z = 0
                self.velocity_publisher.publish(vel_msgs)
                I_error = 0
                rospy.loginfo("Goal Reached")
                break
            else:
                if I_error > 50:
                    I_error = 0
                vel_msgs.linear.x = 4*(kp*(error) + kd * (D_error) + ki * (I_error))
                vel_msgs.linear.y = 0
                vel_msgs.linear.z = 0
                vel_msgs.angular.x = 0
                vel_msgs.angular.y = 0

                signed_angle_diff = angle_error - self.pose.theta

                if signed_angle_diff > pi:
                    signed_angle_diff -= 2 * pi
                elif signed_angle_diff < -pi:
                    signed_angle_diff += 2 * pi

                vel_msgs.angular.z = 8*(kp*signed_angle_diff + (kd - 0.1)*d_angle_err +\
                                           ki * i_angle_err)

                rospy.loginfo("Linear Velocity of PT is %f" , vel_msgs.linear.x)
                rospy.loginfo("Angular velocity of PT is %f ", vel_msgs.angular.z)
                last_vel,t_final = self.__common_functions.step_vel(self.velocity_publisher,
                                                                    vel_msgs,last_vel,t_final)

                self.velocity_publisher.publish(vel_msgs)
                self.rate.sleep()
                last_error = error
                last_angle_err = angle_error

    def chase(self,target_pose):
        # function to chase the Robber Turtle which continuosly subscribe
        # to topic "/rt/real/pose" and check the distance between PT and RT

        target_pose_log =', '.join(map(str, [target_pose.x, target_pose.y, target_pose.theta,
                          target_pose.linear_velocity, target_pose.angular_velocity]))

        rospy.loginfo("Target Pose for PT is %s   ", target_pose_log)

        error = sqrt((self.pose.x-target_pose.x)**2+(self.pose.y-target_pose.y)**2)

        rospy.loginfo("Current distance is %f ",error)
        # time.sleep(1)

        if(error <= 0.5):
            rospy.loginfo("Distance is less than 0.5 units. Caught Robber Turtle \n")
            self.real_pose_subscriber.unregister()
            rospy.set_param('caughtStatus',True)
        else:
            rospy.loginfo("Chasing RT")
            self.__moveGoal(target_pose)

 

if __name__ == '__main__':
    try:
        time.sleep(10)
        x = ChaseRT()
        rospy.spin()


    except rospy.ROSInterruptException: pass