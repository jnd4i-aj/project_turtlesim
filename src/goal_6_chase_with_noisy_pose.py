#!/usr/bin/env python3

from math import atan2, sqrt, pow, pi, sin, cos, degrees, radians
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import time
from common_utils import CommonFunctions
kp = 1
kd = 0.1
ki = 0.0001
b = 0
a = []

class turtlebot():
    
    def __init__(self):
        rospy.init_node('chasing', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.noisy_pose_subscriber = rospy.Subscriber('/rt_noisy_pose', Pose, self.plan)
        self.rate = rospy.Rate(10)
        self.pose_subscriber = rospy.Subscriber('/turtle2/pose', Pose, self.__posecallback)

        self.pose = Pose()
        self.__common_functions = CommonFunctions()
        self.max_vel = 1        

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
                vel_msgs.linear.x = (kp*(error) + kd * (D_error) + ki * (I_error))/2
                vel_msgs.linear.y = 0
                vel_msgs.linear.z = 0
                vel_msgs.angular.x = 0
                vel_msgs.angular.y = 0

                signed_angle_diff = angle_error - self.pose.theta

                if signed_angle_diff > pi:
                    signed_angle_diff -= 2 * pi
                elif signed_angle_diff < -pi:
                    signed_angle_diff += 2 * pi

                vel_msgs.angular.z = 6*(kp*signed_angle_diff)

                rospy.loginfo("Linear Velocity of PT is %f" , vel_msgs.linear.x)
                rospy.loginfo("Angular velocity of PT is %f ", vel_msgs.angular.z)
                last_vel,t_final = self.__common_functions.step_vel(self.velocity_publisher,
                                                                    vel_msgs,last_vel,t_final)

                self.velocity_publisher.publish(vel_msgs)
                self.rate.sleep()
                last_error = error
                last_angle_err = angle_error

    def __distance(self,point):
        return(sqrt(pow((self.pose.x-point[0]), 2) + pow((self.pose.y-point[1]), 2)))
    
    def __goal_distance(self,goal):
        # The goal distance is the distance between self and goal i.e. the location of RT
        return(sqrt(pow((self.pose.x-goal.x), 2) + pow((self.pose.y-goal.y), 2)))
    
    def __define_circle(self, p1, p2, p3):

        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        mid1 = ((x1 + x2) / 2, (y1 + y2) / 2)
        mid2 = ((x2 + x3) / 2, (y2 + y3) / 2)

        if y2 - y1 == 0:
            slope1 = np.inf
        else:
            slope1 = -1 / ((y2 - y1) / (x2 - x1))

        if y3 - y2 == 0:
            slope2 = np.inf
        else:
            slope2 = -1 / ((y3 - y2) / (x3 - x2))

        if slope1 == slope2:
            return None, np.inf

        cx = (slope1 * mid1[0] - slope2 * mid2[0] + mid2[1] - mid1[1]) / (slope1 - slope2)
        cy = slope1 * (cx - mid1[0]) + mid1[1]
        radius = np.sqrt((x1 - cx) ** 2 + (y1 - cy) ** 2)

        return (cx, cy), radius

    
    def __next_point(self,p2,p3,center,radius):
            theta2=atan2(p2[1]-center[1],p2[0]-center[0])
            theta3=atan2(p3[1]-center[1],p3[0]-center[0])
            dtheta=degrees(theta2-theta3)
            theta4=degrees(theta3)-dtheta
            nextval=([0,0])
            if(theta4<0):
                theta4=theta4+360
            nextval[0]=center[0]+radius*cos(radians(theta4))
            nextval[1]=center[1]+radius*sin(radians(theta4))
            return (nextval)
    

    def plan(self,target):
        # Here, target is the rt_real_pose

        rospy.loginfo("Current distance: %f ",self.__goal_distance(target))
        predicted_pos=Pose()
        if(np.shape(a)[0]<3):
            a.append([target.x,target.y])
        print(a)
        if(np.shape(a)[0]>=3):
            t1=time.time()
            center,radius=self.__define_circle(a[0],a[1],a[2])
            next_point=self.__next_point(a[1],a[2],center,radius)	
            a.append(next_point)
            plan_length=1
            while(True):
                print("Planning for {0} sec ahead:".format(plan_length*5))
                if(self.__distance(next_point)/self.max_vel>(5*plan_length)):
                    a.append(next_point)
                    rospy.loginfo("Estimated time: %f ",(self.__distance(next_point)/self.max_vel))
                    
                    plan_length=plan_length+1			
                
                else:
                    predicted_pos.x=next_point[0]			
                    predicted_pos.y=next_point[1]
                    print("Planned for pos:",predicted_pos)
                    self.__moveGoal(predicted_pos)
                    break


if __name__ == '__main__':
    try:
        time.sleep(10)
        x = turtlebot()
        rospy.spin()

    except rospy.ROSInterruptException: pass