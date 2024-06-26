#! /usr/bin/env python3


import rospy
import rosservice
import random
from math import pi
import time


# Spawning PT turtle from a random location
x = random.random()*10
y = random.random()*10
theta = random.random()*pi

rospy.wait_for_service('/spawn')
print("Waiting for 10 seconds before spawning PT!")
# Spawnning PT after 10 seconds RT is launched
time.sleep(10)
rosservice.call_service("/spawn",[x, y, theta, ""])