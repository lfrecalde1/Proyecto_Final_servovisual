#!/usr/bin/env python3.6
from controller import *
import cv2 
import time
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Declaracion de las variables de comuinicacion 
vx=0
vy=0
vz=0
wx=0
wy=0
wz=0

def velocityCallback(velocity_message):
    global vx
    global vy
    global vz
    global wx
    global wy
    global wz

    vx=velocity_message.linear.x
    vy=velocity_message.linear.y
    vz=velocity_message.linear.z
    wx=velocity_message.angular.x
    wy=velocity_message.angular.y
    wz=velocity_message.angular.z
def bucle():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    loop_rate=rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.loginfo("Escuchando velocidades")
        print ('x = {}'.format(vx)) #new in python 3
        print ('y = {}' .format(vy)) #used in python 2
        print ('yaw = {}'.format(vz)) #new in python 3
        loop_rate.sleep()




