#!/usr/bin/env python
#
# This program defines the twist and accelerator control
# Actual velocity is calculated by camera using ORB_SLAM2
# Reads previously created map

# Fuzzy controller for turtlebot
'''
This code is to integrate turtleTests.py and fuzzyController_V0.0.py
'''

import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Twist, Pose
from std msg.msg import Int16

os_pos_x = 0.0
os_pos_y = 0.0
os_theta = 0.0

time_stamp = 0.0
last_time_stamp = 0.0
delta_time = 0.1
speed_d = 0
last_q = Pose()

os_state = 1

q_ = Pose()
gamma = Twist()


def callback_pose(data):
    global os_pos_x
    global os_pos_y
    global os_theta
    global time_stamp
    global q_
    q_ = data.pose
    # os_pos_x = data.pose.position.x
    # os_pos_x = data.pose.position.y
    # os_theta = data.pose.orientation.y
    time_stamp = data.header.stamp.nsecs

def callback_state(data):
    global os_state
    os_state= data.data


def getfile():
    rfile = open("KeyFrameTrajectory.txt", "r")  # open the text file that contains all the track points data
    rfile = rfile.read()                            # rfile.read() opens the txt file to be read
    rfile = rfile.split("\n")                       # separates the text of the file every enter
    values = []                                     # initialize values array
    fileRows = len(rfile)                           # gets the numer of rows in the file
    count = 0                                       # auxiliary counter for getting just the floats on the file and not strings
    for item in rfile:                              # iteration that firs divides all the text file in rows
        buffer = []                                 # and then spaces different values divided by spaces
        if count == fileRows - 1:                   # breaking the for loop when it reaches the end of the file length
            break                                   #
        elif count == 0:                            # for deleting the headers of the txt file
            print("Program Running... ")            #
        else:                                       #
            for element in item.split(" "):         #
                buffer.append(element)              # appends every number to the current matrix row
                buffer = [float(i) for i in buffer] # converts the string type data to float on the buffer array
            buffer.pop(0)
            values.append(buffer)
        count += 1
    return (values)
