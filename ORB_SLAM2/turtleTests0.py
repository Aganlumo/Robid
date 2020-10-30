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
from std_msgs.msg import Int16

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


# os_pos_x = 0.0
# os_pos_y = 0.0
# os_theta = 0.0

time_stamp = 0.0
last_time_stamp = 0.0
delta_time = 0.1
speed_d = 0
last_q = Pose()

os_state = 1

q_ = Pose()
gamma = Twist()

q_.position.x = 1.0
q_.position.y = 1.0

def callback_pose(data):
    # global os_pos_x
    # global os_pos_y
    # global os_theta
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


# Function in charge of reading an extern file that cotains the data of the map
# this data is fill with coordinates x,y and angle, also it cotains the time_stamp variable
# this function also puts this info into a buffer
# @return values its a buffer of frames from the whole map

# this method converts the txt file to a matrix float (nested lists) for data usage
# and comparing values
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

# Function incharge of finding the nearest key frame
# @param values is the array of data

def nearKeyFrame(values):
    global gamma    # Is the value of the stearing wheel
    # global q_acc    # Is the value of the accelerator pedal
    # global q_brake  # Is the value of the brake pedal
    global q_
    n=0
    global last_time_stamp  # Is the time enlapse
    for b in values:
        if type(b)==type([]):
            n+=1
    global frame_id
    error_list = []
    condition = 0
    print('Finding in route...')
    gamma.angular.z = 0.0
    gamma.linear.x = 0.0
    pub_twist_msg = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # pub_acc = rospy.Publisher('Control/accelerator', Int16, queue_size=10)
    # pub_brk = rospy.Publisher('Control/brakes', Int16, queue_size=10)
    pub_twist_msg.publish(gamma)
    # pub_acc.publish(q_acc)
    # pub_brk.publish(q_brake)
    time.sleep(1)
    while condition != 1 or not rospy.core.is_shutdown():
        rospy.Subscriber('ORB_SLAM2/pose', PoseStamped, callback_pose)
        while frame_id < n - 1 and q_.position.x != 1 and q_.position.y != 1:
            x_coordinate = values[frame_id][0]
            y_coordinate = values[frame_id][1]
            error_x = q_.position.x - x_coordinate
            error_y = q_.position.y - y_coordinate
            error_d = math.sqrt(math.pow(error_x, 2) + math.pow(error_y, 2))
            error_list.append(error_d)
            frame_id += 1
        if last_time_stamp != time_stamp:
            last_time_stamp = time_stamp
            minValue = min(error_list)
            frame_id = error_list.index(minValue)
            return frame_id
        else:
            frame_id = 0
            error_list = []


if __name__ == '__main__':
    last_error_theta = 0.0
    last_error_speed = 0.0
    next_frame_stop = 35
    on_curve = False
    speed_error = 0.0
    output_gamma = 0.0
    output_brk = 0.0
    output_acc = 0.0

    angle = ctrl.Antecedent(np.arange(-1, 1, 0.1), 'angle')
    distance = ctrl.Antecedent(np.arange(0, 6.5, 0.1), 'distance')

    out_right = ctrl.Consequent(np.arange(0, 6, 0.1), 'out_right')
    out_left = ctrl.Consequent(np.arange(0, 6, 0.1), 'out_left')

    # [left vertex, up, right vertex] for triangular function in controller
    # Triangular functions for angle input
    angle['N'] = fuzz.trimf(angle.universe, [-2, -1, 0]) # N -> Negative
    angle['Z'] = fuzz.trimf(angle.universe, [-1, 0, 1]) # Z -> Zero
    angle['P'] = fuzz.trimf(angle.universe, [0, 1, 2]) # P -> Positive

    # Triangular functions for distance input
    distance['Z'] = fuzz.trimf(distance.universe, [-3.25, 0, 3.25]) # Z -> Zero
    distance['M'] = fuzz.trimf(distance.universe, [0, 3.25, 6.5]) # M -> Mid
    distance['F'] = fuzz.trimf(distance.universe, [3.25, 6.5, 9.75]) # F -> Far

    # Triangular functions right motor output
    out_right['S'] = fuzz.trimf(out_right.universe, [-3, 0, 3]) # s -> slow
    out_right['M'] = fuzz.trimf(out_right.universe, [0, 3, 6]) # M -> mid
    out_right['F'] = fuzz.trimf(out_right.universe, [3, 6, 9]) # F -> Fast

    # Triangular functions left motor output
    out_left['S'] = fuzz.trimf(out_left.universe, [-3, 0, 3])
    out_left['M'] = fuzz.trimf(out_left.universe, [0, 3, 6])
    out_left['F'] = fuzz.trimf(out_left.universe, [3, 6, 9])

    # Rule set for right motor
    rule1r = ctrl.Rule(angle['N'] & distance['F'], out_right['M'])
    rule2r = ctrl.Rule(angle['N'] & distance['M'], out_right['M'])
    rule3r = ctrl.Rule(angle['N'] & distance['Z'], out_right['S'])
    rule4r = ctrl.Rule(angle['Z'] & distance['F'], out_right['F'])
    rule5r = ctrl.Rule(angle['Z'] & distance['M'], out_right['M'])
    rule6r = ctrl.Rule(angle['Z'] & distance['Z'], out_right['S'])
    rule7r = ctrl.Rule(angle['P'] & distance['F'], out_right['F'])
    rule8r = ctrl.Rule(angle['P'] & distance['M'], out_right['F'])
    rule9r = ctrl.Rule(angle['P'] & distance['Z'], out_right['F'])

    # Rule set for left motor
    rule1l = ctrl.Rule(angle['N'] & distance['F'], out_left['F'])
    rule2l = ctrl.Rule(angle['N'] & distance['M'], out_left['F'])
    rule3l = ctrl.Rule(angle['N'] & distance['Z'], out_left['M'])
    rule4l = ctrl.Rule(angle['Z'] & distance['F'], out_left['F'])
    rule5l = ctrl.Rule(angle['Z'] & distance['M'], out_left['M'])
    rule6l = ctrl.Rule(angle['Z'] & distance['Z'], out_left['S'])
    rule7l = ctrl.Rule(angle['P'] & distance['F'], out_left['M'])
    rule8l = ctrl.Rule(angle['P'] & distance['M'], out_left['M'])
    rule9l = ctrl.Rule(angle['P'] & distance['Z'], out_left['S'])

    right_ctrl = ctrl.ControlSystem([rule1r, rule2r, rule3r, rule4r, rule5r, rule6r, rule7r, rule8r, rule9r])
    left_ctrl = ctrl.ControlSystem([rule1l, rule2l, rule3l, rule4l, rule5l, rule6l, rule7l, rule8l, rule9l])

    right = ctrl.ControlSystemSimulation(right_ctrl)
    left = ctrl.ControlSystemSimulation(left_ctrl)

    rospy.init_node('listener', anonymous=True)
    frame_id = 0
    values = getfile()
    frame_id = nearKeyFrame(values)

    pub_twist_msg = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub_stop = rospy.Publisher('Control/stopRoutine', Int16, queue_size=10)
    rate = rospy.Rate(10)

    gamma.linear.y = 0
    gamma.linear.z = 0
    gamma.angular.x = 0
    gamma.angular.y = 0

    while not rospy.core.is_shutdown():
        # Ctrl + C shuts program down
        rospy.Subscriber('ORB_SLAM2/pose', PoseStamped, callback_pose)
        rospy.Subscriber('ORB_SLAM2/Camera_State', Int16, callback_state)

        desired_x = values[frame_id][0]
        desired_y = values[frame_id][1]

        error_x = desired_x - q_.position.x # error en componente x en m
        error_y = desired_y - q_.position.y # error en componente y en m
        error_d = math.sqrt(error_x**2 + error_y**2)
        x_prime = error_x*math.cos(q_.orientation.y*math.pi/180) - error_y*math.sin(q_.orientation.y*math.pi/180)
        y_prime = error_x*math.sin(q_.orientation.y*math.pi/180) + error_y*math.cos(q_.orientation.y*math.pi/180)
        theta = -math.atan(x_prime/y_prime)*180/math.pi

        if last_time_stamp != time_stamp:
            # print 'Last time stamp: ', last_time_stamp
            if last_time_stamp > time_stamp:
                delta_time = (time_stamp + 10**9) - last_time_stamp
            else:
                delta_time = (time_stamp - last_time_stamp) #conversion to seconds
            # print(delta_time)
            # print(' ')
            speed_x = (3.6*(q_.position.x-last_q.position.x)/(delta_time))*10**9 # m/s
            speed_y = (3.6*(q_.position.y-last_q.position.y)/(delta_time))*10**9 # m/s
            speed_d = math.sqrt(speed_x**2 + speed_y**2)
            last_q = q_
            last_time_stamp = time_stamp

            # speed_error = speed_sp
            # print("Entered is_shutdown")

            right.input['angle'] = theta
            right.input['distance'] = 6*error_d
            left.input['angle'] = theta
            left.input['distance'] = 6*error_d
            right.compute()
            left.compute()

            r = right.output['out_right']
            l = left.output['out_left']
            angle_speed = (r-l)*1000/287 # ???
            angle_speed = (0.7*angle_speed)/20.9059233449 # ????
            linear_speed = (r*0.26)/6

            if os_state == 2: #camera_state 2 is camera lost
                frame_id = nearKeyFrame(values)
            else:
                gamma.angular.z = angle_speed
                gamma.linear.x = linear_speed
                # q_brake = brk_control(output_brk)

                if frame_id >= (next_frame_stop - 6) and frame_id <= (next_frame_stop + 6):
                    # frame_id += 7
                    # print ('should stop')
                    pub_stop.publish(1)
                else:
                    # print ('ok')
                    pub_stop.publish(0)

            pub_twist_msg.publish(gamma)

            last_error_theta = theta
            # last_error_speed = speed_error

            if (error_d <= 1 or y_prime < 0):
                # integral_theta = 0
                #integral_speed = 0
                frame_id += 2
                if frame_id > len(values) - 5:
                    frame_id = 0


            if (angle_speed < 0):
                print('Go RIGHT')
            elif angle_speed > 0:
                print('Go LEFT')

        print('Theta:           ', theta)
        print('Next KeyFrame:   ', frame_id)
        print('x_prime:         ', x_prime)
        print('y_prime:         ', y_prime)
        # print 'Next_curve_id:   ', nearest_curve_id
        print('on_curve:        ', on_curve)
        # print 'd_to_next_curve: ', d_to_next_curve
        print('Speed:           ', speed_d)
        # print('Speed setpoint:  ', speed_sp)
        # print('Speed error:     ', speed_error)
        print('Distance Error:  ', error_d)
        print('twist_msg Setpoint:  ', gamma)
        # print('Acc Setpoint:    ', q_acc)
        # print 'Brakes Setpoint: ', q_brake
        print('Output gamma:    ', output_gamma)
        print('Output_brk:      ', output_brk)
        print(' ')
        rate.sleep()
