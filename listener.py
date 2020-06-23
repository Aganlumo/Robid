#!/usr/bin/env python2
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic
import rospy
import re
import math
import cv2
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
import sys
sys.path.append('/home/david/Robid/astar_ros')
import mapcropper
from astarTuneado import mapWAstar
import cubic_spline_planner

# Lista de posiciones a perseguir
#PX = [-1.0101483, -0.490952, -0.505858, 0.5536769]
#PY = [-0.551212, -0.534223, -1.685323, -1.6705178]
#PA = [6.253257, 6.246468, 4.669046, 6.27733]

global waypoints_x
global waypoints_y
global waypoints_a

# Flag para iniciar el listener
start_flag = 0

# Flag para que el programa siga corriendo
program_flag = 1

# Counter para moverse en la listas de pose
counter = 0

def odometryCb(msg):
	global start_flag
	global program_flag
	global counter

	#Pose perseguida
	x_goal = waypoints_x[counter]
	y_goal = waypoints_y[counter]
	angle_goal = waypoints_a[counter]

	if (program_flag == 1):
		if (start_flag == 0):
			raw_input('Press enter to start')
			start_flag = 1
		else:
			# conversion q4 a euler
			a = math.acos(msg.pose.pose.orientation.w)*2
			x = msg.pose.pose.position.x
			y = msg.pose.pose.position.y

			# publishes velocity robot
			velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
			vel_msg = Twist()
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0

			# Error distancia euclidiana y angulos
			distance_error = math.sqrt(math.pow(x_goal-x,2)+math.pow(y_goal-y,2))
			angle_error = angle_goal-a

			##############################
			angle_error = angle_error * -1
			while(angle_error <= (-2*math.pi)):
				angle_error = angle_error + (2*math.pi)
			while(angle_error >= (2*math.pi)):
				angle_error = angle_error - (2*math.pi)
			angle_error = angle_error * -1
			##############################
			rigth.input['angle'] = angle_error
			rigth.input['distance'] = 6*distance_error
			left.input['angle'] = angle_error
			left.input['distance'] = 6*distance_error

			# Compute es parte de fuzzy
			rigth.compute()
			left.compute()
			r = rigth.output['out_rigth']
			l = left.output['out_left']
			angle_speed = (r-l)*1000/287
			angle_speed = (1.82*angle_speed)/20.9059233449
			linear_speed = (r*0.26)/6
			print("Counter = "+str(counter))
			print("Gx = "+str(x_goal))
			print("Gy = "+str(y_goal))
			print("PX = "+str(x))
			print("PY = "+str(y))
			print("PA = "+str(a))
			print("Error de distancia = "+str(distance_error))
			print("Error de orientacion = "+str(angle_error))
			#print("Punto = "+str(counter))
			#print("R = "+s tr(rigth.output['out_rigth']))
			#print("L = "+str(left.output['out_left']))
			#print("A = "+str(angle_speed))
			#print("Linear = "+str(linear_speed))
			#print ("holi")
			if(abs(angle_error) < 0.2 and abs(distance_error) < 0.1):
				vel_msg.linear.x = 0
				vel_msg.angular.z = 0
				if(counter == len(PX)):
					program_flag = 0
				else:
					counter = counter + 1
			#elif(abs(angle_speed) < 0.2):
			#	vel_msg.linear.x = linear_speed
			#	vel_msg.angular.z = 0
			else:
				vel_msg.linear.x = linear_speed
				vel_msg.angular.z = angle_speed
			velocity_publisher.publish(vel_msg)

	else:
		print("Goal reached")
		sys.exit(0)

def read_pgm(filename, byteorder='>'):

    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

if __name__ == "__main__":

	# Setup A*
	sx = 99
	sy = 183
	gx = 99
	gy = 10
	grid_size = 1
	robot_radius = 2
	image = read_pgm("map.pgm", byteorder='<')
	image = mapcropper.crop(image)
	image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
	# Call a astar
	xbase, ybase = mapWAstar(sx, sy, gx, gy, grid_size, robot_radius, image)

	# Obtencion de puntos y angulos
	csp = cubic_spline_planner.Spline2D(xbase,ybase)
	s = np.arange(0, csp.s[-1], 0.1)
	rx, ry, ryaw = [], [], []
	for i_s in s:
		ix, iy = csp.calc_position(i_s)
		rx.append(ix)
		ry.append(iy)
		ryaw.append(csp.calc_yaw(i_s))
	waypoints_x = rx
	waypoints_y = ry
	waypoints_a = ryaw
	newCounter = len(waypoints_x)
	#Esto es para pasar la imagen a metros
	for i in range(newCounter):
		waypoints_x[i] = (waypoints_x[i]*14.52/299) - 7.26
		waypoints_y[i] = (waypoints_y[i]*10.22/233) - 5.11
	print (len(ryaw))


	angle = ctrl.Antecedent(np.arange(-1, 1, 0.1), 'angle')
	distance = ctrl.Antecedent(np.arange(0, 6.5, 0.1), 'distance')
	out_rigth = ctrl.Consequent(np.arange(0, 6, 0.1), 'out_rigth')
	out_left = ctrl.Consequent(np.arange(0, 6, 0.1), 'out_left')

	angle['N'] = fuzz.trimf(angle.universe, [-1, -1, 0])
	angle['Z'] = fuzz.trimf(angle.universe, [-1, 0, 1])
	angle['P'] = fuzz.trimf(angle.universe, [0, 1, 1])

	distance['Z'] = fuzz.trimf(distance.universe, [0, 0, 3.25])
	distance['M'] = fuzz.trimf(distance.universe, [0, 3.25, 6.5])
	distance['F'] = fuzz.trimf(distance.universe, [3.25, 6.5, 6.5])

	out_rigth['S'] = fuzz.trimf(out_rigth.universe, [0, 0, 3])
	out_rigth['M'] = fuzz.trimf(out_rigth.universe, [0, 3, 6])
	out_rigth['F'] = fuzz.trimf(out_rigth.universe, [3, 6, 6])

	out_left['S'] = fuzz.trimf(out_left.universe, [0, 0, 3])
	out_left['M'] = fuzz.trimf(out_left.universe, [0, 3, 6])
	out_left['F'] = fuzz.trimf(out_left.universe, [3, 6, 6])

	rule1r = ctrl.Rule(angle['N'] & distance['F'], out_rigth['M'])
	rule2r = ctrl.Rule(angle['N'] & distance['M'], out_rigth['M'])
	rule3r = ctrl.Rule(angle['N'] & distance['Z'], out_rigth['S'])
	rule4r = ctrl.Rule(angle['Z'] & distance['F'], out_rigth['F'])
	rule5r = ctrl.Rule(angle['Z'] & distance['M'], out_rigth['M'])
	rule6r = ctrl.Rule(angle['Z'] & distance['Z'], out_rigth['S'])
	rule7r = ctrl.Rule(angle['P'] & distance['F'], out_rigth['F'])
	rule8r = ctrl.Rule(angle['P'] & distance['M'], out_rigth['F'])
	rule9r = ctrl.Rule(angle['P'] & distance['Z'], out_rigth['F'])

	rule1l = ctrl.Rule(angle['N'] & distance['F'], out_left['F'])
	rule2l = ctrl.Rule(angle['N'] & distance['M'], out_left['F'])
	rule3l = ctrl.Rule(angle['N'] & distance['Z'], out_left['M'])
	rule4l = ctrl.Rule(angle['Z'] & distance['F'], out_left['F'])
	rule5l = ctrl.Rule(angle['Z'] & distance['M'], out_left['M'])
	rule6l = ctrl.Rule(angle['Z'] & distance['Z'], out_left['S'])
	rule7l = ctrl.Rule(angle['P'] & distance['F'], out_left['M'])
	rule8l = ctrl.Rule(angle['P'] & distance['M'], out_left['M'])
	rule9l = ctrl.Rule(angle['P'] & distance['Z'], out_left['S'])

	rigth_ctrl = ctrl.ControlSystem([rule1r, rule2r, rule3r, rule4r, rule5r, rule6r, rule7r, rule8r, rule9r])
	left_ctrl = ctrl.ControlSystem([rule1l, rule2l, rule3l, rule4l, rule5l, rule6l, rule7l, rule8l, rule9l])

	rigth = ctrl.ControlSystemSimulation(rigth_ctrl)
	left = ctrl.ControlSystemSimulation(left_ctrl)
	rospy.init_node('my_odom', anonymous=True)
	rospy.Subscriber('odom',Odometry,odometryCb)
	rospy.spin()
