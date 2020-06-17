#!/usr/bin/env python
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
import math
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import sys

PX = [-1.0101483, -0.490952, -0.505858, 0.5536769]
PY = [-0.551212, -0.534223, -1.685323, -1.6705178]
PA = [6.253257, 6.246468, 4.669046, 6.27733]
start_flag = 0
program_flag = 1
counter = 0

def odometryCb(msg):
	global start_flag
	global program_flag
	global counter
	global PX
	global PY
	global PA
	x_goal = PX[counter]
	y_goal = PY[counter]
	angle_goal = PA[counter]
	if (program_flag == 1):
		if (start_flag == 0):
			raw_input('Press enter to start')
			start_flag = 1
		else:
			a = math.acos(msg.pose.pose.orientation.w)*2
			x = msg.pose.pose.position.x
			y = msg.pose.pose.position.y
			velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
			vel_msg = Twist()
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			distance_error = math.sqrt(math.pow(x_goal-x,2)+math.pow(y_goal-y,2))
			angle_error = angle_goal-a
			angle_error = angle_error * -1
			while(angle_error <= (-2*math.pi)):
				angle_error = angle_error + (2*math.pi)
			while(angle_error >= (2*math.pi)):
				angle_error = angle_error - (2*math.pi)
			angle_error = angle_error * -1
			rigth.input['angle'] = angle_error
			rigth.input['distance'] = 6*distance_error
			left.input['angle'] = angle_error
			left.input['distance'] = 6*distance_error
			rigth.compute()
			left.compute()
			r = rigth.output['out_rigth']
			l = left.output['out_left']
			angle_speed = (r-l)*1000/287
			angle_speed = (1.82*angle_speed)/20.9059233449
			linear_speed = (r*0.26)/6
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

if __name__ == "__main__":
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










'''
import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import String
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
client = ModbusClient(method = 'rtu', port='/dev/ttyUSB0', timeout=1, stopbits=1, bytsize=8, parity='N', baudrate=9600)
client.connect()
check=0
ohuh=0
apagalootto=0
prro=0
cx1=0
cx2=0
cxf=0
oops=0
#def talker(a):
 #   pub = rospy.Publisher('chatter', String, queue_size=10)
  #  rospy.init_node('talker', anonymous=True)
   # rate = rospy.Rate(50) # 10hz
    #while not rospy.is_shutdown():
   # rospy.loginfo(a)
   # pub.publish(a)
   # rate.sleep()
def avanzar():
	#freno
	print("VROOM")
	client.write_register(44,485,unit=5) #valor inferior
	client.write_register(45,490,unit=5) #medio
	client.write_register(46,492,unit=5) #superior
	#acelerador
	client.write_register(16,8,unit=5) #valor inferior
	client.write_register(17,14,unit=5) #superior
	client.write_register(18,11,unit=5) #medio
def alto():
	#freno
	print("PARA")
	client.write_register(44,410,unit=5) #inferior
	client.write_register(45,412,unit=5) #medio
	client.write_register(46,416,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,7,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
def derecha():
	print("derecha")
	client.write_register(15,37,unit=5)#38
def tower():
	print("por un segundo")
	client.write_register(15,28,unit=5)
def izquierda():
	print("izquierda")
	client.write_register(15,23,unit=5)#22
def frente():
	print("frente")
	client.write_register(15,30,unit=5)
def obstaculo(espera,nmms):
	#81 paro de emergencia
	while(espera==1 or nmms==1):
		rr=client.read_input_registers(83,1,unit=5)
		espera=rr.registers[0]
		rocky2=client.read_input_registers(81,1,unit=5)		
		nmms=rocky2.registers[0]
		#freno
		client.write_register(44,410,unit=5) #inferior
		client.write_register(45,412,unit=5) #medio
		client.write_register(46,416,unit=5) #superior
		#acelerador
		client.write_register(16,3,unit=5)
		client.write_register(17,7,unit=5)
		client.write_register(18,5,unit=5)
		time.sleep(1)
def neutro():
	#Freno
	print("avanzo")
	client.write_register(44,485,unit=5) #infrior+2
	client.write_register(45,490,unit=5) #mdio
	client.write_register(46,492,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,6,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
def parking():
	client.write_register(48,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(48,0,unit=5)
	time.sleep(3)
def drive():
	client.write_register(50,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(50,0,unit=5)
	time.sleep(5)
def reversa():
	client.write_register(49,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(49,0,unit=5)
	time.sleep(3)
def encendido():
	#Arrancado de vehiculo			
	#cambio velocidad			
	print("parking")
	client.write_register(48,1,unit=5)
	time.sleep(0.2)
	for i in range(4):
		print("NOTETRABES")
	client.write_register(48,0,unit=5)
	time.sleep(1)
	for x in range(5):
		print("freno")
		#freno
		client.write_register(44,410,unit=5)
		client.write_register(45,412,unit=5)
		client.write_register(46,416,unit=5)
		#direccion
		#client.write_register(15,42,unit=5)
		#acelerador
		client.write_register(16,5,unit=5)
		client.write_register(17,7,unit=5)
		client.write_register(18,6,unit=5)
		time.sleep(1)
	print("ignition")
	client.write_register(84,0,unit=5)
	time.sleep(0.2)
	client.write_register(80,1,unit=5)
def apagado():
	client.write_register(80,0,unit=5) 
	time.sleep(0.2)
	client.write_register(84,1,unit=5)
def cuidado():
	global ohuh
	global apagalootto
	rocky6=client.read_input_registers(81,1,unit=5)
	raios=client.read_input_registers(83,1,unit=5)
	ohuh=raios.registers[0]
	apagalootto=rocky6.registers[0]
	if(ohuh==0 and apagalootto==0):	
		print("vas bien")
		return 0		
	elif(ohuh==1 or apagalootto==1):
		print("GOLPE AVISA")
		obstaculo(ohuh,apagalootto)

def runo():
	frente()	
	alto()
	time.sleep(4)	
	drive()
	time.sleep(4)
	neutro()
	frente()
	time.sleep(4)
	derecha()
	time.sleep(11.5)
	frente()
	time.sleep(8)
	izquierda()
	time.sleep(10)
	frente()
	time.sleep(1)
	alto()
	time.sleep(5)
	reversa()
	time.sleep(4)
	neutro()
	derecha()
	time.sleep(4)
	frente()
	time.sleep(2)
	alto()
	time.sleep(5)
	parking()
	client.write_register(36,0,unit=5)				


if __name__ == '__main__':
    #while not rospy.is_shutdown():
	client.write_register(36,1,unit=5)        
	runo()
    #except rospy.ROSInterruptException:
     #   pass
'''