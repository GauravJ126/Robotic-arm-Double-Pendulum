# -*- coding: utf-8 -*-
"""
Created on Tue Nov 22 09:43:56 2022

@author: Venom
"""

import math
import matplotlib.pyplot as plt

#Inputs
l1 = 5.65
l2 = 5.65

n_theta = 10
theta_start = 0
theta_end = math.pi/2

theta1 = []
theta2 = []

for i in range(0,n_theta):
	tmp = theta_start + i*(theta_end - theta_start)/(n_theta - 1)
	theta1.append(tmp)
	theta2.append(tmp)

# Base of the Robot
x0 = 0
y0 = 0

ct = 1

for t1 in theta1:
	for t2 in theta2:

		# Link 1, Link 2 connector
		x1 = l1*math.cos(t1)
		y1 = l1*math.sin(t1)

		# Coordinates of the manipulator
		x2 = x1 + l2*math.cos(t2)
		y2 = y1 + l2*math.sin(t2)

		filename = str(ct) + '.png'
		ct = ct + 1

		# Plotting
		plt.figure()
		plt.plot([x0,x1],[y0,y1])
		plt.plot([x1,x2],[y1,y2])
		plt.xlim([-8,8])
		plt.ylim([-8,8])
# 		plt.savefig(filename)


################################################################################

# import time
# import math
# import RPi.GPIO as GPIO

# # Set the GPIO mode
# GPIO.setmode(GPIO.BCM)

# # Define the pin numbers for the two servo motors
# servo_pin1 = 18
# servo_pin2 = 19

# # Set the servo pins as outputs
# GPIO.setup(servo_pin1, GPIO.OUT)
# GPIO.setup(servo_pin2, GPIO.OUT)

# # Define the pulse width range for the servo motors
# pulse_min = 0.5
# pulse_max = 2.5

# # Define the system parameters
# l1 = 1.0 # length of the first pendulum
# l2 = 1.0 # length of the second pendulum
# m1 = 1.0 # mass of the first pendulum
# m2 = 1.0 # mass of the second pendulum
# g = 9.8 # acceleration due to gravity

# # Define the initial and final states of the system
# x1_0 = 0.0 # initial x-coordinate of the first pendulum
# y1_0 = l1 # initial y-coordinate of the first pendulum
# x2_0 = x1_0 + l2 * math.cos(math.pi/4) # initial x-coordinate of the second pendulum
# y2_0 = y1_0 + l2 * math.sin(math.pi/4) # initial y-coordinate of the second pendulum
# x1_f = 2.0 # final x-coordinate of the first pendulum
# y1_f = 2.0 # final y-coordinate of the first pendulum
# x2_f = x1_f + l2 * math.cos(math.pi/4) # final x-coordinate of the second pendulum
# y2_f = y1_f + l2 * math.sin(math.pi/4) # final y-coordinate of the second pendulum

# # Define the frequency of the servo PWM signal
# frequency = 50

# # Create PWM objects for the two servo motors
# pwm1 = GPIO.PWM(servo_pin1, frequency)
# pwm2 = GPIO.PWM(servo_pin2, frequency)

# # Start the PWM signals
# pwm1.start(0)
# pwm2.start(0)

# # Define the function to calculate the duty cycle for a given angle
# def calc_duty_cycle(angle):
#     return (pulse_min + angle / 180 * (pulse_max - pulse_min)) / 20 * 100

# # Define the inverse kinematics function for the double pendulum
# def inv_kin(x1, y1, x2, y2):
#     theta1 = math.atan2(y1, x1)
#     theta2 = math.atan2(y2 - y1, x2 - x1) - theta1
#     return theta1, theta2

# # Calculate the initial joint angles
# theta1_0, theta2_0 = inv_kin(x1_0, y1_0, x2_0, y2_0)

