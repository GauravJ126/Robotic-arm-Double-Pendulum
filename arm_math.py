# -*- coding: utf-8 -*-
"""
Created on Sun Mar 12 10:56:26 2023
@author: Venom
"""

# speed = 2
# distance = 8
# t = distance/speed


# arm_1_angle = 30
# arm_new_1_angle = 60
# delta_arm1 = arm_new_1_angle - arm_1_angle


# arm_2_angle = 70
# arm_new_2_angle = 30 
# delta_arm2 = arm_new_2_angle - arm_2_angle


# angular_v2 = delta_arm2 / t
# angular_v1 = delta_arm1 / t

# print(angular_v1,angular_v2)

import numpy as np
from scipy.optimize import fsolve
from math import atan2, acos, sin, cos, pi, sqrt

def solve_angles(x, A, B):
    alpha, beta = x
    D = np.linalg.norm(B - A) / (2 * l1)
    alpha = atan2(-cos(beta)*sin(acos(D)), D + sin(beta)*cos(acos(D)))
    return [alpha - atan2(A[1], A[0]), beta - alpha - atan2(B[1] - A[1], B[0] - A[0])]

def solve_velocities(x, A, B, v):
    w1, w2 = x[2], x[3]
    theta1, theta2 = x[0] + pi/2, x[1] + pi/2
    J11 = -l1*sin(theta1) - l2*sin(theta1 + theta2)
    J12 = -l2*sin(theta1 + theta2)
    J21 = l1*cos(theta1) + l2*cos(theta1 + theta2)
    J22 = l2*cos(theta1 + theta2)
    J = np.array([[J11, J12], [J21, J22]])
    J_inv = np.linalg.inv(J)
    e = np.array([[B[0] - A[0]], [B[1] - A[1]]])
    theta_dot = np.matmul(J_inv, e * v)
    return [theta_dot[0][0] - w1, theta_dot[1][0] - w2]

def find_arm_velocities(point_a, point_b, v):
    global l1, l2
    l1, l2 = 15, 15
    alpha = atan2(point_a[1], point_a[0])
    beta = acos((l1**2 + np.linalg.norm(point_b - point_a)**2 - l2**2) / (2*l1*np.linalg.norm(point_b - point_a)))
    alpha, beta = fsolve(solve_angles, [alpha, beta], args=(point_a, point_b))
    w1, w2 = fsolve(solve_velocities, [alpha, beta, 0, 0], args=(point_a, point_b, v))
    return w1, w2

find_arm_velocities({1,2}, {1,5}, 2)
