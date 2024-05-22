# -*- coding: utf-8 -*-
"""
Created on Sun Jan  2 13:32:10 2022

@author: natan
"""

from math import acos, atan2, degrees, sqrt, sin, cos, radians
import numpy as np

# Constants for your robotic arm (in mm)
base_to_ground = 150
h1 = 26.19
h2 = 102.88
V1 = 95
h3 = 33
l0 = V1 + h1 + base_to_ground
l1 = h2
l2 = 157
l3 = 42

# Servo angle limits
BASE_MIN = 15
BASE_MAX = 140
SHOULDER_MIN = 0
SHOULDER_MAX = 70
ELBOW_MIN = 0
ELBOW_MAX = 100

def move_to_position_cart(x, y, z):
    # Calculate planar distance r
    r = sqrt(x**2 + y**2)

    # Calculate base angle theta_base
    theta_base = degrees(atan2(y, x))
    theta_base = constrain_angle(theta_base, BASE_MIN, BASE_MAX)

    # Adjust z to account for base height and V1
    z_adjusted = z - base_to_ground - V1

    # Inverse kinematics for shoulder and elbow
    d = sqrt(r**2 + z_adjusted**2)

    # Check if the position is reachable
    if d > (l1 + l2) or d < abs(l1 - l2):
        raise ValueError("Position out of reach")

    # Law of cosines to find angles
    angle_a = acos((l1**2 + d**2 - l2**2) / (2 * l1 * d))
    angle_b = acos((l1**2 + l2**2 - d**2) / (2 * l1 * l2))
    angle_c = atan2(z_adjusted, r)

    theta_shoulder = degrees(angle_a + angle_c)
    theta_elbow = 180 - degrees(angle_b)  # Adjust to the correct range for the elbow servo

    # Ensure angles are within the valid ranges
    theta_shoulder = constrain_angle(theta_shoulder, SHOULDER_MIN, SHOULDER_MAX)
    theta_elbow = constrain_angle(theta_elbow, ELBOW_MIN, ELBOW_MAX)

    return [round(theta_base), round(theta_shoulder), round(theta_elbow)]

def constrain_angle(angle, min_angle, max_angle):
    return max(min_angle, min(max_angle, angle))

def get_previous_teta():
    text_file = open("prev_teta.txt", "r")
    prev_teta_string = text_file.read()
    text_file.close()

    prev_teta = list(map(int, prev_teta_string.split(";")[:6]))
    return prev_teta

def backlash_compensation_base(theta_base):
    theta_base = round(theta_base)
    theta_base_comp = theta_base

    compensation_value_CW = 8  # degrees (CW rotation)
    compensation_value_CCW = np.linspace(0, 14, 135)
    prev_angles = get_previous_teta()  # get previous theta's from txt file
    theta_base_prev = prev_angles[0]

    delta_theta_base = theta_base - theta_base_prev

    if delta_theta_base > 1:
        if theta_base <= 45:
            theta_base_comp = theta_base
        else:
            index = int(round(theta_base - 46))
            theta_base_comp = theta_base + compensation_value_CCW[index]
            theta_base_comp = round(theta_base_comp)
    if delta_theta_base < -1:
        theta_base_comp = theta_base - compensation_value_CW

    return theta_base_comp

# Example usage
if __name__ == "__main__":
    try:
        angles = move_to_position_cart(100, 50, 150)
        print(f"Base: {angles[0]}, Shoulder: {angles[1]}, Elbow: {angles[2]}")
    except ValueError as e:
        print(e)
