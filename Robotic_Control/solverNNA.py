from sympy import *
from math import *
import numpy as np

# Define the given lengths
base_height = 30
shoulder_offset_x = 42
shoulder_offset_y = 46.5
h1 = 26.19
h2 = 102.88
V1 = 95
h3 = 33
end_effector_length = 160


def is_valid_xy(x, y):
    # Adjust coordinates for the shoulder position
    x -= shoulder_offset_x
    y -= shoulder_offset_y
    r = sqrt(x ** 2 + y ** 2)
    return r > V1


def move_to_position_cart(x, y, z):
    if not is_valid_xy(x, y):
        print(
            f"Error: The provided x, y coordinates ({x}, {y}) result in a planar distance r that is less than or equal to V1 ({V1}).")
        print("Please provide coordinates such that the distance from the origin in the xy-plane is greater than V1.")
        return [0, 0, 0, 0]  # Return a default position

    # Adjust coordinates for the shoulder position
    x -= shoulder_offset_x
    y -= shoulder_offset_y
    z -= base_height

    # Calculate the planar distance from the shoulder joint
    r = sqrt(x ** 2 + y ** 2)

    # Distance from the shoulder joint to the end-effector in the xy-plane
    d_xy = sqrt(r ** 2 - V1 ** 2)

    # Height difference between shoulder and end-effector
    d_z = z - h1 - h3

    # Calculate joint angles using inverse kinematics
    try:
        theta1 = atan2(y, x)  # Base rotation angle
        D = (d_xy ** 2 + d_z ** 2 - h2 ** 2 - end_effector_length ** 2) / (2 * h2 * end_effector_length)

        if D < -1 or D > 1:
            print(f"Error: D is out of range for acos, D: {D}.")
            return [0, 0, 0, 0]  # Return a default position

        theta3 = atan2(sqrt(1 - D ** 2), D)  # Elbow angle

        alpha = atan2(d_z, d_xy)
        beta = atan2(end_effector_length * sin(theta3), h2 + end_effector_length * cos(theta3))
        theta2 = alpha - beta  # Shoulder angle

    except ValueError as e:
        print(f"Error: {e}")
        return [0, 0, 0, 0]  # Return a default position

    # Adjust angles to degrees
    theta1 = degrees(theta1)
    theta2 = degrees(theta2)
    theta3 = degrees(theta3)

    # Return the calculated angles
    return [round(theta1), round(theta2), round(theta3), 90]  # Wrist angle set to 90 for pseudo values


def get_previous_teta2():
    text_file = open("prev_teta.txt", "r")
    prev_teta_string = text_file.read()
    text_file.close()

    prev_teta = list(prev_teta_string.split(";"))
    prev_teta.pop(6)
    prev_teta = [int(i) for i in prev_teta]
    return prev_teta


def backlash_compensation_base(theta_base):
    theta_base = round(theta_base)
    theta_base_comp = theta_base

    compensation_value_CW = 8  # degrees (CW rotation)
    compensation_value_CCW = np.linspace(0, 14, 135)
    prev_angles = get_previous_teta2()  # get previous theta's from txt file
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
