# Control Script
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 28 12:21:04 2021
Modified to work with a new robotic arm setup with PCA9685
"""

# Importing Libraries
import serial
import time
import solverNNA
import numpy as np

base = [90, 0, 180, 0]  # default value for base, min value and max value, write location
shoulder = [90, 15, 165, 1]
elbow = [90, 0, 180, 2]
# Pseudo values for unused joints
wrist = [90, 0, 180, 3]
wristRot = [90, 0, 180, 4]
gripper = [73, 10, 73, 5]

arm = serial.Serial('COM7', 115200, timeout=5)
print("Initializing arm")
time.sleep(2)
arm.write(b'H\n')  # home the arm at low speeds
time.sleep(2)


def write_arduino(angles):
    angles[0] = 180 - angles[0]  # invert degrees for base
    angle_string = ','.join([str(elem) for elem in angles])  # join the list values together
    angle_string = "P" + angle_string + ",150\n"  # Default speed 150
    arm.write(angle_string.encode())  # .encode encodes the string to bytes


def home(speed=150):
    angle_string_def_angles = [base[0], shoulder[0], elbow[0], wrist[0], wristRot[0], gripper[0]]
    write_arduino(angle_string_def_angles)


def write_position(theta_base=base[0], theta_shoulder=shoulder[0], theta_elbow=elbow[0], grip="closed"):
    # Use pseudo values for unused joints
    theta_wrist = wrist[0]
    theta_wristRot = wristRot[0]

    if grip.lower() == "closed":
        theta_gripper = gripper[1]
    elif grip.lower() == "open":
        theta_gripper = gripper[2]
    else:
        print("Invalid gripper state, defaulting to closed.")
        theta_gripper = gripper[1]

    theta_base_comp = solverNNA.backlash_compensation_base(theta_base)  # check if compensation is needed
    angle_string_def_angles = [theta_base_comp, theta_shoulder, theta_elbow, theta_wrist, theta_wristRot, theta_gripper]
    write_arduino(angle_string_def_angles)

    # write angle values in txt file without the compensation
    angles = [theta_base, theta_shoulder, theta_elbow, theta_wrist, theta_wristRot, theta_gripper]
    text_file = open("prev_teta.txt", "w")
    iteration = [0, 1, 2, 3, 4, 5]
    for elem in iteration:
        text_file.write(str(angles[elem]))
        text_file.write(";")
    text_file.close()


def go_to_coordinate(x, y, z, grip_position="closed"):
    theta_list = solverNNA.move_to_position_cart(x, y, z)
    if theta_list == [0, 0, 0, 0]:
        print("Error in calculating angles. Please check the input values.")
        return
    write_position(theta_list[0], theta_list[1], theta_list[2], grip=grip_position)


def manual_input():
    while True:
        x = float(input("Enter X coordinate: "))
        y = float(input("Enter Y coordinate: "))
        z = float(input("Enter Z coordinate: "))
        grip = input("Enter gripper state (open/closed): ").strip().lower()
        if grip not in ["open", "closed"]:
            print("Invalid gripper state, defaulting to closed.")
            grip = "closed"
        go_to_coordinate(x, y, z, grip)
        if input("Continue? (y/n): ").lower() != 'y':
            break


# Now you can call `manual_input()` to manually input coordinates and test the inverse kinematics
manual_input()
