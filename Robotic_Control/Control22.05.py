import os
import serial
import time
import numpy as np
from math import acos, atan2, sqrt, degrees, radians, sin, cos
from solverNNA_VARA_Kinematics import move_to_position_cart, backlash_compensation_base

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
SHOULDER_MAX = 78
ELBOW_MIN = 0
ELBOW_MAX = 100

# Servo parameters
base = [80, BASE_MIN, BASE_MAX, 0]
shoulder = [40, SHOULDER_MIN, SHOULDER_MAX, 1]
elbow = [50, ELBOW_MIN, ELBOW_MAX, 2]
wrist = [0, 0, 180, 3]
wristRot = [90, 0, 180, 4]
gripper = [73, 10, 73, 5]


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def initialize_prev_teta():
    initial_teta_values = [base[0], shoulder[0], elbow[0], wrist[0], wristRot[0], gripper[0]]
    if not os.path.exists("prev_teta.txt"):
        with open("prev_teta.txt", "w") as text_file:
            for angle in initial_teta_values:
                text_file.write(f"{angle};")
        print("Initialized prev_teta.txt with default values.")
    else:
        print("prev_teta.txt already exists.")


def write_arduino(angles):
    angles[0] = constrain(180 - angles[0], base[1], base[2])  # invert degrees for base
    angles[1] = constrain(angles[1], shoulder[1], shoulder[2])
    angles[2] = constrain(angles[2], elbow[1], elbow[2])
    angles[3] = constrain(180 - angles[3], wrist[1], wrist[2])  # invert degrees for wrist
    angles[4] = constrain(angles[4], wristRot[1], wristRot[2])
    angles[5] = constrain(angles[5], gripper[1], gripper[2])

    angle_string = ','.join([str(elem) for elem in angles])
    angle_string = "P" + angle_string + ",200\n"
    arm.write(angle_string.encode())


def rotate_joint(joint):
    def calculate_joint(joint, number):
        angle_string_def_angles = [base[0], shoulder[0], elbow[0], wrist[0], wristRot[0], gripper[0]]
        angle_string_def_angles[joint[3]] = joint[number]
        write_arduino(angle_string_def_angles)

    calculate_joint(joint, 1)
    time.sleep(2)
    calculate_joint(joint, 2)
    time.sleep(2)
    calculate_joint(joint, 1)
    time.sleep(2)


def home(speed=20):
    angle_string_def_angles = [base[0], shoulder[0], elbow[0], wrist[0], wristRot[0], gripper[0]]
    write_arduino(angle_string_def_angles)


def rotate_all_joints():
    print("The base.")
    rotate_joint(base)
    print("The shoulder.")
    rotate_joint(shoulder)
    print("The elbow.")
    rotate_joint(elbow)
    print("The vertical axis of the wrist.")
    rotate_joint(wrist)
    print("The rotational axis of the wrist.")
    rotate_joint(wristRot)
    print("The gripper.")
    rotate_joint(gripper)


def write_position(theta_base=base[0], theta_shoulder=shoulder[0], theta_elbow=elbow[0], theta_wrist=wrist[0],
                   theta_wristRot=wristRot[0], grip="closed"):
    if grip == "closed":
        theta_gripper = gripper[1]
    elif grip == "open":
        theta_gripper = gripper[2]
    else:
        theta_gripper = gripper[0]

    theta_base_comp = backlash_compensation_base(theta_base)
    angle_string_def_angles = [theta_base_comp, theta_shoulder, theta_elbow, theta_wrist, theta_wristRot, theta_gripper]
    write_arduino(angle_string_def_angles)

    angles = [theta_base, theta_shoulder, theta_elbow, theta_wrist, theta_wristRot, theta_gripper]
    with open("prev_teta.txt", "w") as text_file:
        for angle in angles:
            text_file.write(f"{angle};")


def go_to_coordinate(x, y, z, grip_position="closed"):
    try:
        theta_list = move_to_position_cart(x, y, z)
        print(f"Calculated joint angles: {theta_list}")  # Debugging statement to print calculated joint angles
        if len(theta_list) < 5:
            theta_list += [0] * (5 - len(theta_list))  # Ensure theta_list has at least 5 elements
        write_position(theta_list[0], theta_list[1], theta_list[2], theta_list[3], theta_list[4], grip=grip_position)
    except ValueError as e:
        print(f"Error: {e}. Coordinates ({x}, {y}, {z}) are out of reach.")


def move_vertical(x, y):
    loop_iteration = np.linspace(0, 350, 2)
    for z in loop_iteration:
        print(z)
        go_to_coordinate(x, y, round(z))
        time.sleep(2)


def move_horizontal(z):
    loop_iteration = np.linspace(100, 350, 2)
    for x in loop_iteration:
        print(x)
        go_to_coordinate(round(x), 0, z)
        time.sleep(2)


def get_previous_teta():
    with open("prev_teta.txt", "r") as text_file:
        prev_teta_string = text_file.read()
    prev_teta = list(map(int, prev_teta_string.split(";")[:6]))
    return prev_teta


def open_gripper():
    prev_angles = get_previous_teta()
    write_position(prev_angles[0], prev_angles[1], prev_angles[2], prev_angles[3], prev_angles[4], grip="open")


def close_gripper():
    prev_angles = get_previous_teta()
    write_position(prev_angles[0], prev_angles[1], prev_angles[2], prev_angles[3], prev_angles[4], grip="closed")


def pick_up(x, y):
    glass_pos = [310, 95]  # x, y pos of glass
    delay = 1  # delay between steps
    pick_up_height = 10  # height of the object
    home()
    time.sleep(delay)
    go_to_coordinate(x, y, 100, "closed")
    time.sleep(delay)
    open_gripper()
    time.sleep(delay)
    print('pick-up foam')
    go_to_coordinate(x, y, pick_up_height - 20, "open")
    time.sleep(delay)
    close_gripper()
    time.sleep(delay)
    go_to_coordinate(x, y, 200, "closed")
    time.sleep(delay)
    go_to_coordinate(glass_pos[0], glass_pos[1], 200, "closed")
    time.sleep(delay)
    go_to_coordinate(glass_pos[0], glass_pos[1], 120, "closed")
    time.sleep(delay)
    open_gripper()
    home()


def backlash():
    time.sleep(5)
    write_position(90, 0, 90, 90)
    time.sleep(2)
    write_position(45, 0, 90, 90)
    time.sleep(2)
    write_position(90, 0, 90, 90)


def camera_compensation(x_coordinate, y_coordinate):
    h_foam = 80  # foam height of 80mm
    camera_position = [480, 150, 880]  # x, y, z coordinate from origin in mm
    offset = 300  # add 300 to move origin to under the camera
    x_coordinate = (offset - x_coordinate) + (camera_position[0] - offset)

    # Perform compensation
    x_compensated = x_coordinate - (h_foam / (camera_position[2] / x_coordinate))
    if y_coordinate < camera_position[1]:
        y_compensated = y_coordinate - (h_foam / (camera_position[2] / y_coordinate))
    else:
        y_compensated = y_coordinate + (h_foam / (camera_position[2] / y_coordinate))

    # Subtract the offset
    x_compensated = offset - (x_compensated - (camera_position[0] - offset))

    return int(x_compensated), int(y_compensated)


def power_on():
    arm.write(b'1\n')


def power_off():
    arm.write(b'0\n')


def main_menu():
    while True:
        print("\nSelect a command:")
        print("1. Home Position")
        print("2. Rotate All Joints")
        print("3. Open Gripper")
        print("4. Close Gripper")
        print("5. Pick Up Object")
        print("6. Move Vertical")
        print("7. Move Horizontal")
        print("8. Power On")
        print("9. Power Off")
        print("10. Exit")

        choice = input("Enter the number of your choice: ")

        if choice == '1':
            home()
        elif choice == '2':
            rotate_all_joints()
        elif choice == '3':
            open_gripper()
        elif choice == '4':
            close_gripper()
        elif choice == '5':
            x = int(input("Enter x-coordinate: "))
            y = int(input("Enter y-coordinate: "))
            pick_up(x, y)
        elif choice == '6':
            x = int(input("Enter x-coordinate: "))
            y = int(input("Enter y-coordinate: "))
            move_vertical(x, y)
        elif choice == '7':
            z = int(input("Enter z-coordinate: "))
            move_horizontal(z)
        elif choice == '8':
            power_on()
        elif choice == '9':
            power_off()
        elif choice == '10':
            print("Exiting...")
            break
        else:
            print("Invalid choice, please try again.")


if __name__ == "__main__":
    try:
        # Initialize prev_teta.txt with default angles if it doesn't exist
        initialize_prev_teta()

        arm = serial.Serial('COM7', 115200, timeout=5)
        print("Initializing arm")
        time.sleep(2)
        arm.write(b'H0,90,20,90,90,73,20\n')
        time.sleep(2)
        main_menu()
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
