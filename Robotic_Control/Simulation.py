import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from math import atan2, sqrt, degrees, radians, sin, cos, pi

# Define the given lengths
base_height = 30
shoulder_offset_y = 42
shoulder_offset_z = 46.5
h1 = 26.19
h2 = 102.88
V1 = 95
h3 = 33
end_effector_length = 160

# Function to validate if (x, y) is within the valid range
def is_valid_xy(x, y):
    r = sqrt(x ** 2 + y ** 2)
    return r > V1

# Inverse Kinematics Solver
def move_to_position_cart(x, y, z):
    if not is_valid_xy(x, y):
        print(f"Error: The provided x, y coordinates ({x}, {y}) result in a planar distance r that is less than or equal to V1 ({V1}).")
        print("Please provide coordinates such that the distance from the origin in the xy-plane is greater than V1.")
        return [0, 0, 0]  # Return a default position

    # Adjust coordinates for the shoulder position
    y -= shoulder_offset_y
    z -= base_height + shoulder_offset_z

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
            return [0, 0, 0]  # Return a default position

        theta3 = atan2(sqrt(1 - D ** 2), D)  # Elbow angle

        alpha = atan2(d_z, d_xy)
        beta = atan2(end_effector_length * sin(theta3), h2 + end_effector_length * cos(theta3))
        theta2 = alpha - beta  # Shoulder angle

    except ValueError as e:
        print(f"Error: {e}")
        return [0, 0, 0]  # Return a default position

    # Adjust angles to degrees
    theta1 = degrees(theta1)
    theta2 = degrees(theta2)
    theta3 = degrees(theta3)

    # Return the calculated angles
    return [round(theta1), round(theta2), round(theta3)]

# Function to calculate joint positions based on angles
def calculate_joint_positions(theta1, theta2, theta3):
    theta1 = radians(theta1)
    theta2 = radians(theta2)
    theta3 = radians(theta3)

    # Shoulder position
    shoulder_y = shoulder_offset_y
    shoulder_z = shoulder_offset_z

    # Elbow position
    elbow_y = shoulder_y + h2 * cos(theta2)
    elbow_z = shoulder_z + h2 * sin(theta2)

    # End-effector position
    end_effector_y = elbow_y + end_effector_length * cos(theta2 + theta3)
    end_effector_z = elbow_z + end_effector_length * sin(theta2 + theta3)

    return (shoulder_y, shoulder_z), (elbow_y, elbow_z), (end_effector_y, end_effector_z)

# Function to visualize the robot
def visualize_robot(theta1, theta2, theta3):
    shoulder, elbow, end_effector = calculate_joint_positions(theta1, theta2, theta3)

    plt.clf()
    plt.plot([0, shoulder[1]], [0, shoulder[0]], 'ro-', label='Base to Shoulder')
    plt.plot([shoulder[1], elbow[1]], [shoulder[0], elbow[0]], 'go-', label='Shoulder to Elbow')
    plt.plot([elbow[1], end_effector[1]], [elbow[0], end_effector[0]], 'bo-', label='Elbow to End Effector')
    plt.scatter([0, shoulder[1], elbow[1], end_effector[1]], [0, shoulder[0], elbow[0], end_effector[0]],
                color=['black', 'red', 'green', 'blue'])
    plt.xlim(-200, 200)
    plt.ylim(-200, 200)
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.grid(color='gray', linestyle='--', linewidth=0.5)
    plt.legend()
    plt.title('Robot Arm Configuration')
    plt.xlabel('Z (mm)')
    plt.ylabel('Y (mm)')
    plt.draw()

# Interactive plot with sliders
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.35)

# Initial coordinates
x = 150
y = 100
z = 50

# Initial plot
angles = move_to_position_cart(x, y, z)
if angles != [0, 0, 0]:
    visualize_robot(*angles)
else:
    print("Invalid configuration.")

# Create sliders
axcolor = 'lightgoldenrodyellow'
ax_x = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_y = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
ax_z = plt.axes([0.25, 0.2, 0.65, 0.03], facecolor=axcolor)

s_x = Slider(ax_x, 'X', -200, 200, valinit=x)
s_y = Slider(ax_y, 'Y', -200, 200, valinit=y)
s_z = Slider(ax_z, 'Z', 0, 200, valinit=z)

# Update function for sliders
def update(val):
    x = s_x.val
    y = s_y.val
    z = s_z.val
    angles = move_to_position_cart(x, y, z)
    if angles != [0, 0, 0]:
        visualize_robot(*angles)
    else:
        print("Invalid configuration.")

s_x.on_changed(update)
s_y.on_changed(update)
s_z.on_changed(update)

plt.show()
