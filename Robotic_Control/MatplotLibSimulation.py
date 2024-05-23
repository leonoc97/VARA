import numpy as np
import math
import matplotlib.pyplot as plt
from ipywidgets import interact, FloatSlider, VBox
import ipywidgets as widgets
from IPython.display import display

# Define the robotic arm dimensions
base_x = 0
base_y = 0
shoulder_x_offset = 42
h1 = 95  # Link length from shoulder to elbow
l3 = 165  # Link length from elbow to end effector


def calculate_joint_positions(x, y):
    # Base angle calculation
    dx = x - base_x
    dy = y - base_y
    base_angle = math.degrees(math.atan2(dy, dx))

    # Convert to the shoulder coordinate system
    shoulder_x = base_x + shoulder_x_offset * math.cos(math.radians(base_angle))
    shoulder_y = base_y + shoulder_x_offset * math.sin(math.radians(base_angle))

    # Distance from shoulder to target
    d = math.sqrt((x - shoulder_x) ** 2 + (y - shoulder_y) ** 2)

    # Calculate shoulder angle using the law of cosines
    cos_theta1 = (d ** 2 + h1 ** 2 - l3 ** 2) / (2 * d * h1)
    cos_theta1 = np.clip(cos_theta1, -1, 1)  # Ensure within valid range
    theta1 = math.degrees(math.acos(cos_theta1)) + math.degrees(math.atan2(y - shoulder_y, x - shoulder_x))

    # Calculate elbow angle using the law of cosines
    cos_theta2 = (h1 ** 2 + l3 ** 2 - d ** 2) / (2 * h1 * l3)
    cos_theta2 = np.clip(cos_theta2, -1, 1)  # Ensure within valid range
    theta2 = math.degrees(math.acos(cos_theta2))

    shoulder_angle = theta1
    elbow_angle = 180 - theta2  # To make sure the arm bends correctly

    return base_angle, shoulder_angle, elbow_angle


def plot_arm(x, y):
    base_angle, shoulder_angle, elbow_angle = calculate_joint_positions(x, y)

    shoulder_x = base_x + shoulder_x_offset * math.cos(math.radians(base_angle))
    shoulder_y = base_y + shoulder_x_offset * math.sin(math.radians(base_angle))

    shoulder_angle_rad = math.radians(shoulder_angle)
    elbow_x = shoulder_x + h1 * math.cos(shoulder_angle_rad)
    elbow_y = shoulder_y + h1 * math.sin(shoulder_angle_rad)

    elbow_angle_rad = shoulder_angle_rad + math.radians(elbow_angle)
    end_x = elbow_x + l3 * math.cos(elbow_angle_rad)
    end_y = elbow_y + l3 * math.sin(elbow_angle_rad)

    fig, ax = plt.subplots(figsize=(12, 6))

    # 2D Plot
    ax.plot([base_x, shoulder_x], [base_y, shoulder_y], 'r-o', label="Base to Shoulder")
    ax.plot([shoulder_x, elbow_x], [shoulder_y, elbow_y], 'g-o', label="Shoulder to Elbow")
    ax.plot([elbow_x, end_x], [elbow_y, end_y], 'b-o', label="Elbow to End Effector")

    ax.scatter([base_x, shoulder_x, elbow_x, end_x], [base_y, shoulder_y, elbow_y, end_y], color='k', zorder=5)

    ax.text(base_x, base_y, 'Base', fontsize=12, ha='right')
    ax.text(shoulder_x, shoulder_y, 'Shoulder', fontsize=12, ha='right')
    ax.text(elbow_x, elbow_y, 'Elbow', fontsize=12, ha='right')
    ax.text(end_x, end_y, 'End Effector', fontsize=12, ha='right')

    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(f"Robotic Arm 2D Visualization\nTarget Position: ({x}, {y})")
    ax.legend()
    plt.show()


def interactive_simulation(x, y):
    plot_arm(x, y)


# Interactive sliders for x, y
x_slider = FloatSlider(min=-200, max=200, step=1, value=base_x + shoulder_x_offset, description='X:')
y_slider = FloatSlider(min=-200, max=200, step=1, value=0, description='Y:')

# Display interactive widgets
ui = VBox([x_slider, y_slider])
out = widgets.interactive_output(interactive_simulation, {'x': x_slider, 'y': y_slider})
display(ui, out)
