import serial
import time
import math
import numpy as np

# Open serial connection to Arduino
arduino = serial.Serial('COM7', 115200, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

# Define the robotic arm dimensions and constraints
base_x = 0
base_y = -42
base_z = 30
shoulder_x_offset = 0
shoulder_y_offset = 42
shoulder_z_offset = 46.5
#For the elbow
h1 = 26.19
h2 = 102.88
V1 = 95
h3 = 33
#Link length from elbow to endeffector
l3 = 165

# Angle constraints for each joint
BASE_MIN_ANGLE = 0
BASE_MAX_ANGLE = 180
SHOULDER_MIN_ANGLE = 15
SHOULDER_MAX_ANGLE = 165
ELBOW_MIN_ANGLE = 0
ELBOW_MAX_ANGLE = 180


def send_command(command):
    arduino.write(command.encode())
    response = arduino.readline().decode().strip()
    print(f"Arduino Response: {response}")
    return response


def power_off():
    send_command('0\n')


def power_on():
    send_command('1\n')


def home_position():
    # Set the arm to the initial angles specified
    base_angle = 140
    shoulder_angle = 110
    elbow_angle = 50
    command = f"P{base_angle},{shoulder_angle},{elbow_angle},90,90,73,150\n"
    send_command(command)


def move_to_origin():
    home_position()  # Move to home position first


def move_all_joints():
    positions = [
        "P15,15,0,0,0,10,150\n",
        "P140,70,100,180,180,73,150\n",
        "P90,35,50,90,90,41,150\n"
    ]
    for pos in positions:
        send_command(pos)
        time.sleep(2)


def calculate_inverse_kinematics(x, y, z):
    # Ensure the target is within positive Cartesian space
    if x < 0 or y < 0:
        raise ValueError("Target coordinates must be in the positive Cartesian space.")

    # Base angle calculation
    dx = x - base_x
    dy = y - base_y
    base_angle = math.degrees(math.atan2(dy, dx))

    # Convert to the shoulder coordinate system
    shoulder_x = x - (base_x + shoulder_x_offset * math.cos(math.radians(base_angle)))
    shoulder_y = y - base_y
    shoulder_z = z - (base_z + shoulder_z_offset)

    # Distance from shoulder to target
    d = math.sqrt(shoulder_x ** 2 + shoulder_z ** 2)

    # Calculate shoulder angle using the law of cosines
    cos_theta1 = (d ** 2 + h1 ** 2 - V1 ** 2) / (2 * d * h1)
    cos_theta1 = np.clip(cos_theta1, -1, 1)  # Ensure within valid range
    theta1 = math.degrees(math.acos(cos_theta1)) + math.degrees(math.atan2(shoulder_z, shoulder_x))

    # Calculate elbow angle based on the parallelogram linkage
    cos_theta2 = (V1 ** 2 + h3 ** 2 - h2 ** 2) / (2 * V1 * h3)
    cos_theta2 = np.clip(cos_theta2, -1, 1)  # Ensure within valid range
    theta2 = math.degrees(math.acos(cos_theta2))

    shoulder_angle = theta1
    elbow_angle = theta2

    return base_angle, shoulder_angle, elbow_angle

def move_to_position(x, y, z):
    try:
        base_angle, shoulder_angle, elbow_angle = calculate_inverse_kinematics(x, y, z)
        # Constrain angles to their limits
        base_angle = np.clip(base_angle, BASE_MIN_ANGLE, BASE_MAX_ANGLE)
        shoulder_angle = np.clip(shoulder_angle, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE)
        elbow_angle = np.clip(elbow_angle, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE)

        # Log the angles
        print(f"Moving to (x={x}, y={y}, z={z})")
        print(f"Calculated angles -> Base: {base_angle}, Shoulder: {shoulder_angle}, Elbow: {elbow_angle}")

        command = f"P{int(base_angle)},{int(shoulder_angle)},{int(elbow_angle)},90,90,73,150\n"
        response = send_command(command)
        if response == "E1":
            print("Error: Invalid command or position.")
    except ValueError as e:
        print(e)


def move_to_servo_angles(base, shoulder, elbow):
    base = np.clip(base, BASE_MIN_ANGLE, BASE_MAX_ANGLE)
    shoulder = np.clip(shoulder, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE)
    elbow = np.clip(elbow, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE)

    command = f"P{base},{shoulder},{elbow},90,90,73,150\n"
    send_command(command)


def main_menu():
    while True:
        print("Select an action:")
        print("0. Power Off")
        print("1. Power On")
        print("2. Home Position")
        print("3. Move end effector to origin")
        print("4. Move all joints")
        print("5. Move to position via x, y, z")
        print("6. Move position via servo angle input")

        choice = input("Enter choice: ")

        if choice == '0':
            power_off()
        elif choice == '1':
            power_on()
        elif choice == '2':
            home_position()
        elif choice == '3':
            move_to_origin()
        elif choice == '4':
            move_all_joints()
        elif choice == '5':
            x = float(input("Enter x: "))
            y = float(input("Enter y: "))
            z = float(input("Enter z: "))
            move_to_position(x, y, z)
        elif choice == '6':
            base = int(input("Enter base angle: "))
            shoulder = int(input("Enter shoulder angle: "))
            elbow = int(input("Enter elbow angle: "))
            move_to_servo_angles(base, shoulder, elbow)
        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    power_on()
    home_position()
    main_menu()
