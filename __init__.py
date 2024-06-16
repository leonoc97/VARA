import serial
import time
import math
import numpy as np

# Configure the serial port and baud rate
ser = serial.Serial('COM7', 115200, timeout=1)


def send_command(command):
    ser.write((command + '\n').encode())
    time.sleep(1)
    response = ser.readline().decode().strip()
    return response


def calibrate_servo(base_rad, shoulder_rad, elbow_rad):
    wrist_rot = 90  # default value
    wrist_pitch = 90  # default value
    gripper = 73  # default value
    speed = 150  # default speed

    # Convert radians to degrees and take absolute values
    base = abs(math.degrees(base_rad))
    shoulder = abs(math.degrees(shoulder_rad))
    elbow = abs(math.degrees(elbow_rad))

    # Send calibration command
    command = f'P{int(base)},{int(shoulder)},{int(elbow)},{wrist_rot},{wrist_pitch},{gripper},{speed}'
    response = send_command(command)
    print(f'Sent: {command}, Received: {response}')


def home_position():
    response = send_command('H')
    print(f'Sent: H, Received: {response}')


def power_off():
    response = send_command('0')
    print(f'Sent: 0, Received: {response}')


def power_on():
    response = send_command('1')
    print(f'Sent: 1, Received: {response}')


# DH Parameters
DH_params = [
    {'alpha': 1.57, 'a': 42.00, 'd': 46.50, 'theta_offset': 0.3},
    {'alpha': 0.00, 'a': 95.00, 'd': 0.00, 'theta_offset': 1.70},
    {'alpha': 0.00, 'a': 165.00, 'd': 2.00, 'theta_offset': 3.14}
]

BASE_HEIGHT = 0  # Base height in mm

# Measurements for the parallelogram linkage
h1 = 26.19
h2 = 102.88
V1 = 95
h3 = 33

# Convert degrees to radians
DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi


# Calculate shoulder and elbow angles
def calculate_angles(beta):
    beta_rad = beta * DEG_TO_RAD

    diag1 = math.sqrt(V1 ** 2 + h2 ** 2 - 2 * V1 * h1 * math.cos(beta_rad) + h1 ** 2)
    print(f"diag1: {diag1}")

    value1 = (-h2 ** 2 + h3 ** 2 + h1 ** 2 + V1 ** 2 - diag1 ** 2) / (2 * h3 * diag1)
    value2 = (-h3 ** 2 + h1 ** 2 + V1 ** 2 + diag1 ** 2) / (2 * V1 * diag1)

    # Ensure values are within the range [-1, 1]
    value1 = min(1, max(-1, value1))
    value2 = min(1, max(-1, value2))

    alpha1 = math.acos(value1)
    alpha2 = math.acos(value2)
    alpha = (alpha1 + alpha2 - math.pi / 2) * RAD_TO_DEG

    return alpha


# Forward kinematics
def forward_kinematics(theta1, theta2, theta3):
    # Convert degrees to radians
    theta1 = theta1 * DEG_TO_RAD
    theta2 = theta2 * DEG_TO_RAD
    theta3 = theta3 * DEG_TO_RAD

    T = np.eye(4)
    for i, param in enumerate(DH_params):
        alpha = param['alpha']
        a = param['a']
        d = param['d']
        theta = param['theta_offset'] + [theta1, theta2, theta3][i]

        T_i = np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
            [0, 0, 0, 1]
        ])

        T = np.dot(T, T_i)

    return T


# Inverse kinematics
def inverse_kinematics(x, y, z):
    # Adjust for base height of 150mm
    z = z - BASE_HEIGHT

    # Simplified inverse kinematics for a planar 3DOF arm
    theta1 = np.arctan2(y, x)
    r = np.sqrt(x ** 2 + y ** 2)
    z_prime = z - DH_params[0]['d']
    r_prime = r - DH_params[0]['a']

    D = (r_prime ** 2 + z_prime ** 2 - DH_params[1]['a'] ** 2 - DH_params[2]['a'] ** 2) / (
                2 * DH_params[1]['a'] * DH_params[2]['a'])
    if D < -1 or D > 1:
        raise ValueError("Invalid position: D out of range")

    theta3 = np.arctan2(np.sqrt(1 - D ** 2), D)
    theta2 = np.arctan2(z_prime, r_prime) - np.arctan2(DH_params[2]['a'] * np.sin(theta3),
                                                       DH_params[1]['a'] + DH_params[2]['a'] * np.cos(theta3))

    # Apply theta offsets
    theta1_offset = theta1 + DH_params[0]['theta_offset']
    theta2_offset = theta2 + DH_params[1]['theta_offset']
    theta3_offset = theta3 + DH_params[2]['theta_offset']

    # Calculate the correct shoulder angle using the parallelogram linkage
    shoulder_angle = calculate_angles(theta2 * RAD_TO_DEG)

    # Calculate theta1 with an additional offset of PI
    theta1_pi_offset = theta1 + math.pi
    theta1_pi_offset_with_additional = theta1_pi_offset + DH_params[0]['theta_offset']

    return (theta1, shoulder_angle * DEG_TO_RAD, theta3), (theta1_offset, theta2_offset, theta3_offset), (
    theta1_pi_offset, shoulder_angle * DEG_TO_RAD, theta3), (
    theta1_pi_offset_with_additional, theta2_offset, theta3_offset)


def main():
    while True:
        print("\nServo Calibration Menu")
        print("1. Calibrate Servos (Base, Shoulder, Elbow)")
        print("2. Move to Home Position")
        print("3. Power Off")
        print("4. Power On")
        print("5. Calculate and Move to Position (x, y, z)")
        print("6. Exit")

        choice = input("Enter your choice: ")

        if choice == '1':
            base_rad = float(input("Enter base angle in radians (0-π): "))
            shoulder_rad = float(input("Enter shoulder angle in radians (0.26-2.88): "))
            elbow_rad = float(input("Enter elbow angle in radians (0-π): "))
            calibrate_servo(base_rad, shoulder_rad, elbow_rad)
        elif choice == '2':
            home_position()
        elif choice == '3':
            power_off()
        elif choice == '4':
            power_on()
        elif choice == '5':
            x = float(input("Enter x position (mm): "))
            y = float(input("Enter y position (mm): "))
            z = float(input("Enter z position (mm): "))
            try:
                (base_rad, shoulder_rad, elbow_rad), (base_offset, shoulder_offset, elbow_offset), (
                base_pi_offset, shoulder_pi_offset, elbow_pi_offset), (
                base_pi_offset_with_additional, shoulder_offset, elbow_offset) = inverse_kinematics(x, y, z)
                print(
                    f"Calculated theta angles (radians): Base: {base_rad}, Shoulder: {shoulder_rad}, Elbow: {elbow_rad}")
                print(
                    f"Calculated theta angles with offsets (radians): Base: {base_offset}, Shoulder: {shoulder_offset}, Elbow: {elbow_offset}")
                print(
                    f"Calculated theta angles with base offset of PI (radians): Base: {base_pi_offset}, Shoulder: {shoulder_pi_offset}, Elbow: {elbow_pi_offset}")
                print(
                    f"Calculated theta angles with base offset of PI and additional offsets (radians): Base: {base_pi_offset_with_additional}, Shoulder: {shoulder_offset}, Elbow: {elbow_offset}")

                calibrate_servo(base_rad, shoulder_rad, elbow_rad)

                # Forward kinematics for absolute position with offsets
                T_with_offsets = forward_kinematics(base_offset * RAD_TO_DEG, shoulder_offset * RAD_TO_DEG,
                                                    elbow_offset * RAD_TO_DEG)
                print(f"End-effector position with offsets: {T_with_offsets[:3, 3]}")

                # Forward kinematics for absolute position without offsets
                T_without_offsets = forward_kinematics(base_rad * RAD_TO_DEG, shoulder_rad * RAD_TO_DEG,
                                                       elbow_rad * RAD_TO_DEG)
                print(f"End-effector position without offsets: {T_without_offsets[:3, 3]}")

                # Forward kinematics for absolute position with base offset of PI
                T_with_pi_offset = forward_kinematics(base_pi_offset * RAD_TO_DEG, shoulder_pi_offset * RAD_TO_DEG,
                                                      elbow_pi_offset * RAD_TO_DEG)
                print(f"End-effector position with base offset of PI: {T_with_pi_offset[:3, 3]}")

                # Forward kinematics for absolute position with base offset of PI and additional offsets
                T_with_pi_and_additional_offset = forward_kinematics(base_pi_offset_with_additional * RAD_TO_DEG,
                                                                     shoulder_offset * RAD_TO_DEG,
                                                                     elbow_offset * RAD_TO_DEG)
                print(
                    f"End-effector position with base offset of PI and additional offsets: {T_with_pi_and_additional_offset[:3, 3]}")

            except ValueError as e:
                print(f"Error: {e}")
        elif choice == '6':
            ser.close()
            break
        else:
            print("Invalid choice. Please try again.")


if __name__ == '__main__':
    main()
