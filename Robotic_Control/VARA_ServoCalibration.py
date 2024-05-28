import serial
import time
import math

# Configure the serial port and baud rate
ser = serial.Serial('COM7', 115200, timeout=1)

def send_command(command):
    ser.write((command + '\n').encode())
    time.sleep(1)
    response = ser.readline().decode().strip()
    return response

def calibrate_servo(base_rad, shoulder_rad, elbow_rad):
    wrist_rot = 90        # default value
    wrist_pitch = 90      # default value
    gripper = 73          # default value
    speed = 150           # default speed

    # Convert radians to degrees
    base = math.degrees(base_rad)
    shoulder = math.degrees(shoulder_rad)
    elbow = math.degrees(elbow_rad)

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

def main():
    while True:
        print("\nServo Calibration Menu")
        print("1. Calibrate Servos (Base, Shoulder, Elbow)")
        print("2. Move to Home Position")
        print("3. Power Off")
        print("4. Power On")
        print("5. Exit")

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
            ser.close()
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == '__main__':
    main()
