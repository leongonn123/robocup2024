#!/usr/bin/env python3

import serial
import time
import math

# Initialize the serial port
port = "/dev/ttyUSB0"  # define particular port used
ser = serial.Serial(port, 1000000)  # define serial port
ser.close()  # close port if previously open
ser.open()  # open port
print(ser.isOpen())  # make sure we could open the port!

# Define functions to calculate the Goal Position
def GoalPositionValH(angle):
    if math.floor(angle * 3.41) > 256:
        return int(math.floor(angle * 3.41 / 256))
    else:
        return 0

def GoalPositionValL(angle):
    if math.floor(angle * 3.41) < 256:
        return int(math.floor(angle * 3.41))
    else:
        more = int(math.floor(angle * 3.41 / 256))
        return int(math.floor(angle * 3.41 - more * 256))

def checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH):
    CHECKSUM = id + length + Write_instruction + GoalPositionAddress + DegL + DegH
    more = int(math.floor(CHECKSUM / 256))
    return 255 - int(math.floor(CHECKSUM - more * 256))

start = [255, 255]
id = 2
length = 5
Write_instruction = 3
GoalPositionAddress = 30

# Function to write the position to a servo
def write_position(angle):
    DegH = GoalPositionValH(angle)
    DegL = GoalPositionValL(angle)
    CHECKSUM = checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH)
    print("checksum =", CHECKSUM)
    return "".join(map(chr, start + [id, length, Write_instruction, GoalPositionAddress, DegL, DegH, CHECKSUM]))

# Function to convert Degrees to Radians
def Deg2Rad(Deg):
    return Deg * 3.141592654 / 180

# Function to convert Radians to Degrees
def Rad2Deg(Rad):
    return math.floor(Rad * 180 / 3.141592654)

center_position = 45
clockwise_position = center_position + 90
counterclockwise_position = center_position - 90

period = 1  # 1 second delay between movements

try:
    # Define your target angle here
    target_position = 150  # Example: move to 240 degrees

    # Ensure the target position is within the valid range for the AX-12A servo
    if 0 <= target_position <= 300:
        # Determine the direction and move step by step
        step_size = 1  # Adjust the step size for smooth movement
        if target_position > center_position:
            for pos in range(center_position, target_position + 1, step_size):
                ser.write(write_position(pos).encode('latin-1'))
                print(f"Moving to {pos} degrees")
                time.sleep(0.01)  # Adjust the delay for smoother or faster movement
        elif target_position < center_position:
            for pos in range(center_position, target_position - 1, -step_size):
                ser.write(write_position(pos).encode('latin-1'))
                print(f"Moving to {pos} degrees")
                time.sleep(0.01)  # Adjust the delay for smoother or faster movement

        # Update the current position
        center_position = target_position

    else:
        print("Invalid target angle. Please set an angle between 0 and 300 degrees.")

except KeyboardInterrupt:
    # Stop the script when Ctrl+C is pressed
    print("Stopping the servo control")

finally:
    ser.close()  # Close the serial port when done

