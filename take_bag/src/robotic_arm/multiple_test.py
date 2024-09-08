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
length = 5
Write_instruction = 3
GoalPositionAddress = 30

# Function to write the position to a servo
def write_position(id, angle):
    DegH = GoalPositionValH(angle)
    DegL = GoalPositionValL(angle)
    CHECKSUM = checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH)
    print(f"Checksum for ID {id} =", CHECKSUM)
    return "".join(map(chr, start + [id, length, Write_instruction, GoalPositionAddress, DegL, DegH, CHECKSUM]))

# Function to convert Degrees to Radians
def Deg2Rad(Deg):
    return Deg * 3.141592654 / 180

# Function to convert Radians to Degrees
def Rad2Deg(Rad):
    return math.floor(Rad * 180 / 3.141592654)

# Set initial positions for servos
servo1_id = 1
servo2_id = 2
center_position = 150

# Define target positions for both servos
servo1_target_position = 240  # Servo 1 moves 90 degrees clockwise from 150
servo2_target_position = 60    # Servo 2 moves 180 degrees counterclockwise from 240

# Define time period between movements
period = 1  # 1 second delay between movements

try:
    # Move Servo 1 to 240 degrees (90 degrees clockwise)
    ser.write(write_position(servo1_id, servo1_target_position).encode('latin-1'))
    print(f"Servo 1 moving to {servo1_target_position} degrees (clockwise)")
    time.sleep(period)

    # Move Servo 2 to 60 degrees (180 degrees counterclockwise)
    ser.write(write_position(servo2_id, servo2_target_position).encode('latin-1'))
    print(f"Servo 2 moving to {servo2_target_position} degrees (counterclockwise)")
    time.sleep(period)

    # Return Servo 1 to its original position (150 degrees)
    ser.write(write_position(servo1_id, center_position).encode('latin-1'))
    print(f"Servo 1 returning to {center_position} degrees")
    time.sleep(period)

    # Return Servo 2 to its original position (150 degrees)
    ser.write(write_position(servo2_id, center_position).encode('latin-1'))
    print(f"Servo 2 returning to {center_position} degrees")
    time.sleep(period)

except KeyboardInterrupt:
    # Stop the script when Ctrl+C is pressed
    print("Stopping the servo control")

finally:
    ser.close()  # Close the serial port when done

