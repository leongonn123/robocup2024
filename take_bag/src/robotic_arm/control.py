#!/usr/bin/env python3

import serial
import time
import sys
import math

port = "/dev/ttyUSB0"  # define particular port used
ser = serial.Serial(port, 1000000)  # define serial port
ser.close()  # close port if previously open
ser.open()  # open port
print(ser.isOpen())  # make sure we could open the port!

def GoalPositionValH(angle):
    if math.floor(angle*3.41) > 256:
        return int(math.floor(angle*3.41/256))
    else:
        return 0

def GoalPositionValL(angle):
    if math.floor(angle*3.41) < 256:
        return int(math.floor(angle*3.41))
    else:
        more = int(math.floor(angle*3.41/256))
        return int(math.floor(angle*3.41 - more*256))

def checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH):
    CHECKSUM = id + length + Write_instruction + GoalPositionAddress + DegL + DegH
    more = int(math.floor(CHECKSUM / 256))
    return 255 - int(math.floor(CHECKSUM - more * 256))

start = [255, 255]
id = 1
length = 5
Write_instruction = 3
GoalPositionAddress = 30

def write_position(angle):
    DegH = GoalPositionValH(angle)
    DegL = GoalPositionValL(angle)
    CHECKSUM = checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH)
    print("checksum =", CHECKSUM)
    return "".join(map(chr, start + [id, length, Write_instruction, GoalPositionAddress, DegL, DegH, CHECKSUM]))

def Deg2Rad(Deg):
    return Deg * 3.141592654 / 180

def Rad2Deg(Rad):
    return math.floor(Rad * 180 / 3.141592654)

center_position = 150
clockwise_position = center_position + 90
counterclockwise_position = center_position - 90

period = 1  # 1 second delay between movements

try:
    while True:
        # Move to 240 degrees (clockwise)
        ser.write(write_position(clockwise_position).encode('latin-1'))
        print(f"Moving to {clockwise_position} degrees (clockwise)")
        time.sleep(period)

        # Move to 60 degrees (counterclockwise)
        ser.write(write_position(counterclockwise_position).encode('latin-1'))
        print(f"Moving to {counterclockwise_position} degrees (counterclockwise)")
        time.sleep(period)

except KeyboardInterrupt:
    # Stop the script when Ctrl+C is pressed
    print("Stopping the servo control")

finally:
    ser.close()  # Close the serial port when done

