#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import serial
import time
import math

# Initialize the serial port
port = "/dev/ttyUSB0"  # define particular port used
ser = serial.Serial(port, 1000000)  # define serial port
ser.close()  # close port if previously open
ser.open()  # open port
print(ser.isOpen())  # make sure we could open the port!

# Initialize ROS node and publisher
rospy.init_node('servo_controller', anonymous=True)
node_complete_pub = rospy.Publisher('/node_complete', Bool, queue_size=10)

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

# Define functions to calculate the speed values
def SpeedValH(speed):
    if speed > 256:
        return int(math.floor(speed / 256))
    else:
        return 0

def SpeedValL(speed):
    if speed < 256:
        return int(speed)
    else:
        more = int(math.floor(speed / 256))
        return int(speed - more * 256)

def checksum(id, length, Write_instruction, address, paramL, paramH):
    CHECKSUM = id + length + Write_instruction + address + paramL + paramH
    more = int(math.floor(CHECKSUM / 256))
    return 255 - int(math.floor(CHECKSUM - more * 256))

start = [255, 255]
length = 5
Write_instruction = 3

# Function to write the speed to a servo
def set_speed(id, speed):
    SpeedH = SpeedValH(speed)
    SpeedL = SpeedValL(speed)
    CHECKSUM = checksum(id, length, Write_instruction, 32, SpeedL, SpeedH)
    print(f"ID {id}: Setting speed to {speed} with checksum = {CHECKSUM}")
    ser.write("".join(map(chr, start + [id, length, Write_instruction, 32, SpeedL, SpeedH, CHECKSUM])).encode('latin-1'))

# Function to write the position to a servo
def write_position(id, angle):
    DegH = GoalPositionValH(angle)
    DegL = GoalPositionValL(angle)
    CHECKSUM = checksum(id, length, Write_instruction, 30, DegL, DegH)
    print(f"ID {id}: Moving to {angle} degrees with checksum = {CHECKSUM}")
    return "".join(map(chr, start + [id, length, Write_instruction, 30, DegL, DegH, CHECKSUM]))

# Function to move the servo to the specified angle with a specified speed
def move_servo(id, angle, speed=128):
    set_speed(id, speed)
    time.sleep(0.1)  # Slight delay to ensure speed is set
    ser.write(write_position(id, angle).encode('latin-1'))
    time.sleep(0.5)  # Increase the delay for slower movement

# Function to control the gripper with angle constraint
def control_gripper(id, angle, speed=128):
    if 140 <= angle <= 200:
        move_servo(id, angle, speed)
    else:
        print(f"Gripper angle {angle} is out of range. Must be between 140 and 180 degrees.")

try:
    # Initialize positions
    #control_gripper(1, 180, speed=64)  # Initialize gripper to 180 degrees with very slow speed
    #move_servo(6, 40, speed=128)  # Initialize joint to 40 degrees with slow speed
    #move_servo(3, 220, speed=128)  # Initialize base to 260 degrees with slow speed
    
    #rospy.sleep(2)
    
    control_gripper(1, 140, speed=80)
    
    move_servo(6, 120, speed=128)

    #move_servo(3, 80, speed=100)
    
    rospy.sleep(0.5)
    
    move_servo(3, 130, speed=100)
    #move_servo(6, 80, speed=128)
    
    rospy.sleep(3)
    
    control_gripper(1, 180, speed=80)
    
    rospy.sleep(1)

    control_gripper(1, 140, speed=80)

    rospy.sleep(1)

    control_gripper(1, 180, speed=80)

    rospy.sleep(1)

    control_gripper(1, 140, speed=80)

    rospy.sleep(1)
    
    control_gripper(1, 180, speed=80)

    rospy.sleep(1)

    move_servo(3, 180, speed=100)

    rospy.sleep(1)

    move_servo(6, 20, speed=100)

    #move_servo(6, 80, speed=128)
    #move_servo(3, 160, speed=128)
    
    # Move gripper from 180 to 140 degrees with slower speed
    # control_gripper(1, 140, speed=80)
    
    # Move joint (ID 6) to 200 degrees with slower speed, then to 90 degrees
    #move_servo(6, 300, speed=240)
    #move_servo(6, 90, speed=128)

    # Move base (ID 3) from 260 to 210 degrees with slower speed
    #move_servo(3, 210, speed=100)

    # Return to initial positions
    #control_gripper(1, 180, speed=128)  # Gripper back to 180 degrees with slow speed
    #move_servo(3, 260, speed=128)  # Base back to 260 degrees with slow speed
    #move_servo(6, 40, speed=128)  # Joint back to 40 degrees with slow speed

    # Publish True to indicate that the sequence is complete
    node_complete_pub.publish(True)
    rospy.loginfo("Published completion message to /node_complete")

except KeyboardInterrupt:
    print("Stopping the servo control")

finally:
    ser.close()  # Close the serial port when done
