#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# Global variable to store the current yaw (orientation)
current_yaw = 0.0
target_angle = math.radians(90)  # 90 degrees in radians
initial_yaw = None

# Callback function to handle odometry data and extract yaw
def odom_callback(msg):
    global current_yaw, initial_yaw

    # Extract orientation quaternion from the odometry message
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    # Convert quaternion to Euler angles
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    # Set the initial yaw if it's not set
    if initial_yaw is None:
        initial_yaw = yaw

    # Store the current yaw
    current_yaw = yaw

def rotate_turtlebot():
    global current_yaw, target_angle, initial_yaw

    # Initialize the ROS node
    rospy.init_node('turtlebot_rotate_90_node', anonymous=True)

    # Publisher for velocity commands to control TurtleBot
    pub_cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # Subscriber to the /odom topic to get the robot's orientation
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Set the rate for the loop
    rate = rospy.Rate(10)  # 10 Hz

    # Command to rotate the TurtleBot
    twist_cmd = Twist()
    twist_cmd.angular.z = 0.5  # Rotate counterclockwise at 0.5 rad/s (adjust if needed)

    # Command to stop the TurtleBot
    stop_cmd = Twist()

    rospy.loginfo("Starting to rotate TurtleBot...")

    while not rospy.is_shutdown():
        # Check if the robot has rotated 90 degrees
        if initial_yaw is not None:
            # Calculate the difference between the current yaw and the initial yaw
            yaw_diff = current_yaw - initial_yaw

            # Handle the wrap-around case for yaw (angles around ±pi)
            if yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            elif yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi

            # Check if we've rotated by at least 90 degrees
            if abs(yaw_diff) >= target_angle:
                rospy.loginfo("Reached 90 degrees, stopping TurtleBot.")
                pub_cmd_vel.publish(stop_cmd)  # Stop the TurtleBot
                break

        # Keep rotating the TurtleBot
        pub_cmd_vel.publish(twist_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        rotate_turtlebot()
    except rospy.ROSInterruptException:
        pass
