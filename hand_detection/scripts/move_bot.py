#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi

class TurtleBotTurner:
    def __init__(self):
        # Initializes the ROS node
        rospy.init_node('turtlebot_turn_controller', anonymous=True)

        # Publisher for TurtleBot velocity commands
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Publisher to indicate completion of the node's task
        self.complete_pub = rospy.Publisher('/node_complete', Bool, queue_size=1)

        # Subscriber to subscribe to the odometry and pointing hand topics
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pointing_hand_subscriber = rospy.Subscriber('/pointing_hand', Int32, self.pointing_hand_callback)

        self.current_angle = 0
        self.start_angle = None
        self.total_angle_turned = 0
        self.rotating = False  # Flag to control rotation
        self.has_rotated = False  # Ensures rotation is only triggered once
        self.twist = Twist()
        self.angular_speed = 0.5  # radians per second (tune this value based on your robot)
        self.rotation_direction = 1  # Direction of rotation (1 for clockwise, -1 for counterclockwise)

    def pointing_hand_callback(self, msg):
        if not self.has_rotated:
            rospy.loginfo("Received pointing direction: %d", msg.data)
            if msg.data == 1:
                # Set rotation direction to clockwise
                rospy.loginfo("Turning clockwise")
                self.rotation_direction = -1
            elif msg.data == 2:
                # Set rotation direction to anticlockwise
                rospy.loginfo("Turning anticlockwise")
                self.rotation_direction = 1

            self.has_rotated = True  # Set the flag to True after initiating the rotation
            self.rotating = True
            self.turn_45_degrees()

    def turn_45_degrees(self):
        rate = rospy.Rate(10)  # 10hz
        self.twist.angular.z = self.rotation_direction * self.angular_speed

        while not rospy.is_shutdown():
            if abs(self.total_angle_turned) >= pi/6:
                break  # Break the loop if 15 degrees turn is completed or exceeded
            self.velocity_pub.publish(self.twist)
            rate.sleep()

        # Stopping the robot after turning 45 degrees
        self.twist.angular.z = 0
        self.velocity_pub.publish(self.twist)
        rospy.loginfo("Rotation completed")
        self.rotating = False  # Reset rotating flag
        self.total_angle_turned = 0  # Reset angle turned

        # Publish True to indicate that the rotation is complete
        self.complete_pub.publish(True)
        rospy.loginfo("Published completion message to /node_complete")

    def odom_callback(self, data):
        # Callback function to get the robot's orientation
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        if self.start_angle is None:
            self.start_angle = yaw
            self.current_angle = yaw
            rospy.loginfo("Starting angle detected at: {:.2f}".format(self.start_angle))
        elif self.rotating:
            angle_difference = yaw - self.current_angle
            if angle_difference > pi:
                angle_difference -= 2 * pi
            elif angle_difference < -pi:
                angle_difference += 2 * pi

            self.total_angle_turned += angle_difference
            self.current_angle = yaw
            rospy.loginfo("Total angle turned: {:.2f} radians".format(self.total_angle_turned))

if __name__ == '__main__':
    try:
        turner = TurtleBotTurner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

