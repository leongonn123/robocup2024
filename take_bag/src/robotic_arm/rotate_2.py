#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from math import pi

class TurtleBotTurner:
    def __init__(self):
        # Initializes the ROS node
        rospy.init_node('turtlebot_turn_controller', anonymous=True)
        # Publisher to publish twist messages to the TurtleBot
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        # Publisher to publish a completion message
        self.complete_publisher = rospy.Publisher('/node_complete', Bool, queue_size=10)
        # Subscriber to subscribe to the odometry and distance reached topics
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.distance_subscriber = rospy.Subscriber('/rotate_2', Bool, self.reach_distance_callback)
        self.current_angle = 0
        self.start_angle = None
        self.total_angle_turned = 0
        self.rotating = False  # Flag to control rotation
        self.rotation_triggered = False  # Ensures rotation is only triggered once

    def reach_distance_callback(self, msg):
        # Trigger rotation only on the first True message
        if msg.data and not self.rotation_triggered:
            rospy.loginfo("Reached target distance. Rotating 180 degrees.")
            self.rotation_triggered = True  # Set flag to prevent re-triggering
            self.rotating = True
            self.turn_180_degrees()
    
    def turn_180_degrees(self):
        rate = rospy.Rate(10)  # 10hz
        velocity_msg = Twist()
        # Setting turning speed (rad/s)
        velocity_msg.angular.z = 0.5  # Adjust this value if necessary

        while not rospy.is_shutdown():
            if abs(self.total_angle_turned) >= (pi - 0.1):
                break  # Break the loop if 180 degrees turn is completed or exceeded
            self.velocity_publisher.publish(velocity_msg)
            rate.sleep()

        # Stopping the robot after turning 180 degrees
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)
        rospy.loginfo("Turn completed")
        self.rotating = False  # Reset rotating flag
        self.total_angle_turned = 0  # Reset angle turned

        # Publish True to indicate that the rotation is complete
        self.complete_publisher.publish(True)
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
