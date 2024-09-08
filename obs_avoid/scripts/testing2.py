#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class OdomStorageNode:
    def __init__(self):
        rospy.init_node('odom_storage_node', anonymous=True)
        
        # Subscribe to the odometry and obstacle detected topics
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.obstacle_sub = rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_callback)
        
        # Publisher for the current odometry when an obstacle is detected
        self.odom_pub = rospy.Publisher('/current_odom', Odometry, queue_size=10)
        
        self.current_odom = None  # To store the current odometry
        self.obstacle_detected = False  # Flag to check if the obstacle was already detected

    def odom_callback(self, data):
        # Update current odometry whenever new data comes in
        self.current_odom = data

    def obstacle_callback(self, msg):
        # Handle the detection of an obstacle
        if msg.data:
            if not self.obstacle_detected and self.current_odom is not None:
                # Publish odometry only if it wasn't already published for this obstacle
                # rospy.loginfo("Obstacle detected. Publishing current odometry:")
                self.odom_pub.publish(self.current_odom)
                self.obstacle_detected = True  # Set flag to true after publishing
        else:
            # Reset the flag when there is no obstacle detected
            self.obstacle_detected = False

if __name__ == '__main__':
    try:
        node = OdomStorageNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

