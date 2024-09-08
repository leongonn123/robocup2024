#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2

class ObstacleAvoidance:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_avoidance', anonymous=True)

        # Publisher for velocity commands
        #self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Subscriber to the point cloud topic and obstacle detected flag
        self.point_cloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.point_cloud_callback)
        self.obstacle_detected_sub = rospy.Subscriber('/barrier_detected', Bool, self.obstacle_detected_callback)

        # Internal state
        self.latest_point_cloud = None
        self.obstacle_distance_threshold = 0.6  # Threshold set to 0.5 meters
        self.obstacle_present = False
        self.twist = Twist()

    def obstacle_detected_callback(self, msg):
        # Update obstacle presence state
        self.obstacle_present = msg.data

    #def point_cloud_callback(self, point_cloud_msg):
        # Update the latest point cloud
        #self.latest_point_cloud = point_cloud_msg

        # Process the latest point cloud if an obstacle is detected
        #if self.obstacle_present:
            #self.avoid_obstacle_point_cloud()

    #def avoid_obstacle_point_cloud(self):
        #if self.latest_point_cloud is None:
            #rospy.logwarn("No point cloud data to process")
            #return

        # Convert point cloud to numpy array
        pc_array = np.array(list(pc2.read_points(self.latest_point_cloud, skip_nans=True, field_names=("x", "y", "z"))))
        #rospy.loginfo(f"Point cloud array shape: {pc_array.shape}")

        # If the point cloud array is empty, assume no obstacles and continue moving
        if pc_array.size == 0:
            #rospy.loginfo("Empty point cloud received, assuming no obstacles detected")
            self.move_forward_continuously()
            return

        # Process the point cloud if it's not empty
        close_obstacles = pc_array[pc_array[:, 2] < self.obstacle_distance_threshold]
        #rospy.loginfo(f"Found {len(close_obstacles)} close obstacles")

        if len(close_obstacles) > 0:
            # Stop the robot if obstacles are detected
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            rospy.loginfo("Obstacle detected within 0.5 meters: Stopping.")
        else:
            # Continue moving forward if no close obstacles
            self.move_forward_continuously()

        # Publish the velocity command
        #self.cmd_vel_pub.publish(self.twist)

    def move_forward_continuously(self):
        """Set the robot to move forward continuously."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    #def start_moving(self):
    #"""Main loop for continuous movement."""
        #rate = rospy.Rate(10)  # 10 Hz
        #while not rospy.is_shutdown():
            #if not self.obstacle_present:
                #self.move_forward_continuously()
            #rate.sleep()

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    #oa.start_moving()
    rospy.spin()

