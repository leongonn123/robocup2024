#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pyrealsense2 as rs


class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')
        
        self.obstacle_distance_threshold = 3.5  # meters, adjust as needed
        self.twist = Twist()
        self.turn_direction_locked = None  # To lock the turning direction
        self.obstacle_avoided = False  # New state to track obstacle avoidance

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.obstacle_avoided_pub = rospy.Publisher('/obstacle_avoided', Bool, queue_size=10)
        self.obstacle_detected_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=10)

        self.obstacle_detected_pub.publish(Bool(data=False))

        rospy.on_shutdown(self.cleanup)
        
        self.main_loop()

    def cleanup(self):
        cv2.destroyAllWindows()
        self.pipeline.stop()

    def main_loop(self):
        try:
            while not rospy.is_shutdown():
                # Get frameset of depth
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue
                
                # Convert depth frame to numpy array
                depth_image = np.asanyarray(depth_frame.get_data())

                # Process the depth data
                self.depth_callback(depth_image)

                # Check if 'q' was pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo("Quitting due to 'q' key press")
                    rospy.signal_shutdown("User requested shutdown")
                    break

        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")
            self.cleanup()

    def depth_callback(self, depth_image):
        # Filter out invalid depth values
        depth_image = np.where(depth_image == 0, np.nan, depth_image)
        
        # Normalize depth values for visualization (optional for debugging)
        depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = np.uint8(depth_display)

        # Display the depth image
        cv2.imshow("Depth Image", depth_display)
        cv2.waitKey(1)

        self.avoid_obstacle(depth_image)

    def avoid_obstacle(self, depth_array):
        rospy.loginfo("Avoiding obstacle...")
        
        # Segment the depth image into left, center, and right sections
        height, width = depth_array.shape
        left_section = depth_array[:, :width//3]
        center_section = depth_array[:, width//3: 2*width//3]
        right_section = depth_array[:, 2*width//3:]
        
        # Calculate average distances in each section, ignoring NaNs
        avg_left_dist = np.nanmean(left_section) / 1000.0  # Convert to meters
        avg_center_dist = np.nanmean(center_section) / 1000.0  # Convert to meters
        avg_right_dist = np.nanmean(right_section) / 1000.0  # Convert to meters
        
        rospy.loginfo(f"Distances (in meters) - Left: {avg_left_dist:.2f}, Center: {avg_center_dist:.2f}, Right: {avg_right_dist:.2f}")
        
        # Determine the closest section and decide on the action
        closest_distance = min(avg_left_dist, avg_center_dist, avg_right_dist)
        rospy.loginfo(f"Closest distance (in meters): {closest_distance:.2f}")

        if closest_distance < self.obstacle_distance_threshold:
            self.obstacle_detected_pub.publish(Bool(data=True))
            rospy.loginfo('Obstacle detected, changing direction!')
            self.twist.linear.x = 0.0  # Stop forward motion immediately

            # Lock direction to avoid oscillation
            if self.turn_direction_locked is None:
                if closest_distance == avg_center_dist:
                    # If obstacle directly ahead, choose the direction with more space
                    if avg_left_dist > avg_right_dist:
                        rospy.loginfo("Obstacle ahead, locking direction to left")
                        self.turn_direction_locked = 'left'
                    else:
                        rospy.loginfo("Obstacle ahead, locking direction to right")
                        self.turn_direction_locked = 'right'
                elif closest_distance == avg_right_dist:
                    rospy.loginfo("Obstacle on the right, locking direction to left")
                    self.turn_direction_locked = 'left'
                else:
                    rospy.loginfo("Obstacle on the left, locking direction to right")
                    self.turn_direction_locked = 'right'

            # Apply the locked direction
            if self.turn_direction_locked == 'left':
                rospy.loginfo("Steering left to avoid obstacle")
                self.twist.angular.z = 1.7  # Turn left

            elif self.turn_direction_locked == 'right':
                rospy.loginfo("Steering right to avoid obstacle")
                self.twist.angular.z = -1.7 # Turn right

            self.obstacle_avoided = False  # Set obstacle avoidance to false since we're still avoiding

        else:
            # If no obstacle is detected, keep moving forward
            rospy.loginfo("Path clear, moving forward")
            self.twist.linear.x = 0.1  # Move forward at a constant speed
            self.twist.angular.z = 0.0  # No turning
            self.turn_direction_locked = None  # Unlock direction when path is clear

            self.obstacle_avoided_pub.publish(Bool(data=True))
        
        # Publish the velocity commands
        self.cmd_vel_pub.publish(self.twist)


if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceNode()  # Instantiate the class
    except rospy.ROSInterruptException:
        pass
