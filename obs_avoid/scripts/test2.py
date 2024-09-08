#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pyrealsense2 as rs
import time
import threading


class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')

        # Parameters
        self.obstacle_distance_threshold = 3.5  # meters, adjust as needed
        self.max_linear_speed = 0.15  # max forward speed
        self.min_linear_speed = 0.05  # min forward speed
        self.max_angular_speed = 1.0  # max turning speed
        self.acceleration = 0.01  # linear acceleration increment
        self.angular_acceleration = 0.05  # angular acceleration increment
        self.forward_move_duration = 3.0  # seconds to move forward after avoiding an obstacle

        # Initialize speeds
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.turn_direction_locked = None  # To lock the turning direction
        self.path_clear_time = None  # Track time when path is first clear
        self.clearance_delay = 1.0  # 1-second delay after path is clear
        self.accumulated_turn_angle = 0.0  # Track the accumulated turn angle

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.obstacle_avoided_pub = rospy.Publisher('/obstacle_avoided', Bool, queue_size=10)
        self.obstacle_detected_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=10)

        # Threads
        self.stop_movement_flag = False  # Flag to stop movement when necessary
        self.movement_thread = threading.Thread(target=self.movement_loop)
        self.movement_thread.daemon = True  # Ensure the thread exits when the node shuts down

        rospy.on_shutdown(self.cleanup)

        # Start threads
        self.movement_thread.start()

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

        # Display the depth image (optional for debugging)
        depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = np.uint8(depth_display)
        cv2.imshow("Depth Image", depth_display)
        cv2.waitKey(1)

        self.avoid_obstacle(depth_image)

    def avoid_obstacle(self, depth_array):
        rospy.loginfo("Avoiding obstacle...")

        # Segment the depth image into left, center, and right sections
        height, width = depth_array.shape
        left_section = depth_array[:, :width // 3]
        center_section = depth_array[:, width // 3: 2 * width // 3]
        right_section = depth_array[:, 2 * width // 3:]

        # Calculate average distances in each section, ignoring NaNs
        avg_left_dist = np.nanmean(left_section) / 1000.0  # Convert to meters
        avg_center_dist = np.nanmean(center_section) / 1000.0  # Convert to meters
        avg_right_dist = np.nanmean(right_section) / 1000.0  # Convert to meters

        # Determine the closest section
        closest_distance = min(avg_left_dist, avg_center_dist, avg_right_dist)

        if closest_distance < self.obstacle_distance_threshold:
            self.obstacle_detected_pub.publish(Bool(data=True))
            rospy.loginfo('Obstacle detected, changing direction!')

            # Lock the turn direction based on the closest obstacle
            if self.turn_direction_locked is None:
                if closest_distance == avg_center_dist:
                    self.turn_direction_locked = 'left' if avg_left_dist > avg_right_dist else 'right'
                elif closest_distance == avg_right_dist:
                    self.turn_direction_locked = 'left'
                else:
                    self.turn_direction_locked = 'right'

            # Turn in the locked direction
            angular_velocity = self.max_angular_speed if self.turn_direction_locked == 'left' else -self.max_angular_speed
            self.current_angular_speed = angular_velocity

            # Stop moving forward
            self.current_linear_speed = 0.0

            # Signal to stop the movement thread
            self.stop_movement_flag = True

        else:
            rospy.loginfo("Path clear, moving forward")

            # Path is clear, move forward immediately without delay
            self.current_linear_speed = self.max_linear_speed
            self.current_angular_speed = 0.0

            # Unlock the turn direction
            self.turn_direction_locked = None

            self.obstacle_avoided_pub.publish(Bool(data=True))

            # Signal to resume movement
            self.stop_movement_flag = False

        # Publish the velocity commands
        self.publish_velocity()

    def movement_loop(self):
        while not rospy.is_shutdown():
            # Check if movement should be stopped due to an obstacle
            if self.stop_movement_flag:
                rospy.loginfo("Movement paused due to obstacle.")
                rospy.sleep(0.1)  # Sleep for a bit and check again
                continue

            # Continue forward movement if not stopped
            self.move_forward()

    def move_forward(self):
        rospy.loginfo("Moving forward to clear the obstacle")
        start_time = time.time()
        while time.time() - start_time < self.forward_move_duration:
            if self.stop_movement_flag:
                rospy.loginfo("Stopping forward movement due to an obstacle")
                break  # Stop moving if an obstacle is detected

            twist = Twist()
            twist.linear.x = self.max_linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)  # Short delay to simulate continuous movement

        # Stop forward movement
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.current_linear_speed
        twist.angular.z = self.current_angular_speed
        self.cmd_vel_pub.publish(twist)

    def rotate_back_by_accumulated_angle(self):
        rospy.loginfo("Rotating back by the accumulated angle")

        if self.accumulated_turn_angle == 0.0:
            rospy.loginfo("No accumulated turn angle; skipping rotation.")
            return  # Skip rotation if no angle was accumulated

        # Rotate back by the accumulated angle in the opposite direction
        rotation_radians = -self.accumulated_turn_angle

        # Perform the rotation
        duration = abs(rotation_radians / self.max_angular_speed)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(duration):
            twist = Twist()
            twist.angular.z = self.max_angular_speed if rotation_radians > 0 else -self.max_angular_speed
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)

        # Stop rotation after completing the turn
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        rospy.loginfo("Completed rotation back by the accumulated angle")

        # Reset the accumulated turn angle after rotating back
        self.accumulated_turn_angle = 0.0


if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceNode()  # Instantiate the class
    except rospy.ROSInterruptException:
        pass
