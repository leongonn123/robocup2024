#!/usr/bin/env python3

import rospy
import actionlib
import pyrealsense2 as rs
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        # Initialize state variables
        self.start = 1
        self.obstacle_detected = False
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize the RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream at 640x480 resolution, 30 FPS
        self.pipeline.start(self.config)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        if not self.move_base.wait_for_server(rospy.Duration(120)):
            rospy.logerr("Unable to connect to move_base action server.")
            rospy.signal_shutdown("Failed to connect to move_base action server.")
            return
        
        rospy.loginfo("Connected to move base server")
        
        rospy.loginfo("Ready to go")
        rospy.sleep(1)
    
        # Define the target location (point A)
        A_x = -2.74
        A_y = -0.0896
        A_theta = 0
        
        quaternion = quaternion_from_euler(0.0, 0.0, A_theta)
        self.target_location = Pose(Point(A_x, A_y, 0.0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        
        self.goal = MoveBaseGoal()
        rospy.loginfo("Starting navigation test")

        while not rospy.is_shutdown():
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            if self.start == 1:
                rospy.loginfo("Going to point A")
                rospy.sleep(2)
                self.goal.target_pose.pose = self.target_location
                self.move_base.send_goal(self.goal)

                while not self.move_base.wait_for_result(rospy.Duration(0.1)):
                    if self.detect_obstacle():
                        rospy.loginfo("Obstacle detected! Executing avoidance behavior...")
                        self.move_base.cancel_goal()
                        self.avoid_obstacle()
                        rospy.loginfo("Resuming navigation...")
                        self.move_base.send_goal(self.goal)
                
                if self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached point A")
                    rospy.sleep(2)
                    self.start = 2  # Stop after reaching point A

            rospy.Rate(5).sleep()

    def detect_obstacle(self):
        """
        Function to detect obstacles using the RealSense depth data (pyrealsense2).
        """
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            return False
        
        # Convert depth frame to numpy array (depth values are in millimeters)
        depth_image = np.asanyarray(depth_frame.get_data())

        # Image width and height
        image_height, image_width = depth_image.shape

        # Segment the depth image
        left_section = depth_image[:, :image_width // 3]
        center_section = depth_image[:, image_width // 3:(2 * image_width) // 3]
        right_section = depth_image[:, (2 * image_width) // 3:]

        # Filter out invalid depth values (zeros)
        left_section = left_section[left_section > 0]
        center_section = center_section[center_section > 0]
        right_section = right_section[right_section > 0]

        # Check if any of the sections are empty after filtering
        if left_section.size == 0:
            left_distance = float('inf')  # No valid data, treat as no obstacle
        else:
            left_distance = np.mean(left_section) / 1000.0  # Convert from mm to meters

        if center_section.size == 0:
            center_distance = float('inf')
        else:
            center_distance = np.mean(center_section) / 1000.0

        if right_section.size == 0:
            right_distance = float('inf')
        else:
            right_distance = np.mean(right_section) / 1000.0

        # Detect obstacle based on threshold distances (in meters)
        if center_distance < 0.5 or left_distance < 0.5 or right_distance < 0.5:
            return True  # Obstacle detected
        else:
            return False

    def avoid_obstacle(self):
        rospy.loginfo("Avoiding obstacle...")

        # Define twist message for movement
        twist = Twist()

        # Set rotation and movement speeds
        rotation_speed = 0.5  # radians/sec
        move_speed = 0.2      # meters/sec

        # Determine where the obstacle is (left, right, or center)
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            return
        
        depth_image = np.asanyarray(depth_frame.get_data())
        image_height, image_width = depth_image.shape
        
        left_section = depth_image[:, :image_width // 3]
        center_section = depth_image[:, image_width // 3:(2 * image_width) // 3]
        right_section = depth_image[:, (2 * image_width) // 3:]

        # Filter out invalid depth values (zeros)
        left_section = left_section[left_section > 0]
        center_section = center_section[center_section > 0]
        right_section = right_section[right_section > 0]

        # Check if any of the sections are empty after filtering
        if left_section.size == 0:
            left_distance = float('inf')  # No valid data, treat as no obstacle
        else:
            left_distance = np.mean(left_section) / 1000.0  # Convert from mm to meters

        if center_section.size == 0:
            center_distance = float('inf')
        else:
            center_distance = np.mean(center_section) / 1000.0

        if right_section.size == 0:
            right_distance = float('inf')
        else:
            right_distance = np.mean(right_section) / 1000.0

        obstacle_detected = False

        if center_distance < 0.5:  # Obstacle in front
            rospy.loginfo("Obstacle detected in front, turning left")
            twist.angular.z = rotation_speed  # Turn left
            obstacle_detected = True
        elif left_distance < 0.5:  # Obstacle on the left
            rospy.loginfo("Obstacle detected on the left, turning right")
            twist.angular.z = -rotation_speed  # Turn right
            obstacle_detected = True
        elif right_distance < 0.5:  # Obstacle on the right
            rospy.loginfo("Obstacle detected on the right, turning left")
            twist.angular.z = rotation_speed  # Turn left
            obstacle_detected = True

        if obstacle_detected:
            # Turn for 3 seconds
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(3)

            # Move straight for 3 seconds
            rospy.loginfo("Moving straight for 3 seconds")
            twist.angular.z = 0  # Stop rotating
            twist.linear.x = move_speed  # Move forward
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(3)

            # Turn in the opposite direction for 3 seconds
            rospy.loginfo("Turning in the opposite direction for 3 seconds")
            twist.angular.z = -twist.angular.z  # Reverse the previous turn direction
            twist.linear.x = 0  # Stop moving forward
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(3)

            # Stop the robot after avoidance
            rospy.loginfo("Obstacle avoidance complete, stopping the robot")
            twist.angular.z = 0
            twist.linear.x = 0
            self.cmd_vel_pub.publish(twist)

    def cleanup(self):
        rospy.loginfo("Shutting down navigation....")
        self.move_base.cancel_goal()
        self.pipeline.stop()

if __name__ == "__main__":
    rospy.init_node('navi_point', anonymous=True)
    try:
        NavToPoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
import actionlib
import pyrealsense2 as rs
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        # Initialize state variables
        self.start = 1
        self.obstacle_detected = False
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize the RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream at 640x480 resolution, 30 FPS
        self.pipeline.start(self.config)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        if not self.move_base.wait_for_server(rospy.Duration(120)):
            rospy.logerr("Unable to connect to move_base action server.")
            rospy.signal_shutdown("Failed to connect to move_base action server.")
            return
        
        rospy.loginfo("Connected to move base server")
        
        rospy.loginfo("Ready to go")
        rospy.sleep(1)
    
        # Define the target location (point A)
        A_x = 2.56
        A_y = -0.0512
        A_theta = 0
        
        quaternion = quaternion_from_euler(0.0, 0.0, A_theta)
        self.target_location = Pose(Point(A_x, A_y, 0.0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        
        self.goal = MoveBaseGoal()
        rospy.loginfo("Starting navigation test")

        while not rospy.is_shutdown():
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            if self.start == 1:
                rospy.loginfo("Going to point A")
                rospy.sleep(2)
                self.goal.target_pose.pose = self.target_location
                self.move_base.send_goal(self.goal)

                while not self.move_base.wait_for_result(rospy.Duration(0.1)):
                    if self.detect_obstacle():
                        rospy.loginfo("Obstacle detected! Executing avoidance behavior...")
                        self.move_base.cancel_goal()
                        self.avoid_obstacle()
                        rospy.loginfo("Resuming navigation...")
                        self.move_base.send_goal(self.goal)
                
                if self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached point A")
                    rospy.sleep(2)
                    self.start = 2  # Stop after reaching point A

            rospy.Rate(5).sleep()

    def detect_obstacle(self):
        """
        Function to detect obstacles using the RealSense depth data (pyrealsense2).
        """
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            return False
        
        # Convert depth frame to numpy array (depth values are in millimeters)
        depth_image = np.asanyarray(depth_frame.get_data())

        # Image width and height
        image_height, image_width = depth_image.shape

        # Segment the depth image
        left_section = depth_image[:, :image_width // 3]
        center_section = depth_image[:, image_width // 3:(2 * image_width) // 3]
        right_section = depth_image[:, (2 * image_width) // 3:]

        # Filter out invalid depth values (zeros)
        left_section = left_section[left_section > 0]
        center_section = center_section[center_section > 0]
        right_section = right_section[right_section > 0]

        # Check if any of the sections are empty after filtering
        if left_section.size == 0:
            left_distance = float('inf')  # No valid data, treat as no obstacle
        else:
            left_distance = np.mean(left_section) / 1000.0  # Convert from mm to meters

        if center_section.size == 0:
            center_distance = float('inf')
        else:
            center_distance = np.mean(center_section) / 1000.0

        if right_section.size == 0:
            right_distance = float('inf')
        else:
            right_distance = np.mean(right_section) / 1000.0

        # Detect obstacle based on threshold distances (in meters)
        if center_distance < 0.5 or left_distance < 0.5 or right_distance < 0.5:
            return True  # Obstacle detected
        else:
            return False

    def avoid_obstacle(self):
        rospy.loginfo("Avoiding obstacle...")

        # Define twist message for movement
        twist = Twist()

        # Set rotation and movement speeds
        rotation_speed = 0.5  # radians/sec
        move_speed = 0.2      # meters/sec

        # Determine where the obstacle is (left, right, or center)
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            return
        
        depth_image = np.asanyarray(depth_frame.get_data())
        image_height, image_width = depth_image.shape
        
        left_section = depth_image[:, :image_width // 3]
        center_section = depth_image[:, image_width // 3:(2 * image_width) // 3]
        right_section = depth_image[:, (2 * image_width) // 3:]

        # Filter out invalid depth values (zeros)
        left_section = left_section[left_section > 0]
        center_section = center_section[center_section > 0]
        right_section = right_section[right_section > 0]

        # Check if any of the sections are empty after filtering
        if left_section.size == 0:
            left_distance = float('inf')  # No valid data, treat as no obstacle
        else:
            left_distance = np.mean(left_section) / 1000.0  # Convert from mm to meters

        if center_section.size == 0:
            center_distance = float('inf')
        else:
            center_distance = np.mean(center_section) / 1000.0

        if right_section.size == 0:
            right_distance = float('inf')
        else:
            right_distance = np.mean(right_section) / 1000.0

        obstacle_detected = False

        if center_distance < 0.5:  # Obstacle in front
            rospy.loginfo("Obstacle detected in front, turning left")
            twist.angular.z = rotation_speed  # Turn left
            obstacle_detected = True
        elif left_distance < 0.5:  # Obstacle on the left
            rospy.loginfo("Obstacle detected on the left, turning right")
            twist.angular.z = -rotation_speed  # Turn right
            obstacle_detected = True
        elif right_distance < 0.5:  # Obstacle on the right
            rospy.loginfo("Obstacle detected on the right, turning left")
            twist.angular.z = rotation_speed  # Turn left
            obstacle_detected = True

        if obstacle_detected:
            # Turn for 3 seconds
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(3)

            # Move straight for 3 seconds
            rospy.loginfo("Moving straight for 3 seconds")
            twist.angular.z = 0  # Stop rotating
            twist.linear.x = move_speed  # Move forward
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(3)

            # Turn in the opposite direction for 3 seconds
            rospy.loginfo("Turning in the opposite direction for 3 seconds")
            twist.angular.z = -twist.angular.z  # Reverse the previous turn direction
            twist.linear.x = 0  # Stop moving forward
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(3)

            # Stop the robot after avoidance
            rospy.loginfo("Obstacle avoidance complete, stopping the robot")
            twist.angular.z = 0
            twist.linear.x = 0
            self.cmd_vel_pub.publish(twist)

    def cleanup(self):
        rospy.loginfo("Shutting down navigation....")
        self.move_base.cancel_goal()
        self.pipeline.stop()

if __name__ == "__main__":
    rospy.init_node('navi_point', anonymous=True)
    try:
        NavToPoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
