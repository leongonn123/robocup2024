#!/usr/bin/env python3

import rospy
import pyrealsense2 as rs
import numpy as np
from std_msgs.msg import Bool

def obstacle_detection_node():
    # Initialize the ROS node
    rospy.init_node('obstacle_detection', anonymous=True)
    
    # Create a publisher to the topic /obstacle_detected
    obstacle_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=10)
    
    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    
    rospy.loginfo("Obstacle detection node started.")
    
    try:
        while not rospy.is_shutdown():
            # Wait for a new frame
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            # Convert depth frame to a numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Check if any value is within the 0.5m range
            obstacle_detected = np.any((depth_image > 0) & (depth_image < 1800)) # depth_image is in millimeters
            
            # Publish the result
            obstacle_pub.publish(Bool(obstacle_detected))
            
            rospy.loginfo(f"Obstacle detected: {obstacle_detected}")
            
            rospy.sleep(0.1)
    
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    
    finally:
        # Stop the pipeline on shutdown
        pipeline.stop()

if __name__ == '__main__':
    try:
        obstacle_detection_node()
    except rospy.ROSInterruptException:
        pass
