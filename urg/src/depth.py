#!/usr/bin/env python3

import pyrealsense2 as rs
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

def realsense_depth_publisher():
    # Initialize the ROS node
    rospy.init_node('realsense_depth_publisher', anonymous=True)
    
    # Create a publisher for the depth image
    depth_pub = rospy.Publisher('/camera/depth_realsense', Image, queue_size=10)
    
    # Create a CvBridge object to convert between ROS Image messages and OpenCV images
    bridge = CvBridge()
    
    # Configure depth stream
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream config
    
    # Start streaming
    pipeline.start(config)
    
    rospy.loginfo("Started RealSense Depth Stream")

    try:
        while not rospy.is_shutdown():
            # Wait for a new frame from the camera
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            if not depth_frame:
                continue

            # Convert depth frame to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())

            # Convert the depth image to a ROS Image message
            depth_ros_image = bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")

            # Publish the depth image
            depth_pub.publish(depth_ros_image)

            rospy.loginfo("Published depth frame to /camera/depth_realsense")

    except rospy.ROSInterruptException:
        pass
    finally:
        # Stop streaming when done
        pipeline.stop()

if __name__ == '__main__':
    try:
        realsense_depth_publisher()
    except rospy.ROSInterruptException:
        pass
