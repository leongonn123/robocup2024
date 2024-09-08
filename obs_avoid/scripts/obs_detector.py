#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

class ObstacleDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_detector', anonymous=True)
        
        # Create a CvBridge to convert ROS image messages to OpenCV format
        self.bridge = CvBridge()
        
        # Subscribe to the depth image topic
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        
        # Publisher for barrier detection
        self.barrier_pub = rospy.Publisher('/barrier_detected', Bool, queue_size=10)
        
        # Set the distance threshold for obstacle detection (0.6 meters)
        self.threshold_distance = 0.4

    def depth_callback(self, data):
        try:
            # Convert the depth image from ROS Image message to an OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="16UC1")  # Assuming 16-bit unsigned depth
            
            # Convert the depth image from millimeters to meters (if needed)
            depth_image = np.array(depth_image, dtype=np.float32) / 1000.0  # Convert millimeters to meters
            
            # Replace invalid values (0.0 or NaN) with a large number (e.g., 10 meters)
            depth_image = np.where((depth_image == 0) | np.isnan(depth_image), 10.0, depth_image)
            
            # Get the minimum valid distance in the depth image
            min_distance = np.nanmin(depth_image)
            
            # Check if there's an obstacle within the threshold distance
            if min_distance < self.threshold_distance:
                rospy.loginfo("Obstacle detected at %.2f meters", min_distance)
                self.barrier_pub.publish(True)
            else:
                self.barrier_pub.publish(False)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    try:
        detector = ObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
