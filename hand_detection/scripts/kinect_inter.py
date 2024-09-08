#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

class ImageThrottle:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_throttle')

        # Publisher for throttled image topic
        self.pub = rospy.Publisher('/camera/rgb/image_reduced', Image, queue_size=10)
        rospy.loginfo("image published")
        # Subscribe to the Kinect's RGB image topic
        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)

        # Frame counter
        self.frame_count = 0

        # Set the frame reduction factor
        self.reduction_factor = 6  # Adjust this to control the output rate

    def callback(self, msg):
        # Publish every nth frame
        if self.frame_count % self.reduction_factor == 0:
            self.pub.publish(msg)
            rospy.loginfo("Published reduced frame %d", self.frame_count)

        # Increment frame count
        self.frame_count += 1

if __name__ == '__main__':
    img_throttle = ImageThrottle()
    rospy.spin()