#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo

def camera_info_publisher():
    # Initialize the ROS node
    rospy.init_node('camera_info_publisher', anonymous=True)
    
    # Create a publisher for the CameraInfo message
    camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)

    # Create a CameraInfo message and fill in the parameters
    camera_info_msg = CameraInfo()

    # Example camera parameters (replace these with actual values)
    camera_info_msg.header.frame_id = "my_camera_frame"
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = [0, 0, 0, 0, 0]  # No distortion
    camera_info_msg.K = [525, 0, 319.5, 0, 525, 239.5, 0, 0, 1]  # Intrinsic camera matrix
    camera_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Rectification matrix
    camera_info_msg.P = [525, 0, 319.5, 0, 0, 525, 239.5, 0, 0, 0, 1, 0]  # Projection matrix

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        camera_info_msg.header.stamp = rospy.Time.now()  # Update the timestamp
        camera_info_pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_info_publisher()
    except rospy.ROSInterruptException:
        pass
