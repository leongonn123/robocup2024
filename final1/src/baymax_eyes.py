#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

def gif_publisher():
    # Initialize ROS node
    rospy.init_node('baymax_gif', anonymous=True)

    # Create a publisher for the image topic
    image_pub = rospy.Publisher('/gif_frames', Image, queue_size=10)

    # Create a CvBridge object to convert OpenCV images to ROS images
    bridge = CvBridge()

    # Load the GIF using OpenCV
    gif_path = '/home/charmander/catkin_ws/src/final_task/src/baymax.gif'
    gif = cv2.VideoCapture(gif_path)

    # Check if the GIF was successfully loaded
    if not gif.isOpened():
        rospy.logerr("Failed to load GIF")
        sys.exit(1)

    # Get screen dimensions or specify target size
    screen_width = 1920  # Example: full HD screen width
    screen_height = 1080  # Example: full HD screen height

    # Set the loop rate (in Hz) according to the GIF frame rate
    rate = rospy.Rate(10)  # Example: 10 Hz

    # Loop to read and publish each frame
    while not rospy.is_shutdown():
        ret, frame = gif.read()
        if not ret:
            rospy.loginfo("End of GIF, restarting...")
            gif.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart the GIF
            continue

        if frame is None:
            rospy.logerr("Empty frame received!")
            continue

        # Resize (maximize) the frame to the target size
        frame = cv2.resize(frame, (screen_width, screen_height), interpolation=cv2.INTER_LINEAR)

        # Display frame for debugging
        cv2.imshow("GIF Frame", frame)
        cv2.waitKey(1)

        # Convert the frame to a ROS Image message
        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the image message
        image_pub.publish(image_msg)
        #rospy.loginfo("Published a frame")

        # Sleep to maintain the loop rate
        rate.sleep()

    # Release the GIF capture object
    gif.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        gif_publisher()
    except rospy.ROSInterruptException:
        pass
