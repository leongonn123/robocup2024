#!/usr/bin/env python3

import rospy
import cv2
import numpy as np  # For color filtering
from std_msgs.msg import Bool  # Boolean message type

# Function to check if the detected region has fire-like colors (red/orange)
def is_fire_color(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Define the lower and upper range for fire-like colors (adjust these values as needed)
    lower_fire = np.array([0, 50, 50])  # Red/orange hue lower bound
    upper_fire = np.array([35, 255, 255])  # Red/orange hue upper bound
    
    # Create a mask that identifies fire-like colors
    mask = cv2.inRange(hsv, lower_fire, upper_fire)
    
    # Calculate the percentage of the region that matches fire-like colors
    fire_pixels = cv2.countNonZero(mask)
    total_pixels = roi.shape[0] * roi.shape[1]
    fire_ratio = fire_pixels / total_pixels
    
    # Consider it fire if a significant portion of the region is fire-colored
    return fire_ratio > 0.4  # Adjust threshold as needed

def fire_detection_node():
    rospy.init_node('fire', anonymous=True)  # Initialize the ROS node
    pub = rospy.Publisher('/fire_state', Bool, queue_size=10)  # Publisher to the /fire_state topic
    rate = rospy.Rate(10)  # Set the loop rate to 10 Hz

    # Load the fire cascade classifier
    fire_cascade = cv2.CascadeClassifier('/home/charmander/catkin_ws/src/final_task_2/src/fire_detection_cascade_model.xml')

    # Start webcam capture
    vid = cv2.VideoCapture(2)  # Adjust the index if you use a different camera

    fire_detected = False  # Initialize fsire detection state

    while not rospy.is_shutdown():
        ret, frame = vid.read()
        
        if not ret:
            rospy.logwarn("Failed to grab frame from the camera.")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale
        fire = fire_cascade.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), maxSize=(500, 500))

        fire_detected_in_frame = False  # Reset fire detection for this frame

        if len(fire) > 0:
            for (x, y, w, h) in fire:
                roi_color = frame[y:y+h, x:x+w]
                
                # Apply color filtering to detect fire-like colors
                if is_fire_color(roi_color):
                    fire_detected_in_frame = True  # Set flag if fire is detected
                    cv2.rectangle(frame, (x-20, y-20), (x+w+20, y+h+20), (255, 0, 0), 2)

        # Publish fire detection state
        if fire_detected_in_frame and not fire_detected:
            rospy.loginfo("Fire detected.")
            pub.publish(True)  # Publish True when fire is detected
            fire_detected = True
        elif not fire_detected_in_frame and fire_detected:
            rospy.loginfo("No fire detected.")
            pub.publish(False)  # Publish False when no fire is detected
            fire_detected = False

        # Show the video feed with detected fire
        cv2.imshow('Webcam Frame', frame)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()  # Sleep to maintain the loop rate

    # Release the video capture and close all windows
    vid.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        fire_detection_node()
    except rospy.ROSInterruptException:
        pass
