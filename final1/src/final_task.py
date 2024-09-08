#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np
import rospy
import threading
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Twist

# Initialize Mediapipe Pose
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Global variables for image data and speeds
global_image_bgr = None
global_image_depth = None
linear_speed = 0.0
angular_speed = 0.0
image_lock = threading.Lock()

# Fall detection variables
previous_avg_shoulder_height = 0
time1 = time.time()

# Event for emergency stop
emergency_stop = threading.Event()

def rgb_image_callback(msg):
    global global_image_bgr
    try:
        if msg.encoding == 'mono8':
            rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2BGR)  # Convert grayscale to BGR
        elif msg.encoding == 'rgb8':
            rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            global_image_bgr = rgb_image  # Now in RGB format
        elif msg.encoding == 'bayer_grbg8':
            bayer_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            rgb_image = cv2.cvtColor(bayer_image, cv2.COLOR_BAYER_GR2BGR)  # Convert Bayer to BGR
            global_image_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)  # Convert to RGB format
        else:
            rospy.logerr(f"Unsupported image encoding: {msg.encoding}")
            return

        with image_lock:
            global_image_bgr = cv2.cvtColor(global_image_bgr, cv2.COLOR_RGB2BGR)  # Ensure it's in BGR for OpenCV
    except Exception as e:
        rospy.logerr(f"Error processing RGB image: {e}")

def depth_image_callback(msg):
    global global_image_depth
    try:
        depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)  # Normalize to 8-bit
        depth_image = np.uint8(depth_image)
        depth_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)  # Convert grayscale to BGR for display

        with image_lock:
            global_image_depth = depth_image
    except Exception as e:
        rospy.logerr(f"Error processing depth image: {e}")

def velocity_callback(msg):
    global linear_speed, angular_speed
    linear_speed = msg.linear.x
    angular_speed = msg.angular.z

def detectFall(landmarks, previous_avg_shoulder_height):
    left_shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y
    right_shoulder_y = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
    
    # Calculate the average y-coordinate of the shoulder
    avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2

    if previous_avg_shoulder_height == 0:
        previous_avg_shoulder_height = avg_shoulder_y
        return False, previous_avg_shoulder_height

    fall_threshold = previous_avg_shoulder_height * 1.5
    
    # Check if the average shoulder y-coordinate falls more than the threshold
    if avg_shoulder_y > fall_threshold:
        previous_avg_shoulder_height = avg_shoulder_y
        return True, previous_avg_shoulder_height
    else:
        previous_avg_shoulder_height = avg_shoulder_y
        return False, previous_avg_shoulder_height

def detect_pose_and_fall():
    global global_image_bgr, global_image_depth, linear_speed, angular_speed, previous_avg_shoulder_height, time1

    # Initialize ROS node
    rospy.init_node('pose_and_fall_detection_node', anonymous=True)

    # Publishers
    pose_pub = rospy.Publisher('/pose_landmarks', Float32MultiArray, queue_size=10)
    fall_pub = rospy.Publisher('/fall', String, queue_size=10)
    stop_fllw_pub = rospy.Publisher('/follow_me_stop', Bool, queue_size=10)

    # Subscribers
    rospy.Subscriber('/mobile_base/commands/velocity', Twist, velocity_callback)
    rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_image_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_image_callback)
    rospy.Subscriber('/follow_me_stop', Bool, follow_me_callback)

    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while not rospy.is_shutdown():
            with image_lock:
                if global_image_bgr is not None and global_image_depth is not None:
                    image_rgb = cv2.cvtColor(global_image_bgr, cv2.COLOR_BGR2RGB)
                    image_rgb.flags.writeable = False
                    results = pose.process(image_rgb)
                    image_rgb.flags.writeable = True

                    if results.pose_landmarks:
                        mp_drawing.draw_landmarks(
                            global_image_bgr, 
                            results.pose_landmarks, 
                            mp_pose.POSE_CONNECTIONS,
                            mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                            mp_drawing.DrawingSpec(color=(144, 238, 144), thickness=2, circle_radius=2)
                        )
                        
                        landmarks = results.pose_landmarks.landmark

                        # Publish hip landmarks
                        left_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP]
                        right_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP]
                        pose_msg = Float32MultiArray()
                        pose_msg.data = [left_hip.x, left_hip.y, right_hip.x, right_hip.y]
                        pose_pub.publish(pose_msg)

                        # Fall detection logic
                        time2 = time.time()
                        if (time2 - time1) > 2:  # Check fall detection every 2 seconds
                            fall_detected, previous_avg_shoulder_height = detectFall(landmarks, previous_avg_shoulder_height)
                            if fall_detected:
                                rospy.loginfo("Fall detected!")
                                fall_pub.publish("Fall detected!")
                                stop_fllw_pub.publish(Bool(data=True))  # Properly publishing a Bool message
                            time1 = time2

                    # Overlay linear and angular speed on the RGB image
                    cv2.putText(global_image_bgr, f'Linear Speed: {linear_speed:.2f} m/s', 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(global_image_bgr, f'Angular Speed: {angular_speed:.2f} rad/s', 
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    # Combine RGB and Depth images side by side
                    combined_image = np.hstack((global_image_bgr, global_image_depth))
                    cv2.imshow("RGB and Depth Image with Pose, Speed, and Fall Detection", combined_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        rospy.signal_shutdown("Shutdown initiated by user.")
                        break

            rospy.sleep(0.05)

def follow_me_callback(msg):
    if msg.data:
        rospy.loginfo("Stopping follow me process.")
        emergency_stop.set()

if __name__ == '__main__':
    try:
        detect_pose_and_fall()
    except rospy.ROSInterruptException:
        pass
