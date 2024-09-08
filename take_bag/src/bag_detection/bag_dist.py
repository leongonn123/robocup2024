#!/usr/bin/env python3

import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import threading
import queue

# Function to rotate the image by the given angle (degrees)
def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    return cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)

# Thread function to capture frames from the camera
def capture_frames(pipeline, frame_queue):
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            rospy.logwarn("Failed to retrieve frames from the camera")
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Rotate images to correct for upside-down camera installation
        depth_image = rotate_image(depth_image, 180)
        color_image = rotate_image(color_image, 180)

        if not frame_queue.full():
            frame_queue.put((depth_image, color_image))

# Callback function for reaching target distance
def reach_distance_callback(msg):
    if msg.data:
        rospy.loginfo("Target distance reached. Shutting down node.")
        rospy.signal_shutdown("Shutting down due to reaching the target distance.")

# Thread function to process frames
def process_frames(frame_queue, distance_pub, angle_pub, model, HORIZONTAL_FOV, IMAGE_WIDTH, velocity_pub):
    while not rospy.is_shutdown():
        if not frame_queue.empty():
            depth_image, color_image = frame_queue.get()

            results = model(color_image, stream=False)

            highest_confidence = 0.0
            best_box = None
            for det in results[0].boxes:
                if det.conf > 0.81:
                    confidence = det.conf[0]
                    if confidence > highest_confidence:
                        highest_confidence = confidence
                        best_box = det.xyxy[0].tolist()

            if best_box:
                x1, y1, x2, y2 = map(int, best_box)
                best_center_x = (x1 + x2) // 2
                best_center_y = (y1 + y2) // 2
                depth_value = depth_image[best_center_y, best_center_x]
                pixel_offset = best_center_x - (IMAGE_WIDTH // 2)
                angle = (pixel_offset / IMAGE_WIDTH) * HORIZONTAL_FOV

                # Publish the distance and angle
                distance_pub.publish(depth_value)
                angle_pub.publish(angle)

                # Annotate the frame
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, 'Bag detected', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                # No bag detected, move forward
                move_cmd = Twist()
                move_cmd.linear.x = 0.1  # Move forward at 0.2 m/s
                move_cmd.angular.z = 0.0  # No angular rotation
                velocity_pub.publish(move_cmd)

            cv2.imshow('Bag Detection', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def main():
    rospy.init_node('bag_detection_node')

    distance_pub = rospy.Publisher('/bag_distance', Float32, queue_size=10)
    angle_pub = rospy.Publisher('/bag_angle', Float32, queue_size=10)
    velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)  # Update this topic name
    rospy.Subscriber('/reach_distance', Bool, reach_distance_callback)

    frame_queue = queue.Queue(maxsize=10)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    model = YOLO("/home/charmander/catkin_ws/src/take_bag/src/bag_detection/best.pt")
    model.cuda()
    HORIZONTAL_FOV = 65.0
    IMAGE_WIDTH = 640

    capture_thread = threading.Thread(target=capture_frames, args=(pipeline, frame_queue))
    process_thread = threading.Thread(target=process_frames, args=(frame_queue, distance_pub, angle_pub, model, HORIZONTAL_FOV, IMAGE_WIDTH, velocity_pub))

    capture_thread.start()
    process_thread.start()

    rospy.spin()

    capture_thread.join()
    process_thread.join()

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass