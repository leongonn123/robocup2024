import cv2
import mediapipe as mp
import numpy as np
import time

# Initialize Mediapipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Initialize drawing utils for visualization
mp_drawing = mp.solutions.drawing_utils

# Specify the drawing style for landmarks and connections
landmark_drawing_spec = mp_drawing.DrawingSpec(thickness=2, circle_radius=2)
connection_drawing_spec = mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)  # Red color line

# Capture video from webcam
cap = cv2.VideoCapture(0)

# Initialize variables
previous_avg_shoulder_height = 0
time1 = time.time()

def detectFall(landmarks, previous_avg_shoulder_height):
    left_shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y
    right_shoulder_y = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
    
    # Calculate the average y-coordinate of the shoulder
    avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2

    if previous_avg_shoulder_height == 0:
        previous_avg_shoulder_height = avg_shoulder_y
        return False, previous_avg_shoulder_height

    fall_threshold = previous_avg_shoulder_height * 1.5
    print(previous_avg_shoulder_height, avg_shoulder_y,end="\n")
    
    # Check if the average shoulder y-coordinate falls more than the threshold
    if avg_shoulder_y > fall_threshold:
        previous_avg_shoulder_height = avg_shoulder_y
        return True, previous_avg_shoulder_height
    else:
        previous_avg_shoulder_height = avg_shoulder_y
        return False, previous_avg_shoulder_height

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the BGR image to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame and detect body landmarks
    results = pose.process(frame_rgb)

    if results.pose_landmarks:
        mp_drawing.draw_landmarks(
            image=frame,
            landmark_list=results.pose_landmarks,
            connections=mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=landmark_drawing_spec,
            connection_drawing_spec=connection_drawing_spec
        )

        landmarks = results.pose_landmarks.landmark

        time2 = time.time()
    
        if (time2 - time1) > 2:  # Check fall detection every 2 seconds
            if landmarks is not None:
                fall_detected, previous_avg_shoulder_height = detectFall(landmarks, previous_avg_shoulder_height)
                if fall_detected:
                    print("Fall detected!")
            time1 = time2

    # Display the resulting frame
    cv2.imshow('Fall Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# Release the video capture object
cap.release()
cv2.destroyAllWindows(