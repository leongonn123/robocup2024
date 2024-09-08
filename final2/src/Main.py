#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import subprocess

# Global variable to store the state of /fall
fall_detected = False

# Callback for /fall topic
def fall_callback(msg):
    global fall_detected
    fall_detected = msg.data
    rospy.loginfo(f"Fall state: {fall_detected}")
    check_fall()

# Function to check if fall state is True
def check_fall():
    global fall_detected
    
    if fall_detected:
        rospy.loginfo("Fall detected! Initiating shutdown and launching new processes.")
        
        # Launch kobuki.launch and navigation.py
        rospy.loginfo("Launching kobuki.launch.")
        #subprocess.Popen(["roslaunch", "kobuki_node", "kobuki.launch"])  # Replace with your package and launch file
        #subprocess.Popen(["roslaunch", "main", "navi.launch"])
        
        rospy.loginfo("Launching navigation.py.")
        navigation_process = subprocess.Popen(["rosrun", "navigation", "navigation.py"])  # Replace with your package and script name
        navigation_process.wait()

        rospy.loginfo("Launching kobuki base.")
        navigation_process = subprocess.Popen(["roslaunch", "kobuki_node", "minimal.launch"])  # Replace with your package and script name
        navigation_process.wait()
       

        # Proceed with other processes after navigation.py completes
        fire_spin_process = subprocess.Popen(["rosrun", "final_task_2", "fire_spin.py"])  # Replace with your package and script name
        fire_spin_process.wait()

        arm_process = subprocess.Popen(["rosrun", "take_bag", "arm.py"])  # Replace with your package and script name
        arm_process.wait()

        spin_process = subprocess.Popen(["rosrun", "final_task_2", "Spinning.py"])
        spin_process.wait()

        TTS_process = subprocess.Popen(["rosrun", "final_task_2", "TTS.py"])
        TTS_process.wait()

def listener():
    # Initialize the node
    rospy.init_node('monitor_fall_node', anonymous=True)

    # Run the fall detection node as the first subprocess
    rospy.loginfo("Starting fall detection node.")
    fall_detection_process = subprocess.Popen(["rosrun", "final_task_2", "Fall_detection_ROS.py"])  # Replace with your package and node
    rospy.loginfo("Fall detection node started.")

    # Subscribe to the /fall topic to detect fall events
    rospy.Subscriber('/fall', Bool, fall_callback)

    # Keep the node alive and processing callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
