#!/usr/bin/env python3

import rospy
import subprocess
import time
import signal
import os
import wave
import pyaudio
from std_msgs.msg import Bool

class RobotManager:
    def __init__(self):
        self.node_complete = False
        rospy.init_node('robot_manager', anonymous=True)
        self.sub = rospy.Subscriber('/node_complete', Bool, self.callback)

    def callback(self, msg):
        rospy.loginfo("Received completion message: {}".format(msg.data))
        self.node_complete = msg.data

    def wait_for_barrier_detection(self):
        rospy.loginfo("Waiting for barrier detection...")
        while not rospy.is_shutdown() and self.detector.barrier_status is None:
            rospy.sleep(0.1)
        return self.detector.barrier_status
        
    def wait_for_completion(self):
        rospy.loginfo("Waiting for task completion...")
        while not self.node_complete and not rospy.is_shutdown():
            rospy.sleep(0.1)  # Sleep briefly to yield time to ROS processing
        self.node_complete = False  # Reset flag after task is complete

    def start_node(self, node_name, script_path):
        rospy.loginfo(f"Starting {node_name}...")
        return subprocess.Popen(['rosrun', node_name, script_path], preexec_fn=os.setsid)

    def stop_node(self, process, node_name, timeout=10):
        if process.poll() is None:  # Check if the process is still running
            rospy.loginfo(f"Stopping {node_name}...")
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # Terminate the process group
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                rospy.logwarn(f"{node_name} did not terminate in time; force killing.")
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)  # Force kill if not terminated after timeout
        else:
            rospy.logwarn(f"{node_name} process already exited or was never started.")


    def start_launch(self, package, launch_file):
        rospy.loginfo(f"Starting launch file {launch_file} from package {package}")
        command = ['roslaunch', package, launch_file]
        process = subprocess.Popen(command, preexec_fn=os.setsid)
        return process

    def stop_launch(self, process, launch_name):
        if process.poll() is None:  # Check if the process is still running
            rospy.loginfo(f"Stopping launch file {launch_name}...")
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # Send the signal to terminate the process group
            process.wait()
        else:
            rospy.logwarn(f"{launch_name} launch file already exited or was never started.")
        rospy.loginfo(f"{launch_name} launch file terminated.")

# Function to play audio using PyAudio and Wave
def play_audio(file_name):
    # Open the .wav file
    wave_file = wave.open(file_name, 'rb')

    # Initialize PyAudio
    p = pyaudio.PyAudio()

    # Open a stream with the correct settings
    stream = p.open(format=p.get_format_from_width(wave_file.getsampwidth()),
                    channels=wave_file.getnchannels(),
                    rate=wave_file.getframerate(),
                    output=True)

    # Read data in chunks
    chunk = 4096
    data = wave_file.readframes(chunk)

    # Play the audio
    rospy.loginfo(f"Playing audio from: {file_name}")
    while data:
        stream.write(data)
        data = wave_file.readframes(chunk)

    # Stop and close the stream
    stream.stop_stream()
    stream.close()

    # Close PyAudio
    p.terminate()

    # Close the .wav file
    wave_file.close()

if __name__ == "__main__":
    manager = RobotManager()
    
    try:
        #launch lidar files
        #start_process = manager.start_launch('main', 'navi.launch')

        play_audio('/home/charmander/catkin_ws/src/main/src/main/audio/audio2.wav')

        first_process = manager.start_launch('main', 'point_bag.launch')
        manager.wait_for_completion()
        manager.stop_launch(first_process, 'point_bag.launch')
        
        play_audio('/home/charmander/catkin_ws/src/main/src/main/audio/takebag.wav')
        
        second_process = manager.start_launch('main', 'approach_bag.launch')
        manager.wait_for_completion()
        manager.stop_launch(second_process, 'approach_bag.launch')
        
        third_process = manager.start_launch('main', 'arm_rotate.launch')
        manager.wait_for_completion()
        manager.stop_launch(third_process, 'arm_rotate.launch')
        
        rospy.sleep(1)
        
        rotate_process = manager.start_node('take_bag', 'rotation.py')
        manager.wait_for_completion()
        manager.stop_node(rotate_process, 'rotate_node')
        
        rotate_process = manager.start_node('follow_me', 'backward.py')
        manager.wait_for_completion()
        manager.stop_node(rotate_process, 'backward_node')
        
        play_audio('/home/charmander/catkin_ws/src/main/src/main/audio/audio3.wav')

        fourth_process = manager.start_launch('follow_me', 'follow_me.launch')
        rospy.sleep(1)
        barrier_detect = manager.start_launch('main', 'barrier.launch')
        
        rospy.sleep(20)

        fifth_process = manager.start_node('follow_me', 'follow_stop.py')

        manager.wait_for_completion()
        manager.stop_launch(fourth_process, 'follow_me.launch')
        manager.stop_launch(barrier_detect, 'barrier.launch')
        manager.stop_node(fifth_process, 'stop_node')
        
        rotate_process = manager.start_node('take_bag', 'rotation.py')
        manager.wait_for_completion()
        manager.stop_node(rotate_process, 'rotate_node')
        
        drop_process = manager.start_launch('main', 'drop_rotate.launch')
        manager.wait_for_completion()
        manager.stop_launch(drop_process, 'drop_node')
        
        # run navigation scripts
        sixth_process = manager.start_node('navigation', 'navigation.py')
        manager.wait_for_completion()
        manager.stop_node(sixth_process, 'reach_node')

        rospy.sleep

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Shutting down nodes.")
    
    rospy.loginfo("Main node operation completed.")
