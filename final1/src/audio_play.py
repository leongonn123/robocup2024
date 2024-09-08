#!/usr/bin/env python3

import rospy
import pygame
import time
import sys

def play_audio():
    # Initialize the ROS node
    rospy.init_node('baymax_audio', anonymous=True)

    # Initialize pygame mixer
    pygame.mixer.init()

    # Load the audio file
    audio_file = "/home/charmander/catkin_ws/src/final_task/src/baymax_2.wav"

    try:
        pygame.mixer.music.load(audio_file)
    except pygame.error as e:
        rospy.logerr(f"Error loading audio file: {e}")
        sys.exit(1)

    # Play the audio file
    pygame.mixer.music.play()

    # Wait for the audio to finish playing
    while pygame.mixer.music.get_busy() and not rospy.is_shutdown():
        time.sleep(0.1)  # Sleep briefly to prevent high CPU usage

    # Optional: Clean up
    pygame.mixer.music.stop()
    pygame.mixer.quit()

if __name__ == '__main__':
    try:
        play_audio()
    except rospy.ROSInterruptException:
        pass
