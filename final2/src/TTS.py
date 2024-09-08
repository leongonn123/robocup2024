#!/usr/bin/env python3

import rospy
from gtts import gTTS
from playsound import playsound
import os
from std_msgs.msg import Bool  # Changed to Bool since you're triggering a fixed message

# Function to generate speech using gTTS and play it
def say_text():
    try:
        fixed_message = "I saw a victim falling on the ground, emergency rescue is on the way"
        rospy.loginfo(f"Generating speech for: {fixed_message}")
        tts = gTTS(text=fixed_message, lang='en')
        tts.save("/tmp/speech.mp3")  # Save the generated speech to a file
        
        rospy.loginfo("Playing speech...")
        playsound("/tmp/speech.mp3")  # Play the audio file
        
        # Optionally, remove the file after playing
        os.remove("/tmp/speech.mp3")
    except Exception as e:
        rospy.logerr(f"Failed to generate or play speech: {e}")

# Callback function when a trigger is received
def speech_callback(msg):
    if msg.data:  # If the received boolean is True, trigger the speech
        say_text()

# Main function
def gtts_node():
    rospy.init_node('gtts_node', anonymous=True)

    # Subscribe to a /trigger_speech topic to trigger the fixed speech
    rospy.Subscriber('/trigger_speech', Bool, speech_callback)

    rospy.loginfo("gTTS Node is ready. Waiting for trigger to speak...")

    # Keep the node alive and processing callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        gtts_node()
    except rospy.ROSInterruptException:
        pass
