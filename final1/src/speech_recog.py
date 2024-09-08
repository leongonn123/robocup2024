#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import speech_recognition as sr

class SpeechRecognitionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('speech_recognition_node', anonymous=True)

        # Initialize the speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Subscribe to the /fall_recovery_flag topic
        rospy.Subscriber('/fall_recovery_flag', Bool, self.callback)

    def callback(self, msg):
        # Check if the fall recovery flag is True
        if msg.data:
            rospy.loginfo("Fall recovery flag is True. Starting speech recognition...")
            self.recognize_speech()
        else:
            rospy.loginfo("Fall recovery flag is False. Not listening.")

    def recognize_speech(self):
        # Use the microphone as a source for input
        with self.microphone as source:
            rospy.loginfo("Adjusting for ambient noise... Please wait.")
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Listening for speech...")
            audio = self.recognizer.listen(source)

        # Recognize speech using Google's speech recognition
        try:
            recognized_text = self.recognizer.recognize_google(audio)
            rospy.loginfo(f"Recognized speech: {recognized_text}")
        except sr.UnknownValueError:
            rospy.logwarn("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            rospy.logerr(f"Could not request results from Google Speech Recognition service; {e}")

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        sr_node = SpeechRecognitionNode()
        sr_node.run()
    except rospy.ROSInterruptException:
        pas
