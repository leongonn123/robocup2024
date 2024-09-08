#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sounddevice as sd
import vosk
import queue
import pyttsx3
import os
import sys

# Initialize TTS engine
tts_engine = pyttsx3.init()

# Reduce the TTS voice speed
tts_engine.setProperty('rate', 150)  # Decrease the rate (default is around 200)

# Queue to handle audio data
q = queue.Queue()

# Vosk Model (make sure to download a Vosk model and set the path correctly)
vosk_model_path = "/home/charmander/catkin_ws/src/follow_me/scripts/vosk-model-small-en-us-0.15"
if not os.path.exists(vosk_model_path):
    rospy.logerr(f"Vosk model not found at {vosk_model_path}")
    sys.exit(1)

model = vosk.Model(vosk_model_path)

# Global variable for the publisher
acknowledge_pub = None

# Function to handle audio input
def callback(indata, frames, time, status):
    if status:
        rospy.logwarn(f"Sounddevice status: {status}")
    q.put(bytes(indata))

# Function to speak a text using TTS
def speak(text):
    tts_engine.say(text)
    tts_engine.runAndWait()

# Function to listen to the victim
def listen_to_victim():
    global acknowledge_pub

    # Initialize ROS node
    rospy.init_node('listen_after_fall_node', anonymous=True)

    # Publisher to acknowledge victim's response
    acknowledge_pub = rospy.Publisher('/victim_response', String, queue_size=10)

    # Subscriber to listen to fall detection
    rospy.Subscriber('/fall', String, fall_detected_callback)

    rospy.loginfo("Waiting for fall detection...")

    rospy.spin()

def fall_detected_callback(msg):
    rospy.loginfo("Fall detected! Starting to listen to the victim...")
    
    # Notify the victim that the system is listening
    speak("I detected a fall. I'm here to help. Please speak to me.")

    # Start listening to the victim
    with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                           channels=1, callback=callback):
        rec = vosk.KaldiRecognizer(model, 16000)
        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = rec.Result()
                text = eval(result)['text']
                rospy.loginfo(f"Recognized speech: {text}")
                if text:
                    acknowledge_pub.publish(text)
                    rospy.loginfo("Victim's response acknowledged.")
                    speak(f"I heard you say: {text}. Help is on the way.")
                    break
            else:
                partial_result = rec.PartialResult()
                rospy.loginfo(f"Partial recognition: {eval(partial_result)['partial']}")

if __name__ == '__main__':
    try:
        listen_to_victim()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down listen_after_fall_node.")
