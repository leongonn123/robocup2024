#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from gtts import gTTS
from io import BytesIO
from pydub import AudioSegment
from pydub.playback import play

# Initialize recognizer
recognizer = sr.Recognizer()

# Global variable for the publisher
acknowledge_pub = None

# Function to play the TTS audio
def play_audio(audio_data):
    song = AudioSegment.from_file(BytesIO(audio_data), format="mp3")
    play(song)

# Function to speak a text using gTTS
def speak(text):
    tts = gTTS(text=text, lang='en')
    audio_fp = BytesIO()
    tts.write_to_fp(audio_fp)
    audio_fp.seek(0)
    play_audio(audio_fp.read())

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
    with sr.Microphone() as source:
        recognizer.adjust_for_ambient_noise(source)
        rospy.loginfo("Listening for victim's response...")
        try:
            audio = recognizer.listen(source, timeout=10)
            text = recognizer.recognize_google(audio)
            rospy.loginfo(f"Recognized speech: {text}")
            if text:
                acknowledge_pub.publish(text)
                rospy.loginfo("Victim's response acknowledged.")
                speak(f"I heard you say: {text}. Help is on the way.")
        except sr.UnknownValueError:
            rospy.logwarn("Google Speech Recognition could not understand the audio.")
            speak("I'm sorry, I couldn't understand what you said. Please try again.")
        except sr.RequestError as e:
            rospy.logerr(f"Could not request results from Google Speech Recognition service; {e}")
            speak("I'm having trouble connecting to the speech recognition service.")

if __name__ == '__main__':
    try:
        listen_to_victim()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down listen_after_fall_node.")
