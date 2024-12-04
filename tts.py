#!/usr/bin/env python3

import rospy
from gtts import gTTS
from sound_play.libsoundplay import SoundClient
from pydub import AudioSegment
from std_msgs.msg import String
import os
import time

class TTSNode:
    def __init__(self):
        rospy.init_node('tts_node', anonymous=True)
        self.soundhandle = SoundClient()
        self.greeting_delay = 10  # Delay in seconds for each face
        self.last_greeting_times = {}  # Dictionary to track last greeting time for each face
        rospy.Subscriber('person_info', String, self.play_greeting)
        rospy.loginfo("TTS Node ready to greet!")

    def play_greeting(self, msg):
        face_name = msg.data
        current_time = time.time()

        # Check if the face has been greeted recently
        if face_name in self.last_greeting_times:
            time_since_last_greet = current_time - self.last_greeting_times[face_name]
            if time_since_last_greet < self.greeting_delay:
                rospy.logwarn(f"Skipping greeting for {face_name}. Please wait {self.greeting_delay - time_since_last_greet:.1f} seconds.")
                return  # Skip if the delay for this face hasn't passed

        # Update the last greeting time for this face
        self.last_greeting_times[face_name] = current_time
        rospy.loginfo(f"Greeting: {face_name}")
        self.play_text_as_audio(f"Hello, {face_name}!")

    def play_text_as_audio(self, text):
        try:
            tts = gTTS(text=text, lang='en')
            mp3_path = '/tmp/tts_output.mp3'
            wav_path = '/tmp/tts_output.wav'
            tts.save(mp3_path)
            sound = AudioSegment.from_mp3(mp3_path)
            sound.export(wav_path, format="wav")
            self.soundhandle.playWave(wav_path)
            rospy.loginfo("Played audio successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to play audio: {e}")

if __name__ == '__main__':
    try:
        TTSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
