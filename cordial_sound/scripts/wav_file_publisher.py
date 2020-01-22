#!/usr/bin/env python

import rospy
import pyaudio
import numpy as np
import wave

from cordial_sound.msg import Sound
from std_msgs.msg import String


class WavFilePublisher:

    def __init__(
            self,
            wav_header_length,
    ):

        rospy.init_node('wav_player', anonymous=True)
        rospy.Subscriber('/sound/from_file/wav', String, self.play_wav_file)
        self._sound_publisher = rospy.Publisher("/sound/play", Sound, queue_size=1)
        self._pyaudio = pyaudio.PyAudio()
        self._wav_header_length = wav_header_length

    def play_wav_file(self, data):

        file_path = data.data

        try:
            wf = wave.open(file_path, 'rb')
        except IOError:
            rospy.logerr("Not a valid wav file: '{}'".format(file_path))
            return

        audio_format = self._pyaudio.get_format_from_width(wf.getsampwidth())
        framerate = wf.getframerate()
        num_channels = wf.getnchannels()

        data = np.fromfile(file_path, np.uint8)[self._wav_header_length:]
        data = data.astype(np.uint8).tostring()

        sound_msg = Sound()
        sound_msg.format = audio_format
        sound_msg.num_channels = num_channels
        sound_msg.framerate = framerate
        sound_msg.data = data

        rospy.loginfo("Publishing sound from '{}'".format(file_path))
        self._sound_publisher.publish(sound_msg)


if __name__ == '__main__':

    WavFilePublisher(
        wav_header_length=rospy.get_param('wav_header_length', 24)
    )
    rospy.spin()
