#!/usr/bin/env python

import rospy
import pyaudio
import numpy as np
import wave
from pydub import AudioSegment
import math
import os
import tempfile

from cordial_msgs.msg import Sound
from std_msgs.msg import String


class WavFilePublisher:

    def __init__(self):

        rospy.init_node('wav_player', anonymous=True)
        self._wav_header_length = rospy.get_param(
            'cordial/sound/wav/header_length',
            24
        )
        rospy.Subscriber(rospy.get_param(
            'cordial_sound/play_wav_topic'), String, self.play_wav_file)
        self._sound_publisher = rospy.Publisher(rospy.get_param(
            'cordial_sound/play_stream_topic'), Sound, queue_size=1)
        self._pyaudio = pyaudio.PyAudio()

    def _split_audio_once(self, from_sec, to_sec, audio):
        t1 = from_sec * 1000  # seconds to milliseconds
        t2 = to_sec * 1000  # seconds to milliseconds
        split_audio = audio[t1:t2]
        return split_audio

    def split_audio_into_chunks(self, filename, seconds_per_split):
        audio = AudioSegment.from_wav(filename)
        total_seconds = math.ceil(audio.duration_seconds)
        audio_chunks = []
        for i in range(0, total_seconds, seconds_per_split):
            split_audio = self._split_audio_once(i, i+seconds_per_split, audio)
            audio_chunks.append(split_audio)
            if i == total_seconds - seconds_per_split:
                rospy.loginfo(
                    'The audio file splited successfully into chunks')
        return audio_chunks

    def publish_audio_chunk(self, wf, file_path, current_chunk_id):
        audio_format = self._pyaudio.get_format_from_width(
            wf.getsampwidth())
        framerate = wf.getframerate()
        num_channels = wf.getnchannels()

        data = np.fromfile(file_path, np.uint8)[self._wav_header_length:]
        data = data.astype(np.uint8).tostring()

        sound_msg = Sound()
        sound_msg.format = audio_format
        sound_msg.num_channels = num_channels
        sound_msg.framerate = framerate
        sound_msg.data = data

        rospy.loginfo(
            f"Publishing sound chunk {str(current_chunk_id)}/{str(len(self.audio_chunks))} from '{format(file_path)}'")
        self._sound_publisher.publish(sound_msg)

    def play_wav_file(self, data, seconds_per_split=int(rospy.get_param("cordial_sound/seconds_per_chunk"))):

        file_path = data.data

        try:
            wf_original = wave.open(file_path, 'rb')
        except IOError:
            rospy.logerr("Not a valid wav file: '{}'".format(file_path))
            return

        # Splitting wav to multiple parts of 1 sec each
        audio_chunks = self.split_audio_into_chunks(
            file_path, seconds_per_split=seconds_per_split)

        # Creating a temporary file for the audio
        temp_audio_file = tempfile.NamedTemporaryFile(suffix='.wav')

        # Publishing each chunk of 1 sec separtely
        for i, value in enumerate(audio_chunks):
            value.export(temp_audio_file.name, format="wav")

            wf = wave.open(temp_audio_file.name, 'rb')

            self.publish_audio_chunk(wf, file_path, i)


if __name__ == '__main__':

    WavFilePublisher()
    rospy.spin()
