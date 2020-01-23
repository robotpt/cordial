#!/usr/bin/env python

import rospy
import pyaudio
import time

from cordial_sound.msg import Sound


def play_sound(data):

    rospy.loginfo("Sound received")

    p = pyaudio.PyAudio()

    stream = p.open(
        format=data.format,
        channels=data.num_channels,
        rate=data.framerate,
        output=True,
        frames_per_buffer=rospy.get_param(
            'frames_per_buffer', default=512
        ),
    )
    stream.write(data.data)
    time.sleep(1)
    stream.stop_stream()

    stream.close()
    p.terminate()


if __name__ == '__main__':

    rospy.init_node('sound_listener')
    rospy.Subscriber('/sound/play', Sound, play_sound)
    rospy.spin()

