#!/usr/bin/env python

import rospy
import pyaudio
import time

from cordial_msgs.msg import Sound


FRAMES_PER_BUFFER = rospy.get_param(
    'cordial/sound/frames_per_buffer', default=512,
)


def get_speaker_device_index():
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        device = p.get_device_info_by_index(i)
        if device['name'] == 'sysdefault':
            return i


SPEAKER_DEVICE_INDEX = get_speaker_device_index()


def play_sound(data):

    rospy.loginfo("Sound received")

    p = pyaudio.PyAudio()

    # Occasionally pyaudio fails to open correctly.  Online resources said that this was likely because This is an attempt to fix it by closing down the node and restarting the node.

    try:
        stream = p.open(
            format=data.format,
            channels=data.num_channels,
            rate=data.framerate,
            output=True,
            frames_per_buffer=FRAMES_PER_BUFFER,
            output_device_index=SPEAKER_DEVICE_INDEX,
        )
        stream.write(data.data)
        time.sleep(0.05)
        stream.stop_stream()

        stream.close()
        p.terminate()
    except IOError:
        rospy.signal_shutdown("Audio device appears to be busy")


if __name__ == '__main__':

    rospy.init_node('sound_listener')
    rospy.Subscriber(rospy.get_param(
        'cordial_sound/play_stream_topic'), Sound, play_sound)
    rospy.spin()
