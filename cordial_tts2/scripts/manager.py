#!/usr/bin/env python

import rospy
import threading

from std_msgs.msg import String
from aws_polly_client import AwsPollyClient
from cordial_face.msg import FaceRequest


class CordialManager:

    def __init__(
            self,
            aws_voice_name,
            aws_region_name,
            viseme_play_speed,
            delay_to_publish_visemes_in_seconds,
    ):

        rospy.init_node('manager')
        rospy.Subscriber('/cordial/say', String, self._say_callback)
        self._wav_file_publisher = rospy.Publisher('/sound/from_file/wav', String, queue_size=1)
        self._behavior_publisher = rospy.Publisher('/cordial/behaviors', String, queue_size=1)
        self._face_publisher = rospy.Publisher('/cordial/face', FaceRequest, queue_size=1)

        self._aws_client = AwsPollyClient(
            voice=aws_voice_name,
            region=aws_region_name,
        )

        self._viseme_play_speed = viseme_play_speed
        self._delay_to_publish_visemes_in_seconds = delay_to_publish_visemes_in_seconds

    def _say_callback(self, data):
        self.say(data.data)

    def say(self, text):

        _, file_path, behavior_schedule = self._aws_client.run(text)

        visemes_schedule = _filter_behaviors_by_type("viseme", behavior_schedule)
        visemes_to_play = _get_longer_duration_visemes(visemes_schedule)

        self._wav_file_publisher.publish(file_path)
        self._delay_publishing_visemes(visemes_to_play)

    def _delay_publishing_visemes(self, visemes_to_play):

        def publish_visemes_callback_fn():
            self._face_publisher.publish(
                self._get_visemes_message(visemes_to_play)
            )

        threading.Timer(
            self._delay_to_publish_visemes_in_seconds,
            publish_visemes_callback_fn,
        ).start()

    def _get_visemes_message(self, visemes_to_play):
        return FaceRequest(
            visemes=map(lambda b: b["id"], visemes_to_play),
            times=map(lambda b: b["start"], visemes_to_play),
            viseme_ms=self._viseme_play_speed,
        )


def _filter_behaviors_by_type(desired_type, behavior_schedule):
    return filter(lambda ele: ele["type"] == desired_type, behavior_schedule)


def _get_durations_of_visemes(visemes_schedule, last_element_duration=0.05):
    out = []
    for i in range(0, len(visemes_schedule) - 1):
        out.append(
            visemes_schedule[i + 1]["start"] - visemes_schedule[i]["start"]
        )
    out.append(last_element_duration)
    return out


def _get_longer_duration_visemes(visemes_schedule, min_duration_in_seconds=0.05):
    out = []
    durations = _get_durations_of_visemes(visemes_schedule, min_duration_in_seconds)
    for i in range(len(visemes_schedule)):
        if durations[i] >= min_duration_in_seconds:
            out.append(visemes_schedule[i])
    return out


if __name__ == '__main__':

    manager = CordialManager(
        aws_voice_name=rospy.get_param('voice', 'Ivy'),
        aws_region_name=rospy.get_param('region', 'us-west-1'),
        viseme_play_speed=rospy.get_param('viseme_play_speed', 10),
        delay_to_publish_visemes_in_seconds=rospy.get_param('delay_to_publish_visemes_in_seconds', 0.1),
    )

    rospy.spin()
