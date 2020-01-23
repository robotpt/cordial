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
            min_viseme_duration_in_seconds,
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
        self._min_viseme_duration_in_seconds = min_viseme_duration_in_seconds
        self._delay_to_publish_visemes_in_seconds = delay_to_publish_visemes_in_seconds

    def _say_callback(self, data):
        self.say(data.data)

    def say(self, text):

        _, file_path, behavior_schedule = self._aws_client.run(text)

        self._wav_file_publisher.publish(file_path)
        self._delay_publishing_visemes(
            behavior_schedule.get_visemes(
                min_duration_in_seconds=self._min_viseme_duration_in_seconds,
            )
        )

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


if __name__ == '__main__':

    manager = CordialManager(
        aws_voice_name=rospy.get_param('speech/aws/voice_name', 'Ivy'),
        aws_region_name=rospy.get_param('aws/region_name', 'us-west-1'),
        viseme_play_speed=rospy.get_param('speech/viseme/play_speed', 10),
        min_viseme_duration_in_seconds=rospy.get_param('speech/viseme/min_duration_in_seconds', 0.05),
        delay_to_publish_visemes_in_seconds=rospy.get_param('speech/viseme/publish_delay_in_seconds', 0.1),
    )

    rospy.spin()
