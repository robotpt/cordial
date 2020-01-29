#!/usr/bin/env python

import rospy
import threading

from aws_polly_client import AwsPollyClient

from std_msgs.msg import String
from cordial_face.msg import FaceRequest
from cordial_gui.srv import Ask


class CordialManager:

    _NODE_NAME = "cordial_manager"

    _SAY_TOPIC = "cordial/say"
    _PLAY_WAV_FILE_TOPIC = "cordial/sound/play/file_path/wav"
    _PLAY_FACE_TOPIC = "cordial/face/play"

    _SAY_AND_ASK_ON_GUI_SERVICE = "cordial/say_and_ask_on_gui"
    _ASK_ON_GUI_SERVICE = "cordial/gui/ask"

    def __init__(
            self,
            aws_voice_name,
            aws_region_name,
            viseme_play_speed,
            min_viseme_duration_in_seconds,
            delay_to_publish_visemes_in_seconds,
    ):

        rospy.init_node(self._NODE_NAME, anonymous=False)
        rospy.Subscriber(self._SAY_TOPIC, String, self._say_callback)

        self._wav_file_publisher = rospy.Publisher(self._PLAY_WAV_FILE_TOPIC, String, queue_size=1)
        self._face_publisher = rospy.Publisher(self._PLAY_FACE_TOPIC, FaceRequest, queue_size=1)

        self._say_and_ask_server = rospy.Service(
            self._SAY_AND_ASK_ON_GUI_SERVICE,
            Ask,
            self._say_and_ask_on_gui
        )
        self._gui_client = rospy.ServiceProxy(self._ASK_ON_GUI_SERVICE, Ask)

        self._aws_client = AwsPollyClient(
            voice=aws_voice_name,
            region=aws_region_name,
        )

        self._viseme_play_speed = viseme_play_speed
        self._min_viseme_duration_in_seconds = min_viseme_duration_in_seconds
        self._delay_to_publish_visemes_in_seconds = delay_to_publish_visemes_in_seconds

    def _say_callback(self, data):
        self.say(data.data)

    def _say_and_ask_on_gui(self, request):

        rospy.wait_for_service(self._ASK_ON_GUI_SERVICE)

        content = request.display.content
        request.display.content, file_path, behavior_schedule = self._aws_client.run(content)

        self._say(file_path, behavior_schedule)
        try:
            return self._gui_client(request)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    def say(self, text):

        _, file_path, behavior_schedule = self._aws_client.run(text)

        self._say(file_path, behavior_schedule)

    def _say(self, file_path, behavior_schedule):

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

    CordialManager(
        aws_region_name=rospy.get_param('aws/region_name', 'us-west-1'),
        aws_voice_name=rospy.get_param('cordial/speech/aws/voice_name', 'Ivy'),
        viseme_play_speed=rospy.get_param('cordial/speech/viseme/play_speed', 10),
        min_viseme_duration_in_seconds=rospy.get_param('cordial/speech/viseme/min_duration_in_seconds', 0.05),
        delay_to_publish_visemes_in_seconds=rospy.get_param('cordial/speech/viseme/publish_delay_in_seconds', 0.1),
    )
    rospy.spin()
