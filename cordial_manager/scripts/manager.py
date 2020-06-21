#!/usr/bin/env python

import rospy
import threading

from aws_polly_client import AwsPollyClient

from std_msgs.msg import String
from std_msgs.msg import Bool
from cordial_face.msg import FaceRequest
from cordial_gui.srv import Ask


class CordialManager:

    _NODE_NAME = "cordial_manager"

    _SECONDS_BEFORE_TIMEOUT = 15

    _SAY_TOPIC = "cordial/say"
    _PLAY_WAV_FILE_TOPIC = "cordial/sound/play/file_path/wav"

    _IS_FACE_CONNECTED_SERVICE = "cordial/face/is_connected"
    _PLAY_FACE_TOPIC = "cordial/face/play"
    _PLAY_GESTURE_TOPIC = "cordial/gesture/play"
    _IS_FACE_IDLE_TOPIC = "cordial/face/is_idle"
    _IS_GO_TO_SLEEP_TOPIC = "cordial/sleep"

    _IS_GUI_CONNECTED_SERVICE = "cordial/gui/is_connected"
    _SAY_AND_ASK_ON_GUI_SERVICE = "cordial/say_and_ask_on_gui"
    _ASK_ON_GUI_SERVICE = "cordial/gui/ask"

    def __init__(
            self,
            aws_voice_name,
            aws_region_name,
            viseme_play_speed,
            min_viseme_duration_in_seconds,
            delay_to_publish_visemes_in_seconds,
            delay_to_publish_gestures_in_seconds=None,
    ):

        rospy.init_node(self._NODE_NAME, anonymous=False)
        rospy.Subscriber(self._SAY_TOPIC, String, self._say_callback, queue_size=1)

        self._wav_file_publisher = rospy.Publisher(self._PLAY_WAV_FILE_TOPIC, String, queue_size=1)
        self._face_publisher = rospy.Publisher(self._PLAY_FACE_TOPIC, FaceRequest, queue_size=1)
        self._gesture_publisher = rospy.Publisher(self._PLAY_GESTURE_TOPIC, String, queue_size=1)
        self._is_idle_publisher = rospy.Publisher(self._IS_FACE_IDLE_TOPIC, Bool, queue_size=1)

        self._go_to_sleep_subscriber = rospy.Subscriber(self._IS_GO_TO_SLEEP_TOPIC, Bool, self._go_to_sleep_callback, queue_size=1)

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
        if delay_to_publish_gestures_in_seconds is None:
            delay_to_publish_gestures_in_seconds = delay_to_publish_visemes_in_seconds
        self._delay_to_publish_gestures_in_seconds = delay_to_publish_gestures_in_seconds

        self._is_awake = True

    def _go_to_sleep_callback(self, msg):
        is_should_wakeup = not msg.data

        if is_should_wakeup:
            if self._is_awake:
                rospy.logdebug("Already awake")
            else:
                rospy.loginfo("Waking up")
                self._wake_face()
        else:
            if self._is_awake:
                rospy.loginfo("Going to sleep")
                self._sleep_face()
            else:
                rospy.logdebug("Already asleep")

    def _sleep_face(self):
        self._is_idle_publisher.publish(Bool(data=False))
        rospy.sleep(2)
        self._gesture_publisher.publish(String(data="close_eyes"))
        self._is_awake = False

    def _wake_face(self):
        self._gesture_publisher.publish(String(data="open_eyes"))
        rospy.sleep(2)
        self._is_idle_publisher.publish(Bool(data=True))
        self._is_awake = True

    def _say_callback(self, data):
        self.say(data.data)

    def _say_and_ask_on_gui(self, request):

        rospy.logdebug("Checking that GUI is connected to ROS websocket")
        rospy.wait_for_service(self._IS_GUI_CONNECTED_SERVICE)
        rospy.logdebug("Done, GUI is connected to ROS websocket")

        content = request.display.content
        request.display.content, file_path, behavior_schedule = self._aws_client.run(content)
        request.display.buttons_delay_seconds = behavior_schedule.get_last_start_time()

        self._say(file_path, behavior_schedule)
        try:
            response = self._gui_client(request)
            rospy.sleep(0.5)  # delay for stability, otherwise the gui may jump to the next question
            return response
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    def say(self, text):

        rospy.logdebug("Checking that face is connected to ROS websocket")
        rospy.wait_for_service(self._IS_FACE_CONNECTED_SERVICE)
        rospy.logdebug("Done, face is connected to ROS websocket")

        _, file_path, behavior_schedule = self._aws_client.run(text)

        self._say(file_path, behavior_schedule)

    def _say(self, file_path, behavior_schedule):

        if not self._is_awake:
            self._wake_face()

        self._wav_file_publisher.publish(file_path)
        self._delay_publishing_visemes(
            behavior_schedule.get_visemes(self._min_viseme_duration_in_seconds)
        )
        self._delay_publishing_gestures(
            behavior_schedule.get_actions()
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

    def _delay_publishing_gestures(
            self,
            gestures,
    ):

        def get_gesture_callback_fn(gesture):
            def callback():
                self._gesture_publisher.publish(
                    String(gesture)
                )
            return callback

        for a in gestures:
            threading.Timer(
                self._delay_to_publish_gestures_in_seconds + a['start'],
                get_gesture_callback_fn(a['id']),
                ).start()


if __name__ == '__main__':

    CordialManager(
        aws_region_name=rospy.get_param('aws/region_name', 'us-west-1'),
        aws_voice_name=rospy.get_param('cordial/speech/aws/voice_name', 'Ivy'),
        viseme_play_speed=rospy.get_param('cordial/speech/viseme/play_speed', 10),
        min_viseme_duration_in_seconds=rospy.get_param('cordial/speech/viseme/min_duration_in_seconds', 0.05),
        delay_to_publish_visemes_in_seconds=rospy.get_param('cordial/speech/viseme/publish_delay_in_seconds', 0.1),
    )
    rospy.spin()
