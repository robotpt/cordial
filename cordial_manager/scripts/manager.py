#!/usr/bin/env python

import actionlib
import rospy
import time
import threading

from aws_polly_client import AwsPollyClient

from std_msgs.msg import Bool, String
from cordial_msgs.msg import AskAction, AskGoal, AskResult, AskOnGuiAction, AskOnGuiResult, FaceRequest
from cordial_msgs.srv import SetString, SetStringResponse


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
            is_debug=False
    ):

        rospy.init_node(self._NODE_NAME, anonymous=False)

        if type(is_debug) is not bool:
            raise TypeError("is_debug must be either True or False")
        self._is_debug = is_debug
        if self._is_debug:
            rospy.loginfo("Running in debug mode")

        rospy.Subscriber(self._SAY_TOPIC, String, self._say_callback, queue_size=1)

        self._wav_file_publisher = rospy.Publisher(self._PLAY_WAV_FILE_TOPIC, String, queue_size=1)
        self._face_publisher = rospy.Publisher(self._PLAY_FACE_TOPIC, FaceRequest, queue_size=1)
        self._gesture_publisher = rospy.Publisher(self._PLAY_GESTURE_TOPIC, String, queue_size=1)
        self._is_idle_publisher = rospy.Publisher(self._IS_FACE_IDLE_TOPIC, Bool, queue_size=1)

        self._go_to_sleep_subscriber = rospy.Subscriber(self._IS_GO_TO_SLEEP_TOPIC, Bool, self._go_to_sleep_callback,
                                                        queue_size=1)

        self._play_sound_service = rospy.Service(self._SAY_TOPIC, SetString, self._say_service)

        self._say_and_ask_action_server = actionlib.SimpleActionServer(
            self._SAY_AND_ASK_ON_GUI_SERVICE,
            AskOnGuiAction,
            execute_cb=self._say_and_ask_on_gui_action_cb,
            auto_start=False
        )
        self._say_and_ask_action_server.register_preempt_callback(self._ask_preempt_callback)
        self._say_and_ask_action_server.start()

        self._gui_action_client = actionlib.SimpleActionClient(self._ASK_ON_GUI_SERVICE, AskAction)

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

    def _say_service(self, request):
        rospy.loginfo("cordial/say service called with text: {}".format(request.text))
        response = SetStringResponse()
        try:
            _, file_path, behavior_schedule = self._aws_client.run(request.text)
            self._say(file_path, behavior_schedule)
            time.sleep(behavior_schedule.get_last_start_time())
            response.succeeded = True
        except rospy.ServiceException as e:
            rospy.loginfo("Say service exception occurred")
            response.succeeded = False
            response.message = str(e)
        return response

    def _say_callback(self, data):
        self.say(data.data)

    def _say_and_ask_on_gui_action_cb(self, goal):

        if not self._is_debug:
            rospy.logdebug("Checking that GUI is connected to ROS websocket")
            rospy.wait_for_service(self._IS_GUI_CONNECTED_SERVICE)
            rospy.logdebug("Done, GUI is connected to ROS websocket")

        result = AskOnGuiResult()

        ask_goal = self._create_ask_goal(goal)
        ask_goal.display.content, file_path, behavior_schedule = self._aws_client.run(goal.content)
        ask_goal.display.buttons_delay_seconds = behavior_schedule.get_last_start_time()

        self._say(file_path, behavior_schedule)

        if not self._is_debug:
            self._gui_action_client.wait_for_server()
            self._gui_action_client.send_goal(ask_goal, feedback_cb=self._say_and_ask_action_server.publish_feedback)
            self._gui_action_client.wait_for_result()

            rospy.sleep(0.5)  # delay for stability, otherwise the gui may jump to the next question

            gui_response = self._gui_action_client.get_result()
        else:
            rospy.sleep(3)
            rospy.loginfo("Returning debug response")
            gui_response = AskResult(data="Debugging")

        try:
            result.data = gui_response.data
        except AttributeError:
            rospy.loginfo("GUI did not return any user input")

        if not self._say_and_ask_action_server.is_preempt_requested():
            rospy.loginfo("Setting goal as succeeded")
            self._say_and_ask_action_server.set_succeeded(result)

    def _create_ask_goal(self, goal):
        ask_goal = AskGoal()
        ask_goal.display.type = goal.type
        ask_goal.display.buttons = goal.options
        ask_goal.display.args = goal.args

        return ask_goal

    def _ask_preempt_callback(self):
        rospy.loginfo("Preempt sent from engine to manager")
        if not self._is_debug:
            self._gui_action_client.cancel_goal()
        rospy.loginfo("Setting goal as preempted")
        self._say_and_ask_action_server.set_preempted()

    def say(self, text):

        if not self._is_debug:
            rospy.logdebug("Checking that face is connected to ROS websocket")
            rospy.wait_for_service(self._IS_FACE_CONNECTED_SERVICE)
            rospy.logdebug("Done, face is connected to ROS websocket")

        _, file_path, behavior_schedule = self._aws_client.run(text)

        self._say(file_path, behavior_schedule)

    def _say(self, file_path, behavior_schedule):

        if not self._is_awake:
            self._wake_face()

        msg = String()
        msg.data = file_path
        self._wav_file_publisher.publish(msg)

        self._delay_publishing_visemes(
            behavior_schedule.get_visemes(self._min_viseme_duration_in_seconds)
        )
        self._delay_publishing_gestures(
            behavior_schedule.get_actions()
        )

    def _delay_publishing_visemes(self, visemes_to_play):

        def publish_visemes_callback_fn():
            self._face_publisher.publish(self._get_visemes_message(visemes_to_play))

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
        is_debug=rospy.get_param('cordial/manager/is_debug', False)
    )
    rospy.spin()
