#!/usr/bin/env python

import rospy
import rostest
import rosunit
import time
import unittest

from std_msgs.msg import String

from cordial_msgs.msg import FaceRequest

PKG = "cordial_manager"
NAME = "test_cordial_manager_pubs_and_subs"
_MANAGER_NODE = "manager"

_SAY_TOPIC = "cordial/say"
_PLAY_WAV_FILE_TOPIC = "cordial/sound/play/file_path/wav"
_PLAY_FACE_TOPIC = "cordial/face/play"
_PLAY_GESTURE_TOPIC = "cordial/gesture/play"
_IS_FACE_IDLE_TOPIC = "cordial/face/is_idle"
_IS_GO_TO_SLEEP_TOPIC = "cordial/sleep"


class TestCordialManagerPubsAndSubs(unittest.TestCase):

    def setUp(self):
        rospy.sleep(2)  # Gives time between tests for ROS to reset
        self._success = False

    def wave_file_subscriber_callback(self, msg):
        self._success = msg.data and msg.data == "test.wav"

    def test_is_wav_file_publisher_connected(self):
        assert rostest.is_publisher(
            rospy.resolve_name(_PLAY_WAV_FILE_TOPIC),
            rospy.resolve_name(_MANAGER_NODE)
        )

        rospy.Subscriber(
            _PLAY_WAV_FILE_TOPIC,
            String,
            callback=self.wave_file_subscriber_callback
        )

    def face_subscriber_callback(self, msg):
        self._success = msg

    def test_is_face_publisher_connected(self):
        assert rostest.is_publisher(
            rospy.resolve_name(_PLAY_FACE_TOPIC),
            rospy.resolve_name(_MANAGER_NODE)
        )

        rospy.Subscriber(
            _PLAY_FACE_TOPIC,
            FaceRequest,
            callback=self.face_subscriber_callback
        )

    def gesture_subscriber_callback(self, msg):
        self._success = msg

    def test_is_gesture_publisher_connected(self):
        assert rostest.is_publisher(
            rospy.resolve_name(_PLAY_GESTURE_TOPIC),
            rospy.resolve_name(_MANAGER_NODE)
        )

        test_gesture_subscriber = rospy.Subscriber(
            _PLAY_GESTURE_TOPIC,
            String,
            self.gesture_subscriber_callback
        )

    def is_idle_subscriber_callback(self, msg):
        self._success = msg

    def test_is_idle_publisher_connected(self):
        assert rostest.is_publisher(
            rospy.resolve_name(_IS_FACE_IDLE_TOPIC),
            rospy.resolve_name(_MANAGER_NODE)
        )

        test_is_idle_subscriber = rospy.Subscriber(
            _PLAY_GESTURE_TOPIC,
            String,
            self.gesture_subscriber_callback
        )

    def test_is_say_subscriber_connected(self):

        assert rostest.is_subscriber(
            rospy.resolve_name(_SAY_TOPIC),
            rospy.resolve_name(_MANAGER_NODE)
        )

    def test_is_go_to_sleep_subscriber_connected(self):

        assert rostest.is_subscriber(
            rospy.resolve_name(_IS_GO_TO_SLEEP_TOPIC),
            rospy.resolve_name(_MANAGER_NODE)
        )


if __name__ == "__main__":
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestCordialManagerPubsAndSubs)
