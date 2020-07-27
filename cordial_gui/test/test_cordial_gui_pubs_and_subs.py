#!/usr/bin/env python

import rospy
import rostest
import rosunit
import time
import unittest

from cordial_msgs.msg import Display
from std_msgs.msg import Empty

PKG = "cordial_manager"
NAME = "test_cordial_gui_pubs_and_subs"
_GUI_NODE = "gui_controller"

_MOUSE_EVENT_TOPIC = "cordial/gui/event/mouse"
_KEYPRESS_EVENT_TOPIC = "cordial/gui/event/keypress"
_NEW_SERVER_EVENT_TOPIC = "cordial/gui/event/new_server"
_USER_RESPONSE_TOPIC = "cordial/gui/user_response"
_DISPLAY_TOPIC = "cordial/gui/display"
_USER_PROMPTED_TOPIC = "cordial/gui/prompt"


class TestCordialGuiPubsAndSubs(unittest.TestCase):

    def setUp(self):
        self._success = False

    def display_subscriber_callback(self, msg):
        self._success = msg

    def test_is_display_publisher_connected(self):

        assert rostest.is_publisher(
            rospy.resolve_name(_DISPLAY_TOPIC),
            rospy.resolve_name(_GUI_NODE)
        )

        test_display_subscriber = rospy.Subscriber(
            _DISPLAY_TOPIC,
            Display,
            callback=self.display_subscriber_callback
        )

    def prompt_subscriber_callback(self, msg):
        self._success = msg

    def test_is_prompt_publisher_connected(self):

        assert rostest.is_publisher(
            rospy.resolve_name(_USER_PROMPTED_TOPIC),
            rospy.resolve_name(_GUI_NODE)
        )

        test_display_subscriber = rospy.Subscriber(
            _USER_PROMPTED_TOPIC,
            Empty,
            callback=self.display_subscriber_callback
        )

    def test_is_mouse_event_subscriber_connected(self):

        assert rostest.is_subscriber(
            rospy.resolve_name(_MOUSE_EVENT_TOPIC),
            rospy.resolve_name(_GUI_NODE)
        )

    def test_is_keypress_event_subscriber_connected(self):

        assert rostest.is_subscriber(
            rospy.resolve_name(_KEYPRESS_EVENT_TOPIC),
            rospy.resolve_name(_GUI_NODE)
        )

    def test_is_new_server_event_subscriber_connected(self):

        assert rostest.is_subscriber(
            rospy.resolve_name(_NEW_SERVER_EVENT_TOPIC),
            rospy.resolve_name(_GUI_NODE)
        )


if __name__ == "__main__":
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestCordialGuiPubsAndSubs)
