#!/usr/bin/env python

import rospy
import rostest
import unittest

from cordial_msgs.srv import SetString, SetStringRequest

PKG = "cordial_manager"
NAME = "test_cordial_manager_services"
_SAY_SERVICE = "cordial/say"


class TestCordialManagerServices(unittest.TestCase):

    def setUp(self):
        rospy.sleep(2)  # Gives time between tests for ROS to reset

    def test_play_sound_service(self):

        rospy.wait_for_service(_SAY_SERVICE)
        test_service_proxy = rospy.ServiceProxy(_SAY_SERVICE, SetString)
        test_messages = [
            "Testing",
            "Hello there",
            "How are you",
        ]

        for test_message in test_messages:
            test_response1 = test_service_proxy(SetStringRequest(test_message))
            test_response2 = test_service_proxy.call(SetStringRequest(test_message))
            self.assertEqual(test_response1.succeeded, test_response2.succeeded)
            self.assertTrue(test_response1.succeeded, "Service response was false")
            rospy.loginfo("Finished service call")


if __name__ == "__main__":
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestCordialManagerServices)
