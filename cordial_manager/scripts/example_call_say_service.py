#!/usr/bin/env python

import rospy
from cordial_manager.srv import SetString, SetStringRequest


class ExampleSayClient:

    _NODE_NAME = "example_say_client"
    _SAY_SERVICE = "cordial/say"

    def __init__(self):

        rospy.init_node(self._NODE_NAME)
        self._client = rospy.ServiceProxy(self._SAY_SERVICE, SetString)

    def send_say_request(self, text):
        rospy.wait_for_service(self._SAY_SERVICE)

        try:
            say_request = SetStringRequest()
            say_request.text = text

            return self._client(say_request)

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed")


if __name__ == "__main__":

    say_client = ExampleSayClient()
    text_to_say = [
        "Hello there!",
        "How are you doing today?",
        "What's your name?"
    ]
    for text in text_to_say:
        say_client.send_say_request(text)
        rospy.sleep(2)