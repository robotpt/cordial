#!/usr/bin/env python

import rospy
from cordial_gui.srv import Ask, AskRequest


class ExampleGuiClient:

    _NODE_NAME = "gui_example_client"
    _ASK_SERVICE = "cordial/gui/ask"

    def __init__(self):

        rospy.init_node(self._NODE_NAME)
        self._client = rospy.ServiceProxy(self._ASK_SERVICE, Ask)

    def ask_text_entry(self):

        rospy.wait_for_service(self._ASK_SERVICE)

        try:
            request = AskRequest()
            request.display.type = "text entry"
            request.display.content = "What is your name?"
            request.display.buttons = ["Is my name"]
            request.display.args = ['']
            request.display.buttons_delay_seconds = 2.0

            return self._client(request)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def ask_multiple_choice(self):

        rospy.wait_for_service(self._ASK_SERVICE)

        try:
            request = AskRequest()
            request.display.type = "multiple choice"
            request.display.content = "How are you?"
            request.display.buttons = ["Great", "Okay", "Bad"]
            request.display.args = ['']
            request.display.buttons_delay_seconds = 2.0

            return self._client(request)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":

    gui_client = ExampleGuiClient()

    while True:

        response = gui_client.ask_text_entry()
        rospy.loginfo("I heard '%s'" % response)
        rospy.sleep(5)

        response = gui_client.ask_multiple_choice()
        rospy.loginfo("I heard '%s'" % response)
        rospy.sleep(5)
