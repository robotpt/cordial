#!/usr/bin/env python

import rospy
from cordial_gui.srv import Ask, AskRequest
from cordial_gui.msg import MouseEvent


class ExampleGuiClient:

    _NODE_NAME = "gui_example_client"
    _ASK_SERVICE = "cordial/say_and_ask_on_gui"

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
            request.display.args = ['Your name']

            return self._client(request)

        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    def ask_slider_entry(self):

        rospy.wait_for_service(self._ASK_SERVICE)

        try:
            request = AskRequest()
            request.display.type = "slider"
            request.display.content = "How many steps would you like to walk?"
            request.display.buttons = ["steps"]
            request.display.args = ["100", "500", "50", "200"]

            return self._client(request)

        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    def ask_time_entry(self):

        rospy.wait_for_service(self._ASK_SERVICE)

        try:
            request = AskRequest()
            request.display.type = "time entry"
            request.display.content = "When would you like to walk?"
            request.display.buttons = ["is when"]
            request.display.args = ["30", "12:30"]

            return self._client(request)

        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    def ask_multiple_choice(self):

        rospy.wait_for_service(self._ASK_SERVICE)

        try:
            request = AskRequest()
            request.display.type = "multiple choice"
            request.display.content = "How are you doing today? Is everything good?"
            request.display.buttons = ["Great", "Okay", "Bad"]
            request.display.args = []

            return self._client(request)

        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":

    gui_client = ExampleGuiClient()

    entry_callbacks = [
        gui_client.ask_text_entry,
        gui_client.ask_multiple_choice,
        gui_client.ask_slider_entry,
        gui_client.ask_time_entry,
    ]
    while True:
        for cb in entry_callbacks:
            cb()
            rospy.sleep(.5)
        rospy.sleep(5)
