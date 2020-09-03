#!/usr/bin/env python

import actionlib
import rospy
from cordial_msgs.msg import AskOnGuiAction, AskOnGuiGoal


class ExampleGuiClient:

    _NODE_NAME = "gui_example_client"
    _ASK_SERVICE = "cordial/say_and_ask_on_gui"

    def __init__(self):

        rospy.init_node(self._NODE_NAME)
        self._action_client = actionlib.SimpleActionClient(self._ASK_SERVICE, AskOnGuiAction)

    def ask_text_entry(self):

        goal = AskOnGuiGoal()
        goal.type = "text entry"
        goal.content = "What is your name?"
        goal.options = ["Is my name"]
        goal.args = ["Your name"]

        self._action_client.wait_for_server()

        self._action_client.send_goal(goal)
        self._action_client.wait_for_result()
        response = self._action_client.get_result()

    def ask_slider_entry(self):

        goal = AskOnGuiGoal()
        goal.type = "slider"
        goal.content = "How many steps would you like to walk?"
        goal.options = ["steps"]
        goal.args = ["100", "500", "50", "200"]

        self._action_client.wait_for_server()

        self._action_client.send_goal(goal)
        self._action_client.wait_for_result()
        response = self._action_client.get_result()

    def ask_time_entry(self):

        goal = AskOnGuiGoal()
        goal.type = "time entry"
        goal.content = "When would you like to walk?"
        goal.options = ["is when"]
        goal.args = ["30", "12:30"]

        self._action_client.wait_for_server()

        self._action_client.send_goal(goal)
        self._action_client.wait_for_result()
        response = self._action_client.get_result()

        rospy.wait_for_service(self._ASK_SERVICE)

    def ask_multiple_choice(self):

        goal = AskOnGuiGoal()
        goal.type = "multiple choice"
        goal.content = "How are you doing today? Is everything good?"
        goal.options = ["Great", "Okay", "Bad"]
        goal.args = []

        self._action_client.wait_for_server()

        self._action_client.send_goal(goal)
        self._action_client.wait_for_result()
        response = self._action_client.get_result()


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
        rospy.sleep(5)
