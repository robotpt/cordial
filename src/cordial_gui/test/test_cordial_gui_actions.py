#!/usr/bin/env python

import actionlib
import rospy
import rostest
import unittest

from cordial_msgs.msg import AskAction, AskGoal, Display

PKG = "cordial_gui"
NAME = "test_cordial_gui_actions"
_ASK_ACTION = "cordial/gui/ask"


class TestCordialGuiActions(unittest.TestCase):

    def test_ask_action(self):

        client = actionlib.SimpleActionClient(
            _ASK_ACTION,
            AskAction
        )

        goal = AskGoal()
        display = Display()
        display.type = "multiple choice"
        display.content = "Hello there"
        display.buttons = ["Next"]
        display.args = []
        display.buttons_delay_seconds = 1
        goal.display = display

        self.assertTrue(client.wait_for_server(rospy.Duration(2)), "Could not connect")
        rospy.loginfo("Connected to server")

        client.send_goal(goal)

        self.assertTrue(client.wait_for_result(rospy.Duration(5)),
                        "Goal didn't finish")
        self.assertEqual(actionlib.GoalStatus.SUCCEEDED, client.get_state())
        self.assertEqual(client.get_result().data, "Debugging")

    def test_ask_action_preempt(self):
        client = actionlib.SimpleActionClient(
            _ASK_ACTION,
            AskAction
        )

        goal = AskGoal()
        display = Display()
        display.type = "multiple choice"
        display.content = "Hello there"
        display.buttons = ["Next"]
        display.args = []
        display.buttons_delay_seconds = 1
        goal.display = display

        self.assertTrue(client.wait_for_server(rospy.Duration(2)), "Could not connect")
        rospy.loginfo("Connected to server")

        client.send_goal(goal)
        rospy.sleep(1)
        client.cancel_goal()
        rospy.sleep(1)
        self.assertEqual(actionlib.GoalStatus.PREEMPTED, client.get_state())


if __name__ == "__main__":
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestCordialGuiActions)
