#!/usr/bin/env python

import actionlib
import rospy
import rostest
import unittest

from cordial_msgs.msg import AskOnGuiAction, AskOnGuiGoal

PKG = "cordial_manager"
NAME = "test_cordial_manager_actions"
_SAY_AND_ASK_ON_GUI_ACTION = "cordial/say_and_ask_on_gui"


class TestCordialManagerActions(unittest.TestCase):

    def test_ask_on_gui_action(self):
        client = actionlib.SimpleActionClient(
            _SAY_AND_ASK_ON_GUI_ACTION,
            AskOnGuiAction
        )

        goal = AskOnGuiGoal()
        goal.type = "multiple choice"
        goal.content = "Hello there"
        goal.options = ["Next"]
        goal.args = []

        self.assertTrue(client.wait_for_server(rospy.Duration(2)), "Could not connect")
        rospy.loginfo("Connected to server")

        client.send_goal(goal)

        self.assertTrue(client.wait_for_result(rospy.Duration(5)),
                        "Goal didn't finish")
        self.assertEqual(actionlib.GoalStatus.SUCCEEDED, client.get_state())
        self.assertEqual(client.get_result().data, "Debugging")

    def test_ask_on_gui_action_preempt(self):
        client = actionlib.SimpleActionClient(
            _SAY_AND_ASK_ON_GUI_ACTION,
            AskOnGuiAction
        )

        goal = AskOnGuiGoal()
        goal.type = "multiple choice"
        goal.content = "Hello there"
        goal.options = ["Next"]
        goal.args = []

        self.assertTrue(client.wait_for_server(rospy.Duration(2)), "Could not connect")
        rospy.loginfo("Connected to server")

        client.send_goal(goal)
        rospy.sleep(1)
        client.cancel_goal()
        rospy.sleep(1)
        self.assertEqual(actionlib.GoalStatus.PREEMPTED, client.get_state())


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestCordialManagerActions)
