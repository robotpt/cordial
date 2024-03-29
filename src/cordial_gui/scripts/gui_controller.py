#!/usr/bin/env python3.8

import actionlib
import datetime
import rospy

from std_msgs.msg import String, Empty
from cordial_msgs.msg import AskAction, AskFeedback, AskResult, Display, MouseEvent


class GuiController:
    _NODE_NAME = "gui_controller"

    _ASK_SERVICE = "cordial/gui/ask"
    _IS_GUI_CONNECTED_SERVICE = "cordial/gui/is_connected"

    class State:
        def __init__(self):
            pass

        BLACK_SCREEN = "black screen"
        TRANSITION_TO_BLACK_SCREEN = "transition to black screen"
        WAITING_FOR_USER_RESPONSE = "waiting for user response"
        WAITING_FOR_ANOTHER_ASK_REQUEST = "waiting for another ask request"

    def __init__(self):
        rospy.init_node(self._NODE_NAME)

        self._seconds_before_timeout = rospy.get_param(
            'cordial/gui/seconds_before_timeout',
            60,
        )
        self._timeout_message = rospy.get_param(
            'cordial/gui/timeout_msg',
            '<timeout>',
        )
        self._seconds_with_no_prompt_before_display_goes_off = rospy.get_param(
            'cordial/gui/seconds_with_no_prompt_before_display_goes_off',
            1,
        )
        self._state_manager_seconds_between_calls = rospy.get_param(
            'cordial/gui/state_manager_seconds_between_calls',
            0.25,
        )
        self._is_debug = rospy.get_param(
            'cordial/gui/is_debug',
            False
        )

        if self._is_debug:
            rospy.loginfo("Running in debug mode")

        self._last_active_time = None
        self._gui_state = None
        self._last_response_time = None

        rospy.Subscriber(rospy.get_param("cordial_gui/event/mouse_topic"), MouseEvent, self._set_last_active_datetime)
        rospy.Subscriber(rospy.get_param("cordial_gui/event/keypress_topic"), String, self._set_last_active_datetime)
        rospy.Subscriber(rospy.get_param("cordial_gui/event/new_server_topic"), Empty, self._show_black_screen_cb)

        self._display_publisher = rospy.Publisher(rospy.get_param("cordial_gui/display_topic"), Display, queue_size=1)
        self._prompt_publisher = rospy.Publisher(rospy.get_param("cordial_gui/prompt_topic"), Empty, queue_size=1)
        self._prompt_action_server = actionlib.SimpleActionServer(
            self._ASK_SERVICE,
            AskAction,
            self._ask_action_cb,
            auto_start=False,
        )
        self._prompt_action_server.register_preempt_callback(self._preempt_callback)
        self._prompt_action_server.start()

        if not self._is_debug:
            rospy.wait_for_service(self._IS_GUI_CONNECTED_SERVICE)
        rospy.timer.Timer(
            rospy.Duration(
                nsecs=int(
                    self._state_manager_seconds_between_calls * 1e9
                )
            ),
            self._state_manager,
        )

    def _state_manager(self, _):
        """
        This manages transitions from one state to another and is called regularly by a rospy.timer.Timer
        """

        rospy.logdebug("Entering timer with state '{}'".format(self._gui_state))

        if self._gui_state in [None, self.State.TRANSITION_TO_BLACK_SCREEN]:
            self._show_black_screen()
            self._gui_state = self.State.BLACK_SCREEN
        elif self.State.WAITING_FOR_ANOTHER_ASK_REQUEST == self._gui_state:
            if self._is_time_passed_to_wait_for_another_ask_request():
                self._gui_state = self.State.TRANSITION_TO_BLACK_SCREEN
            else:
                pass
        elif self._gui_state in [self.State.WAITING_FOR_USER_RESPONSE, self.State.BLACK_SCREEN]:
            """
            The state gets set to WAITING_FOR_USER_RESPONSE in the _ask function, which is 
            called by a service
            """
        else:
            raise NotImplementedError("No rule on handling state '{}'".format(self._gui_state))

    def _is_time_passed_to_wait_for_another_ask_request(self):
        if self._last_response_time is None:
            return False
        seconds_since_response = (datetime.datetime.now() - self._last_response_time).seconds
        return seconds_since_response > self._seconds_with_no_prompt_before_display_goes_off

    def _show_black_screen_cb(self, _):
        self._show_black_screen()

    def _show_black_screen(self):

        display_msg = Display()
        display_msg.type = "black"
        rospy.loginfo("Publishing to display to switch to black screen")
        self._display_publisher.publish(display_msg)
        self._gui_state = self.State.BLACK_SCREEN

    def _set_last_active_datetime(self, _):

        self._last_active_time = datetime.datetime.now()
        rospy.loginfo("User activity detected - updating last active time")

        if self._gui_state == self.State.BLACK_SCREEN:
            rospy.loginfo("Publishing to prompt topic")
            self._prompt_publisher.publish()

    def _ask_action_cb(self, goal):

        class WaitForMessage(object):

            def __init__(self):
                self.msg = None

            def cb(self, msg):
                if self.msg is None:
                    self.msg = msg

        result = AskResult()

        rospy.loginfo("Ask goal received: '" + goal.display.content + "'")

        self._display_publisher.publish(goal.display)
        self._gui_state = self.State.WAITING_FOR_USER_RESPONSE

        self._last_active_time = datetime.datetime.now()

        feedback_timer = rospy.timer.Timer(
            rospy.Duration(
                nsecs=int(
                    1 * 1e9
                )
            ),
            self._publish_action_feedback,
        )

        wfm = WaitForMessage()
        if not self._is_debug:
            s = None
            try:
                s = rospy.topics.Subscriber(rospy.get_param("cordial_gui/response_topic"), String, wfm.cb)
                while (
                        not rospy.core.is_shutdown()
                        and wfm.msg is None
                        and self._gui_state == self.State.WAITING_FOR_USER_RESPONSE
                        and not self._prompt_action_server.is_preempt_requested()
                ):
                    rospy.rostime.wallsleep(0.01)

            finally:
                if s is not None:
                    s.unregister()

                self._last_response_time = datetime.datetime.now()
        else:
            seconds_to_sleep_for_tests = 3
            rospy.sleep(seconds_to_sleep_for_tests)
            rospy.loginfo("Returning debug response")
            wfm.msg = String(data="Debugging")

        rospy.loginfo("Shutting down feedback timer")
        feedback_timer.shutdown()

        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("rospy shutdown")

        if wfm.msg is not None:
            result.data = wfm.msg.data

        rospy.sleep(0.5)  # Allow for fading to occur with this delay

        if not self._prompt_action_server.is_preempt_requested():
            rospy.loginfo("Setting goal as succeeded")
            self._prompt_action_server.set_succeeded(result)

        self._gui_state = self.State.WAITING_FOR_ANOTHER_ASK_REQUEST

    def _publish_action_feedback(self, _):
        feedback = AskFeedback()
        feedback.time_passed = (datetime.datetime.now() - self._last_active_time).seconds
        self._prompt_action_server.publish_feedback(feedback)
        # rospy.loginfo("Feedback: {}".format(feedback.time_passed))

    def _preempt_callback(self):
        rospy.loginfo("Preempt sent from manager to GUI")
        rospy.loginfo("Setting goal as preempted")
        self._prompt_action_server.set_preempted()


class TimeoutException(rospy.ROSException):
    pass


if __name__ == '__main__':
    GuiController()
    rospy.spin()
