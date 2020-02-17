#!/usr/bin/env python

import datetime
import rospy

from std_msgs.msg import String, Empty
from cordial_gui.msg import Display, MouseEvent
from cordial_gui.srv import Ask, AskResponse


class GuiController:

    _NODE_NAME = "gui_server"

    _MOUSE_EVENT_TOPIC = "cordial/gui/event/mouse"
    _KEYPRESS_EVENT_TOPIC = "cordial/gui/event/keypress"
    _NEW_SERVER_EVENT_TOPIC = "cordial/gui/event/new_server"
    _USER_RESPONSE_TOPIC = "cordial/gui/user_response"
    _DISPLAY_TOPIC = "cordial/gui/display"
    _USER_PROMPTED_TOPIC = "cordial/gui/prompt"
    _ASK_SERVICE = "cordial/gui/ask"
    _IS_GUI_CONNECTED_SERVICE = "cordial/gui/is_connected"

    class State:
        def __init__(self):
            pass
        BLACK_SCREEN = "black screen"
        TRANSITION_TO_BLACK_SCREEN = "transition to black screen"
        WAITING_FOR_USER_RESPONSE = "waiting for user response"
        WAITING_FOR_ANOTHER_ASK_REQUEST = "waiting for another ask request"
        TIMEOUT = "timeout"

    def __init__(
            self,
            seconds_before_timeout,
            timeout_message,
            seconds_with_no_prompt_before_display_goes_off,
            state_manager_seconds_between_calls,
    ):

        self._seconds_before_timeout = seconds_before_timeout
        self._seconds_with_no_prompt_before_display_goes_off = seconds_with_no_prompt_before_display_goes_off
        self._last_active_time = None
        self._gui_state = None
        self._last_response_time = None

        rospy.init_node(self._NODE_NAME)
        rospy.Subscriber(self._MOUSE_EVENT_TOPIC, MouseEvent, self._set_last_active_datetime)
        rospy.Subscriber(self._KEYPRESS_EVENT_TOPIC, String, self._set_last_active_datetime)
        rospy.Subscriber(self._NEW_SERVER_EVENT_TOPIC, Empty, self._show_black_screen_cb)

        self._display_publisher = rospy.Publisher(self._DISPLAY_TOPIC, Display, queue_size=1)
        self._prompt_publisher = rospy.Publisher(self._USER_PROMPTED_TOPIC, Empty, queue_size=1)
        self._prompt_server = rospy.Service(self._ASK_SERVICE, Ask, self._ask)
        self._timeout_message = timeout_message

        rospy.wait_for_service(self._IS_GUI_CONNECTED_SERVICE)
        rospy.timer.Timer(
            rospy.Duration(
                nsecs=int(
                    state_manager_seconds_between_calls*1e9
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
        elif self.State.TIMEOUT == self._gui_state:
            self._gui_state = self.State.TRANSITION_TO_BLACK_SCREEN
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

    def _ask(self, ask_request):

        rospy.loginfo("Ask request recieved: '" + ask_request.display.content + "'")

        self._display_publisher.publish(ask_request.display)
        self._gui_state = self.State.WAITING_FOR_USER_RESPONSE

        self._last_active_time = datetime.datetime.now()
        try:
            msg = self._wait_for_message_until_time_since_last_active_time_exceeds_timeout(
                self._USER_RESPONSE_TOPIC,
                String,
            )
            response = msg.data
            rospy.loginfo("Response recieved: '{}'".format(response))
            self._gui_state = self.State.WAITING_FOR_ANOTHER_ASK_REQUEST
        except TimeoutException:
            response = self._timeout_message
            self._gui_state = self.State.TIMEOUT
        finally:
            self._last_response_time = datetime.datetime.now()

        rospy.sleep(0.5)  # Allow for fading to occur with this delay

        ask_response = AskResponse()
        ask_response.data = response
        return ask_response

    def _wait_for_message_until_time_since_last_active_time_exceeds_timeout(self, topic, topic_type):
        """
        This function is similar to `rospy.wait_for_message()` except it uses a class variable to define the timeout.
        """

        class WaitForMessage(object):

            def __init__(self):
                self.msg = None

            def cb(self, msg):
                if self.msg is None:
                    self.msg = msg

        wfm = WaitForMessage()
        s = None
        try:
            s = rospy.topics.Subscriber(topic, topic_type, wfm.cb)
            while (
                    not rospy.core.is_shutdown()
                    and wfm.msg is None
                    and (datetime.datetime.now() - self._last_active_time).seconds < self._seconds_before_timeout
                    and self._gui_state == self.State.WAITING_FOR_USER_RESPONSE
            ):
                rospy.rostime.wallsleep(0.01)
        finally:
            if s is not None:
                s.unregister()
        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("rospy shutdown")

        if wfm.msg is not None:
            return wfm.msg
        else:
            raise TimeoutException

    def _set_last_active_datetime(self, _):

        self._last_active_time = datetime.datetime.now()
        rospy.loginfo("User activity detected - updating last active time")

        if self._gui_state == self.State.BLACK_SCREEN:
            rospy.loginfo("Publishing to prompt topic")
            self._prompt_publisher.publish()


class TimeoutException(rospy.ROSException):
    pass


if __name__ == '__main__':

    GuiController(
        seconds_before_timeout=rospy.get_param(
            'cordial/gui/seconds_before_timeout',
            60,
        ),
        timeout_message=rospy.get_param(
            'cordial/gui/timeout_msg',
            '<timeout>',
        ),
        seconds_with_no_prompt_before_display_goes_off=rospy.get_param(
            'cordial/gui/seconds_with_no_prompt_before_display_goes_off',
            1,
        ),
        state_manager_seconds_between_calls=rospy.get_param(
            'cordial/gui/state_manager_seconds_between_calls',
            0.25,
        )
    )
    rospy.spin()
