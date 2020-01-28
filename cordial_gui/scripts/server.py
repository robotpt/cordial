#!/usr/bin/env python

import datetime
import rospy

from std_msgs.msg import String
from cordial_gui.msg import Display, MouseEvent
from cordial_gui.srv import Ask, AskResponse


class GuiServer:

    _NODE_NAME = "gui_server"

    _MOUSE_EVENT_TOPIC = "cordial/gui/event/mouse"
    _KEYPRESS_EVENT_TOPIC = "cordial/gui/event/keypress"
    _USER_RESPONSE_TOPIC = "cordial/gui/user_response"
    _DISPLAY_TOPIC = "cordial/gui/display"
    _ASK_SERVICE = "cordial/gui/ask"

    def __init__(
            self,
            seconds_before_timeout,
            timeout_message,
    ):

        self._seconds_before_timeout = seconds_before_timeout
        self._last_active_time = None

        rospy.init_node(self._NODE_NAME)
        rospy.Subscriber(self._MOUSE_EVENT_TOPIC, MouseEvent, self._set_last_active_datetime)
        rospy.Subscriber(self._KEYPRESS_EVENT_TOPIC, String, self._set_last_active_datetime)

        self._display_publisher = rospy.Publisher(self._DISPLAY_TOPIC, Display, queue_size=1)
        self._prompt_server = rospy.Service(self._ASK_SERVICE, Ask, self._ask)
        self._timeout_message = timeout_message

    def _ask(self, ask_request):

        rospy.loginfo("Ask request recieved: '" + ask_request.display.content + "'")

        self._display_publisher.publish(ask_request.display)

        self._last_active_time = datetime.datetime.now()
        try:
            msg = self.wait_for_message_until_time_since_last_active_time_exceeds_timeout(
                self._USER_RESPONSE_TOPIC,
                String,
            )
            response = msg.data
        except TimeoutException:
            response = self._timeout_message

        ask_response = AskResponse()
        ask_response.data = response
        return ask_response

    def wait_for_message_until_time_since_last_active_time_exceeds_timeout(self, topic, topic_type):
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


class TimeoutException(rospy.ROSException):
    pass


if __name__ == '__main__':

    GuiServer(
        seconds_before_timeout=rospy.get_param('cordial/gui/seconds_before_timeout', 10),
        timeout_message=rospy.get_param('cordial/gui/timeout_msg', '<timeout>'),
    )
    rospy.spin()
