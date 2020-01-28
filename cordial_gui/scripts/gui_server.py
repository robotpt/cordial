#!/usr/bin/env python


import rospy

from std_msgs.msg import String
from cordial_gui.msg import Display, MouseEvent
from cordial_gui.srv import Ask, AskResponse


class GuiServer:

    def __init__(
            self,
            seconds_before_timeout,
    ):

        rospy.init_node("gui_server")

        rospy.Subscriber("cordial/gui/event/mouse", MouseEvent, self._timeout_fn)
        rospy.Subscriber("cordial/gui/event/keypress", String, self._timeout_fn)

        rospy.Subscriber("cordial/gui/user_response", String, self._response_fn)
        self._display_publisher = rospy.Publisher("cordial/gui/display", Display, queue_size=1)
        self._prompt_server = rospy.Service('cordial/gui/ask', Ask, self._ask)

        self._seconds_before_timeout = seconds_before_timeout

    def _ask(self, ask_request):

        rospy.loginfo("I heard '" + ask_request.display.content + "'")

        display_msg = ask_request.display
        self._display_publisher.publish(display_msg)

        user_response = rospy.wait_for_message("cordial/gui/user_response", String)

        response = AskResponse()
        response.response = user_response.data
        return response

    def _timeout_fn(self, data):

        rospy.loginfo("Mouse or click event registered")

    def _response_fn(self, data):
        response = data.data
        rospy.loginfo("I heard: " + response)


if __name__ == '__main__':

    my_server = GuiServer(
        seconds_before_timeout=rospy.get_param('cordial/gui/seconds_before_timeout', 10),
    )
    rospy.sleep(2)

    rospy.spin()
