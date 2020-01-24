#!/usr/bin/env python


import rospy

from std_msgs.msg import UInt32, String
from cordial_gui.msg import Display


class GuiClient:

    def __init__(
            self,
            seconds_before_timeout,
    ):

        rospy.init_node("gui_client")
        rospy.Subscriber("cordial/gui/idle_time", UInt32, self.timeout_fn)
        rospy.Subscriber("cordial/gui/user_response", String, self.response_fn)
        self._display_publisher = rospy.Publisher("cordial/gui/display", Display, queue_size=1)

        self._seconds_before_timeout = seconds_before_timeout

    def timeout_fn(self, data):

        seconds_elapsed = data.data
        if seconds_elapsed >= self._seconds_before_timeout:
            content = "timeout called - idle for {elapsed} seconds".format(elapsed=seconds_elapsed)
        else:
            content = "user interacting"

        rospy.loginfo(content)
        display_msg = Display()
        display_msg.content = content
        self._display_publisher.publish(display_msg)

    def response_fn(self, data):
        response = data.data
        rospy.loginfo("I heard: " + response)


if __name__ == '__main__':

    GuiClient(
        seconds_before_timeout=rospy.get_param('cordial/gui/seconds_before_timeout', 10),
    )
    rospy.spin()
