#!/usr/bin/env python


import rospy
from std_msgs.msg import String


rospy.init_node('example_manager_publisher', anonymous=True)
say_publisher = rospy.Publisher(rospy.get_param("_SAY_TOPIC"), String, queue_size=1)


if __name__ == '__main__':

    text_to_say = "Can you hear me?  Is my mouth moving?"

    rospy.sleep(5)
    while not rospy.is_shutdown():

        rospy.loginfo("Publishing message '{}'".format(text_to_say))
        say_publisher.publish(text_to_say)
        rospy.sleep(5)
