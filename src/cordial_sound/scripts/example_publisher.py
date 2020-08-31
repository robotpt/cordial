#!/usr/bin/env python


import rospy
import os
from std_msgs.msg import String


rospy.init_node('example_sound_publisher', anonymous=True)
sound_publisher = rospy.Publisher("cordial/sound/play/file_path/wav", String, queue_size=1)


if __name__ == '__main__':

    file_name = 'hi_there.wav'
    current_directory = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(current_directory, file_name)

    rospy.sleep(2)
    while not rospy.is_shutdown():

        rospy.loginfo("Publishing sound")
        sound_publisher.publish(file_path)
        rospy.sleep(2)
