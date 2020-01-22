#!/usr/bin/env python


import rospy
import time

from std_msgs.msg import String
from aws_polly_client import AwsPollyClient

rospy.init_node('manager')
wav_file_publisher = rospy.Publisher('/sound/from_file/wav', String, queue_size=1)
behavior_publisher = rospy.Publisher('/cordial/behaviors', String, queue_size=1)

rospy.loginfo(rospy.get_param('voice'))

aws_client = AwsPollyClient(
    voice=rospy.get_param('voice', 'Ivy'),
    region=rospy.get_param('region', 'us-west-1'),
)

time.sleep(2)

text_to_say = "Hello*wave*, I'm a robot*nod*."
text, file_path, behavior_schedule = aws_client.run(text_to_say)


wav_file_publisher.publish(file_path)
behavior_publisher.publish(str(behavior_schedule))
