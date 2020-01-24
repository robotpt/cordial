#!/usr/bin/python


import rospy
import tf
from cordial_face.msg import *
import threading
from geometry_msgs.msg import Point
import sys


class FaceClient:

    def __init__(self):

        self._robot_name = "cordial"

        base_topic = ""
        self._base_topic=base_topic

        self._face_pub = rospy.Publisher(base_topic+'face', FaceRequest, queue_size=1)
        self._lookat_sub = rospy.Subscriber(base_topic+'lookat', LookatRequest, self.lookat_cb)
        self._keyframe_sub = rospy.Subscriber(base_topic+'face_keyframes', FaceKeyframeRequest, self.keyframe_cb)

        self._shared = {
            "target": "",
            "tracking": False,
            "stop_keyframes": False
        }
        self._tf = tf.TransformListener()
        self._track_thread = threading.Thread(target=self.tracking_thread)
        self._track_thread.start()
        rospy.loginfo("Face ROS server started.")

    def tracking_thread(self):
        tf_rate = 5  # hz
        r = rospy.Rate(tf_rate)

        while not rospy.is_shutdown():
            if self._shared["tracking"]:
                target = self._shared["target"]
                time = rospy.Time.now()
                try:
                    (trans, rot) = self._tf.lookupTransform("/CoRDial/"+self._robot_name+'/lookat_frame',target, rospy.Time(0))
                    f = FaceRequest(hold_gaze=FaceRequest.IDLE_OFF, retarget_gaze=True, gaze_target=Point(x=trans[0]*100,y=trans[1]*100,z=trans[2]*100)) #face deals in cm, tf in m
                    self._face_pub.publish(f)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Lookat server can't transform from frame " + target + " to "+ self._robot_name+'/lookat_frame')
            r.sleep()

    def lookat_cb(self, goal):
        if goal.follow_frame:
            self._shared["target"] = goal.frameid
            self._shared["tracking"] = True
        else:
            self._shared["tracking"] = False
            self._shared["target"] = ""

    #TODO: make playing face keyframes interruptable (action?)
    def keyframe_cb(self, goal):
        aus = map(lambda s: s[2:], goal.face_dofs)
        elapsed = 0
        for i in range(len(goal.frames)):
            poses = goal.frames[i].positions
            time = int(goal.times[i]*1000)-elapsed
            req = FaceRequest(aus=aus, au_degrees=poses, au_ms=time)
            self._face_pub.publish(req)
            rospy.sleep(time/1000.0)
            elapsed += time


if __name__ == '__main__':

    rospy.init_node('face_client')
    l = FaceClient()

    rospy.spin()
