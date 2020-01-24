#!/usr/bin/env python

import rospy
import tf
from cordial_face.msg import LookatRequest, FaceKeyframeRequest, FaceRequest
import threading
from geometry_msgs.msg import Point


class FaceServer:

    def __init__(self):

        rospy.init_node('face_client')
        self._lookat_sub = rospy.Subscriber('cordial/face/look_at', LookatRequest, self.lookat_cb)
        self._keyframe_sub = rospy.Subscriber('cordial/face/keyframes', FaceKeyframeRequest, self.keyframe_cb)
        self._face_pub = rospy.Publisher('cordial/face/play', FaceRequest, queue_size=1)

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
                    (trans, rot) = self._tf.lookupTransform('cordial/lookat_frame', target, rospy.Time(0))

                    # face deals in cm, tf in m
                    f = FaceRequest(
                        hold_gaze=FaceRequest.IDLE_OFF,
                        retarget_gaze=True,
                        gaze_target=Point(
                            x=trans[0]*100,
                            y=trans[1]*100,
                            z=trans[2]*100
                        )
                    )
                    self._face_pub.publish(f)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn(
                        "Can't transform from frame '{target}' to 'cordial/lookat_frame'".format(target=target)
                    )
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

    FaceServer()
    rospy.spin()
