#!/usr/bin/env python

import rospy
import rospkg

import os
import json
import threading

from std_msgs.msg import String
from cordial_msgs.msg import FaceRequest


class FaceGesturePlayer:

    def __init__(self, expression_file_path):

        rospy.init_node('face_expression_publisher')
        self._face_publisher = rospy.Publisher(rospy.get_param("PLAY_FACE_TOPIC"), FaceRequest, queue_size=1)
        rospy.Subscriber(rospy.get_param("PLAY_GESTURE_TOPIC"), String, self._play_expression_callback, queue_size=1)

        self._gestures = Gestures(expression_file_path)
        self._timers = []

    def _play_expression_callback(self, data):
        expression = data.data
        if self._gestures.is_expression(expression):
            self.play_expression(expression)
            rospy.loginfo("Playing face gesture '{}'".format(expression))

    def play_expression(self, expression):

        def get_expression_callback_fn(aus, au_degrees, au_ms):
            def callback():
                face_request = FaceRequest(
                    aus=aus,
                    au_degrees=au_degrees,
                    au_ms=au_ms,
                )
                self._face_publisher.publish(face_request)
            return callback

        action_units = self._gestures.get_action_units(expression)
        poses = self._gestures.get_poses(expression)
        timings = self._gestures.get_timings(expression, is_convert_to_ms=False)
        durations = self._gestures.get_durations(expression, is_convert_to_ms=True)

        is_truncating_expression = False
        while self._timers:
            t = self._timers.pop()
            if t.is_alive():
                is_truncating_expression = True
                t.cancel()
        if is_truncating_expression:
            rospy.loginfo("Expression cut short to start next expression")

        for i in range(self._gestures.get_num_keyframes(expression)):
            t = threading.Timer(
                timings[i],
                get_expression_callback_fn(action_units, au_degrees=poses[i], au_ms=durations[i]),
                )
            t.start()
            self._timers.append(t)

    def get_expressions(self):
        return self._gestures.get_expressions()


class Gestures:

    def __init__(self, file_path):
        with open(file_path) as json_file:
            self._data = json.load(json_file)
        self._error_check_data()

    def _error_check_data(self):
        for expression in self.get_expressions():
            timings = self.get_timings(expression)
            action_units = self.get_action_units(expression)
            poses = self.get_poses(expression)
            assert len(timings) == len(poses)
            for p in poses:
                assert len(p) == len(action_units)
        return True

    def get_expressions(self):
        return tuple(self._data.keys())

    def _get_expression(self, expression):

        if not self.is_expression(expression):
            raise ValueError("'{}' is not a valid expression".format(expression))

        return self._data[expression]

    def is_expression(self, string):
        return string in self.get_expressions()

    def get_timings(self, expression, is_convert_to_ms=True):
        timings = [k['time'] for k in self._get_expression(expression)['keyframes']]
        if is_convert_to_ms:
            timings = [int(t*1000) for t in timings]
        return tuple(timings)

    def get_durations(self, expression, is_convert_to_ms=True):
        timings = self.get_timings(expression, is_convert_to_ms=is_convert_to_ms)
        durations = [timings[0]] + [timings[i+1] - timings[i] for i in range(len(timings)-1)]
        return tuple(durations)

    def get_poses(self, expression):
        return tuple([tuple(k['pose']) for k in self._get_expression(expression)['keyframes']])

    def get_num_keyframes(self, expression):
        return len(self.get_timings(expression))

    def get_action_units(self, expression):
        return tuple(self._get_expression(expression)["action_units"])


def demo_expressions(expressions_file_path, play_expression=None):

    p = FaceGesturePlayer(expressions_file_path)
    if play_expression is not None:
        p.play_expression(play_expression)
    else:
        for exp in p.get_expressions():
            print(exp)
            p.play_expression(exp)
            rospy.sleep(5)


if __name__ == '__main__':

    expressions_file_path_ = os.path.join(
        rospkg.RosPack().get_path('cordial_face'),
        'resources',
        'expressions.json',
    )
    
    #demo_expressions(expressions_file_path_)

    FaceGesturePlayer(expressions_file_path_)
    rospy.spin()

