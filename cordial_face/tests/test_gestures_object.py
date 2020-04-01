#!/usr/bin/env python

import unittest
import mock

import sys
sys.path.append("../scripts")

from face_gesture_player import Gestures


class TestGestures(unittest.TestCase):

    @mock.patch('face_gesture_player.open')
    @mock.patch('face_gesture_player.json')
    def setUp(self, *args):
        self._valid_gestures = Gestures('not_a_path')
        self._valid_gestures._data = {
            "valid_gesture_1": {
                "action_units": ["2"],
                "keyframes": [
                    {
                        "pose": [0],
                        "time": 0
                    },
                    {
                        "pose": [0.48],
                        "time": 0.75
                    },
                    {
                        "pose": [0.48],
                        "time": 1.5
                    },
                    {
                        "pose": [0],
                        "time": 1.75
                    }
                ]
            },
            "valid_gesture_2": {
                "action_units": ["1", "2"],
                "keyframes": [
                    {
                        "pose": [0, 0],
                        "time": 0
                    },
                    {
                        "pose": [0.59, 0.48],
                        "time": 0.75
                    },
                    {
                        "pose": [0.59, 0.48],
                        "time": 1.5
                    },
                    {
                        "pose": [0, 0],
                        "time": 1.75
                    }
                ]
            },
        }
        self._invalid_gestures_1 = Gestures('not_a_path')
        self._invalid_gestures_1._data = {
            "valid_gesture": {
                "action_units": ["2"],
                "keyframes": [
                    {
                        "pose": [0],
                        "time": 0
                    },
                    {
                        "pose": [0.48],
                        "time": 0.75
                    },
                    {
                        "pose": [0.48],
                        "time": 1.5
                    },
                    {
                        "pose": [0],
                        "time": 1.75
                    }
                ]
            },
            "too_many_action_units": {
                "action_units": ["1", "2", "3"],
                "keyframes": [
                    {
                        "pose": [0, 0],
                        "time": 0
                    },
                    {
                        "pose": [0.59, 0.48],
                        "time": 0.75
                    },
                    {
                        "pose": [0.59, 0.48],
                        "time": 1.5
                    },
                    {
                        "pose": [0, 0],
                        "time": 1.75
                    }
                ]
            },
        }

        self._invalid_gestures_2 = Gestures('not_a_path')
        self._invalid_gestures_2._data = {
            "valid_gesture": {
                "action_units": ["2"],
                "keyframes": [
                    {
                        "pose": [0],
                        "time": 0
                    },
                    {
                        "pose": [0.48],
                        "time": 0.75
                    },
                    {
                        "pose": [0.48],
                        "time": 1.5
                    },
                    {
                        "pose": [0],
                        "time": 1.75
                    }
                ]
            },
            "mismatched_pose_number_on_second_action": {
                "action_units": ["1", "2"],
                "keyframes": [
                    {
                        "pose": [0, 0],
                        "time": 0
                    },
                    {
                        "pose": [0.59],
                        "time": 0.75
                    },
                    {
                        "pose": [0.59, 0.48],
                        "time": 1.5
                    },
                    {
                        "pose": [0, 0],
                        "time": 1.75
                    }
                ]
            },
        }
        self._invalid_gestures_3 = Gestures('not_a_path')
        self._invalid_gestures_3._data = {
            "valid_gesture": {
                "action_units": ["2"],
                "keyframes": [
                    {
                        "pose": [0],
                        "time": 0
                    },
                    {
                        "pose": [0.48],
                        "time": 0.75
                    },
                    {
                        "pose": [0.48],
                        "time": 1.5
                    },
                    {
                        "pose": [0],
                        "time": 1.75
                    }
                ]
            },
            "no_time_field": {
                "action_units": ["1", "2", "3"],
                "keyframes": [
                    {
                        "pose": [0, 0],
                        "time": 0
                    },
                    {
                        "pose": [0.59, 0.48],
                        "time": 0.75
                    },
                    {
                        "pose": [0.59, 0.48],
                        "time": 1.5
                    },
                    {
                        "pose": [0, 0],
                        "time": 1.75
                    }
                ]
            },
        }

    def test_error_check(self):

        assert self._valid_gestures._error_check_data()

        self.assertRaises(
            AssertionError,
            self._invalid_gestures_1._error_check_data
        )
        self.assertRaises(
            AssertionError,
            self._invalid_gestures_2._error_check_data
        )
        self.assertRaises(
            AssertionError,
            self._invalid_gestures_3._error_check_data
        )

    def test_getters(self):

        assert self._valid_gestures.get_expressions() == ('valid_gesture_1', 'valid_gesture_2')

        expression_1 = "valid_gesture_1"
        expression_2 = "valid_gesture_2"

        assert self._valid_gestures.is_expression(expression_1)
        assert not self._valid_gestures.is_expression("not an expression")

        assert self._valid_gestures.get_num_keyframes(expression_1) == 4

        assert self._valid_gestures.get_timings(expression_1, is_convert_to_ms=False) == (0, 0.75, 1.5, 1.75)
        assert self._valid_gestures.get_timings(expression_1, is_convert_to_ms=True) == (0, 750, 1500, 1750)

        assert self._valid_gestures.get_durations(expression_1, is_convert_to_ms=False) == (0, 0.75, 0.75, 0.25)
        assert self._valid_gestures.get_durations(expression_1, is_convert_to_ms=True) == (0, 750, 750, 250)

        assert self._valid_gestures.get_poses(expression_1) == ((0,), (0.48,), (0.48,), (0,))
        assert self._valid_gestures.get_poses(expression_2) == ((0, 0), (0.59, 0.48), (0.59, 0.48), (0, 0))

        assert self._valid_gestures.get_action_units(expression_1) == ("2",)
        assert self._valid_gestures.get_action_units(expression_2) == ("1", "2")
