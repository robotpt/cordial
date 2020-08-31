class BehaviorSchedule:

    class Type:

        def __init__(self):
            pass

        VISEMES = "viseme"
        WORDS = "word"
        ACTIONS = "action"

    def __init__(self, behavior_timings):
        self._behavior_timings = behavior_timings

    def get_visemes(self, min_duration_in_seconds=None):

        visemes_timings = self._filter_behaviors_by_type(
            self._behavior_timings,
            self.Type.VISEMES,
        )

        if min_duration_in_seconds is not None:
            visemes_timings = self._get_behaviors_with_longer_durations(
                visemes_timings,
                min_duration_in_seconds,
            )

        return visemes_timings

    def get_words(self):
        return self._filter_behaviors_by_type(
            self._behavior_timings,
            self.Type.WORDS,
        )

    def get_actions(self):
        return self._filter_behaviors_by_type(
            self._behavior_timings,
            self.Type.ACTIONS,
        )

    def get_last_start_time(self):
        return self._behavior_timings[-1]["start"]

    @staticmethod
    def _filter_behaviors_by_type(behavior_timings, desired_type):
        return filter(
            lambda ele: ele["type"] == desired_type,
            behavior_timings,
        )

    @staticmethod
    def _get_behaviors_with_longer_durations(behavior_timings, min_duration_in_seconds=0.05):
        out = []
        durations = BehaviorSchedule._get_duration(behavior_timings, min_duration_in_seconds)
        for i in range(len(behavior_timings)):
            if durations[i] >= min_duration_in_seconds:
                out.append(behavior_timings[i])
        return out

    @staticmethod
    def _get_duration(behavior_timings, last_element_duration=0.05):
        out = []
        for i in range(0, len(behavior_timings) - 1):
            out.append(
                behavior_timings[i + 1]["start"] - behavior_timings[i]["start"]
            )
        out.append(last_element_duration)
        return out


if __name__ == '__main__':

    behavior_timings_ = [
        {'start': 0.006, 'char_end': 2, 'char_start': 0, 'type': 'word', 'value': 'Hi'},
        {'start': 0.006, 'type': 'viseme', 'id': 'VELAR_GLOTTAL'},
        {'start': 0.104, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'},
        {'start': 0.467, 'type': 'viseme', 'id': 'IDLE'},
        {'start': 0.66, 'char_end': 9, 'char_start': 4, 'type': 'word', 'value': 'there'},
        {'start': 0.66, 'type': 'viseme', 'id': 'INTERDENTAL'},
        {'start': 0.67, 'args': [], 'type': 'action', 'id': 'nod'},
        {'start': 0.704, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'},
        {'start': 0.885, 'type': 'viseme', 'id': 'POSTALVEOLAR'},
        {'start': 1.057, 'type': 'viseme', 'id': 'IDLE'},
    ]

    bs = BehaviorSchedule(behavior_timings_)
    print(bs.get_visemes())
    print(bs.get_visemes(0.05))
    print(bs.get_words())
    print(bs.get_actions())
    print(bs.get_last_start_time())
