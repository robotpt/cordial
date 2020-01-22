#!/usr/bin/env python


import sys
import re
import os
import json
import soundfile

from boto3 import client
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing


VISEMES_TRANSLATION = {
    "p": "BILABIAL",
    "f": "LABIODENTAL",
    "T": "INTERDENTAL",
    "s": "DENTAL_ALVEOLAR",
    "t": "DENTAL_ALVEOLAR",
    "S": "POSTALVEOLAR",
    "r": "POSTALVEOLAR",
    "k": "VELAR_GLOTTAL",
    "i": "CLOSE_FRONT_VOWEL",
    "u": "CLOSE_BACK_VOWEL",
    "@": "MID_CENTRAL_VOWEL",
    "a": "OPEN_FRONT_VOWEL",
    "e": "OPEN_FRONT_VOWEL",
    "E": "OPEN_FRONT_VOWEL",
    "o": "OPEN_BACK_VOWEL",
    "O": "OPEN_BACK_VOWEL",
    "sil": "IDLE",
}


class AwsPollyClient:

    def __init__(
            self,
            voice="Ivy",
            region="us-west-1",
    ):
        self._voice = voice
        self._aws_client = client("polly", region_name=region)

    def run(self, text):

        file_path = self.get_wav_file(text)
        behaviors = self.get_schedule_for_behaviors(text)

        return file_path, behaviors

    def get_wav_file(
            self,
            text,
            out_file_name='temp',
            out_directory='',
    ):

        response = self._synthesize_speech(text)
        out_file = self._save_speech_as_wav_file(response, out_file_name, out_directory)

        return out_file

    @staticmethod
    def _save_speech_as_wav_file(response, out_file_name, out_directory):

        out_file_path_no_ext = os.path.abspath(os.path.join(out_directory, out_file_name))
        out_file_path_ogg = out_file_path_no_ext + ".ogg"
        out_file_path_wav = out_file_path_no_ext + ".wav"

        if "AudioStream" in response:
            with closing(response["AudioStream"]) as stream:
                try:
                    with open(out_file_path_ogg, "wb") as f:
                        f.write(stream.read())
                except IOError as error:
                    print(error)
        else:
            print("Could not stream audio")
            sys.exit(-1)

        contents, samplerate = soundfile.read(out_file_path_ogg)
        soundfile.write(out_file_path_wav, contents, samplerate)

        os.remove(out_file_path_ogg)

        return out_file_path_wav

    def _synthesize_speech(self, text):
        try:
            response = self._aws_client.synthesize_speech(
                Text=self._get_phrase(text),
                TextType='ssml',
                OutputFormat="ogg_vorbis",
                VoiceId=self._voice,
            )
        except (BotoCoreError, ClientError) as error:
            print(error)
            sys.exit(-1)
        return response

    def get_schedule_for_behaviors(self, text):
        """
        function used to generate viseme and expression time behavior_schedule.
        input - line of text to be processed with polly
        """

        tokens = map(
            lambda s: self._cond_split(s),
            self._split_by_actions(text)
        )
        words = []

        for t in tokens:
            words += filter(lambda s: len(s) > 0, t)

        actions = []
        i = 0
        for w in words:
            if re.match("\*.*\*", w):
                args = w.strip("*").split()
                name = args.pop(0)
                actions.append([i,name,args])
            else:
                i += 1

        # example server code for reference: https://docs.aws.amazon.com/polly/latest/dg/example-Python-server-code.html
        # use the interface to communicate with the Amazon Polly Client
        try:
            # enclose phrase in speak tags to use Polly's ssml, and let Polly know the language
            response = self._aws_client.synthesize_speech(
                Text=self._get_phrase(text),
                TextType='ssml',
                OutputFormat="json",
                VoiceId=self._voice,
                SpeechMarkTypes =["viseme", "word"],
            )
        except (BotoCoreError, ClientError) as error:
            print(error)
            sys.exit(-1)

        # get json of times to play visemes and words (called an exposure sheet or x-sheet in animation)
        if "AudioStream" in response:
            with closing(response["AudioStream"]) as stream:
                x_sheet = [
                    json.loads(text)
                    for text in stream.read().split('\n')
                    if text != ''
                ]
        else:
            print("Could not stream audio")
            sys.exit(-1)

        # find the times to play beginnings of words, so the actions can be spliced in.
        word_times = filter(lambda l: l["type"]=="word", x_sheet)
        # behavior_schedule will be the list of visemes and actions in order by time
        # behavior_schedule will collect also information about word timing
        behavior_schedule = []
        for w in word_times:
            behavior_schedule.append(
                {
                    "time": float(w["time"]) / 1000.,  # convert ms to seconds
                    "type": "word",
                    "start": float(w["start"]),
                    "value": str(w["value"])
                }
            )
        
        # assign the actions the correct time based on when they appear in the script
        for a in actions:
            if a[0] > len(word_times)-1:
                a[0] = x_sheet[-1]["time"] / 1000.  # convert ms to seconds
            else:
                a[0] = (word_times[a[0]]["time"]) / 1000.  # convert ms to seconds

        # behavior_schedule will contain also the information about the timing of the words
        for a in actions:
            args = a[2]
            behavior_schedule.append(
                {
                    "start": float(a[0])+.01,  # prevent visemes and actions from being at exactly the same time
                    "type": "action",
                    "args": args,
                    "id": a[1]
                }
            )

        # get the times of just the visemes, and convert the viseme keys to the set used by CoRDial.
        visemes = map(
            lambda l: [
                l["time"],
                VISEMES_TRANSLATION[l["value"]]
            ],
            filter(lambda l: l["type"] == "viseme", x_sheet)
        )
        for v in visemes:
            behavior_schedule.append(
                {
                    "start": float(v[0]) / 1000.,  # convert ms to seconds
                    "type": "viseme",
                    "id": v[1]
                }
            )

        return sorted(behavior_schedule, key=lambda index: index['start'])

    @staticmethod
    def _get_phrase(text):
        phrase = (
                '<speak><lang xml:lang="en-US">' +
                ''.join(
                    filter(
                        lambda s: "*" not in s, AwsPollyClient._split_by_actions(text)
                    )
                ) +
                '</lang></speak>'
        )
        return phrase

    @staticmethod
    def _split_by_actions(text):
        tokens = re.split("(\*[^\*\*]*\*)", text)
        return tokens

    @staticmethod
    def _cond_split(s):
        if len(s) >= 2 and s[-1] == "*" and s[0] == "*":
            return [s]
        else:
            return re.split("\s+", s)


if __name__ == '__main__':

    text_to_say = "Hi there, love"

    client = AwsPollyClient()
    wav_file, behaviors = client.run(text_to_say)

    print behaviors
    print wav_file
    os.system("aplay " + wav_file)
