import unittest
from unittest.mock import patch

from src.cordial_manager.src.aws_polly_client.aws_polly_client import AwsPollyClient
from mock import Mock

class TestAWSPollyClient(unittest.TestCase):
    def test_get_wav_file(self):
        mock_AwsPollyClient = Mock()
        mock_AwsPollyClient._synthesize_speech = Mock(return_value="myresponse")
        mock_AwsPollyClient._save_speech_as_wav_file = Mock(return_value="xyz/abc.wav")
        self.assertEqual(AwsPollyClient.get_wav_file(mock_AwsPollyClient,"hello","abc","xyz/"), "xyz/abc.wav")
        print(mock_AwsPollyClient.mock_calls)
        mock_AwsPollyClient._synthesize_speech.assert_called_with("hello")
        mock_AwsPollyClient._save_speech_as_wav_file.assert_called_with("myresponse", "abc", "xyz/")

    def test_get_schedule_for_behaviors(self):
        with patch.object(AwsPollyClient, '_split_by_actions') as mock_patch:
            mock_patch.returnvalue = []
            with self.assertRaises(SystemExit) as cm:
                AwsPollyClient.get_schedule_for_behaviors(mock_patch, "hello")
            self.assertEqual(cm.exception.code, -1)


if __name__ == '__main__':
    unittest.main()