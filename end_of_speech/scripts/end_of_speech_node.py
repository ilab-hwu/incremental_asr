#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from mummer_asr.msg import MummerAsr
from naoqi_bridge_msgs.msg import AudioBuffer
from end_of_speech.msg import EndOfSpeech

SILENCE_THRESHOLD = 2

class EndOfSpeech_node(object):
    def __init__(self):
        rospy.Subscriber("/mummer_asr/result", MummerAsr, self.callback, queue_size=1)
        self.sub = rospy.Subscriber("/noise_filter_node/result", AudioBuffer, self.audio_received, queue_size=1)
        self.pub = rospy.Publisher("~eos", EndOfSpeech, queue_size=1)
        self.output = None
        self.last_end_time = 0
        self.audio_stamp = 0
        self._google_utt_buff = []
        self._google_conf_buff = []

    def callback(self, msg):
        if self.sub is None:
            self.sub = rospy.Subscriber("/noise_filter_node/result", AudioBuffer, self.audio_received, queue_size=1)
        self.last_end_time = msg.header.stamp.secs
        if msg.final != '':
            self._google_utt_buff.append(msg.final)
        rospy.loginfo(msg.confidence)
        if msg.confidence > 0.0:
            self._google_conf_buff.append(msg.confidence)

    def audio_received(self, msg):
        eos_msg = EndOfSpeech()
        self.audio_stamp = msg.header.stamp.secs

        if (not self.audio_stamp - self.last_end_time <= SILENCE_THRESHOLD) and (not self.last_end_time == 0):
            eos_msg.final_utterance, eos_msg.confidence = self.create_final_utterance()
            print "FINAL", eos_msg.final_utterance
            if eos_msg.confidence > 0.:
                rospy.loginfo("Final utterance is '%s' with '%f' confidence" % (eos_msg.final_utterance, eos_msg.confidence))

            self.pub.publish(eos_msg)
            self.sub.unregister()
            self.sub = None
        else:
            pass

    def create_final_utterance(self):
        avg_conf = sum(self._google_conf_buff) / len(self._google_conf_buff) if \
            (len(self._google_conf_buff) > 0) else 0
        utterance = ' '.join(self._google_utt_buff)
        self._google_utt_buff = []
        self._google_conf_buff = []
        return utterance.strip(), avg_conf


if __name__ == '__main__':
    rospy.init_node('end_of_speech')
    EOS = EndOfSpeech_node()
    rospy.spin()
