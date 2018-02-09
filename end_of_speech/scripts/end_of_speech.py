#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from mummer_asr.msg import MummerAsr
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import AudioBuffer
from std_msgs.msg import Empty

SILENCE_THRESHOLD = 4

class EndOfSpeech(object):
    def __init__(self):
        rospy.Subscriber("/mummer_asr/result", MummerAsr, self.callback, queue_size=1)
        rospy.Subscriber("/noise_filter_node/result", AudioBuffer, self.audio_received, queue_size=1)
        self.pub = rospy.Publisher("~eos", Empty, queue_size=1)
        self.output = None
        # self.pause = rospy.ServiceProxy('pause', Empty)
        # self.resume = rospy.ServiceProxy('resume', Empty)
        self.last_end_time = 0
        self.audio_stamp = 0

    def callback(self, msg):
        self.last_end_time = msg.header.stamp.secs

    def audio_received(self, msg):
        self.audio_stamp = msg.header.stamp.secs

        if (not self.audio_stamp - self.last_end_time <= SILENCE_THRESHOLD) and (not self.last_end_time == 0):
            self.pub.publish()
        else:
            pass


if __name__ == '__main__':
    rospy.init_node('end_of_speech')
    EOS = EndOfSpeech()
    rospy.spin()
