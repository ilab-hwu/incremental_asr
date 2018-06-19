#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from end_of_speech.msg import EndOfSpeech
from std_srvs.srv import Empty, EmptyResponse


class EndOfSpeech_node(object):
    def __init__(self):
        self.pub = rospy.Publisher("~eos", EndOfSpeech, queue_size=1)
        rospy.Service("/mummer_asr/pause", Empty, self.dummy_cb)
        rospy.Service("/mummer_asr/resume", Empty, self.dummy_cb)
        print "Enter text below and press enter:"
        while not rospy.is_shutdown():
            s = raw_input()
            eos_msg = EndOfSpeech()
            eos_msg.final_utterance = s
            eos_msg.confidence = 1.0
            self.pub.publish(eos_msg)

    def dummy_cb(self, req):
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('end_of_speech')
    EOS = EndOfSpeech_node()
    rospy.spin()
