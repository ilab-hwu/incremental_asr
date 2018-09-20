#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sounddevice as sd
import rospy
import numpy as np
from naoqi_bridge_msgs.msg import AudioBuffer


class Sound(object):
    def __init__(self):
        self.pub = rospy.Publisher("~result", AudioBuffer, queue_size=10)
        self.buffer = []
        self.rate = rospy.get_param("~rate", 48000)
        self.chunk = int(self.rate/10)

    def callback(self, indata, frames, time, status):
        # print frames, time, status
        data = indata.flatten()
        self.buffer.extend(data.tolist())
        if len(self.buffer) >= self.chunk:
            a = AudioBuffer()
            a.header.stamp = rospy.Time.now()
            a.data = self.buffer[:self.chunk]
            a.frequency = self.rate
            a.channelMap = [0]
            self.pub.publish(a)
            self.buffer = self.buffer[self.chunk:]

    def run(self):
        with sd.InputStream(channels=1, samplerate=self.rate, dtype='int16', callback=self.callback):
           rospy.spin()

if __name__ == "__main__":
    rospy.init_node("microphone_grabber")
    Sound().run()

