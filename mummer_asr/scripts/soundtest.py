import sounddevice as sd
import rospy
import numpy as np
from naoqi_bridge_msgs.msg import AudioBuffer

RATE = 48000
CHUNK_SIZE = int(RATE/10)

class Sound(object):
    def __init__(self):
        self.pub0 = rospy.Publisher("~result0", AudioBuffer, queue_size=10)
        self.pub1 = rospy.Publisher("~result1", AudioBuffer, queue_size=10)
        self.fake_pub = rospy.Publisher("/noise_filter_node/result", AudioBuffer, queue_size=10)
        self.buffer0 = []
        self.buffer1 = []

    # def callback(self, indata, outdata, frames, time, status):
    def callback(self, indata, frames, time, status):
        #print frames, time, status
        data0 = indata.flatten()[::2]
        data1 = indata.flatten()[1::2]
        self.buffer0.extend(data0.tolist())
        self.buffer1.extend(data1.tolist())
        if len(self.buffer0) >= CHUNK_SIZE:
            a = AudioBuffer()
            a.header.stamp = rospy.Time.now()
            a.data = self.buffer0[:CHUNK_SIZE]
            a.frequency = RATE
            a.channelMap = [0]
            self.pub0.publish(a)
            self.fake_pub.publish(a)
            self.buffer0 = self.buffer0[CHUNK_SIZE:]
        if len(self.buffer1) >= CHUNK_SIZE:
            a = AudioBuffer()
            a.header.stamp = rospy.Time.now()
            a.data = self.buffer1[:CHUNK_SIZE]
            a.frequency = RATE
            a.channelMap = [0]
            self.pub1.publish(a)
            self.buffer1 = self.buffer1[CHUNK_SIZE:]

    def run(self):
        # with sd.RawStream(channels=2, dtype='int24', callback=self.callback):
        with sd.InputStream(channels=2, samplerate=RATE, dtype='int16', callback=self.callback):
        #with sd.Stream(channels=1, samplerate=48000, dtype='int16', callback=self.callback):
           rospy.spin()

if __name__ == "__main__":
    rospy.init_node("my_awesome_sound")
    Sound().run()

