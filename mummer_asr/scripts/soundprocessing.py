#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import time
from future.moves import queue
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from naoqi_bridge_msgs.msg import AudioBuffer, HeadTouch
from mummer_asr.msg import MummerAsr
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import signal

# Audio recording parameters
RATE = 48000
CHUNK = int(RATE / 10)  # 100ms


class AudioStream(object):
    def __init__(self, name, rate, chunk):
        rospy.loginfo("Starting {}.".format(name))
        rospy.Subscriber("/noise_filter_node/result", AudioBuffer, self.callback, queue_size=1)
        rospy.Service('~pause', Empty, self.pause)
        rospy.Service('~resume', Empty, self.resume)
        # rospy.Subscriber("/naoqi_driver_node/head_touch", HeadTouch, self.resume, queue_size=1)

        self.publisher = rospy.Publisher("~result", MummerAsr, queue_size=1)

        rospy.loginfo("done")

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True
        self.exit = False

        self.rate = rate
        self.chunk = chunk

    def __enter__(self):
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)

    def callback(self, msg):
        c = np.array(msg.data, dtype=np.int16)
        if not self.closed:
            self._buff.put(c.tobytes())

    def generator(self):
        while not self.closed:
            if rospy.is_shutdown():
                return
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()

            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                if rospy.is_shutdown():
                    return
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield types.StreamingRecognizeRequest(audio_content=b''.join(data))

    def listen_print_loop(self, responses):
        """Iterates through server responses and prints them.

        The responses passed is a generator that will block until a response
        is provided by the server.

        Each response may contain multiple results, and each result may contain
        multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
        print only the transcription for the top alternative of the top result.

        In this case, responses are provided for interim results as well. If the
        response is an interim one, print a line feed at the end of it, to allow
        the next result to overwrite it, until the response is a final one. For the
        final one, print a newline to preserve the finalized transcription.
        """
        num_chars_printed = 0
        for response in responses:
            if rospy.is_shutdown():
                return
            if not response.results:
                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            # print "[EVENT] ", response.speech_event_type
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript

            # Display interim results, but with a carriage return at the end of the
            # line, so subsequent lines will overwrite them.
            #
            # If the previous result was longer than this one, we need to print
            # some extra spaces to overwrite the previous result
            overwrite_chars = ' ' * (num_chars_printed - len(transcript))

            msg = MummerAsr()
            msg.header.stamp = rospy.Time.now()

            if not result.is_final:
                sys.stdout.write(transcript + overwrite_chars + '\r')
                sys.stdout.flush()
                msg.incremental = transcript + overwrite_chars
                self.publisher.publish(msg)
                num_chars_printed = len(transcript)

            else:
                print (transcript + overwrite_chars)
                print "confidence: ", str(result.alternatives[0].confidence)
                num_chars_printed = 0
                msg.final = transcript + overwrite_chars
                msg.confidence = result.alternatives[0].confidence
                self.publisher.publish(msg)

                ##############################################
                # End-Of-Speech detection goes here.
                # For now it's just the recognized word "stop"
                ##############################################
                # if re.search(r'\b(exit|quit|stop)\b', transcript, re.I):
                #     print('End of speech detected..')
                #
                #     self.pause(msg)

    def pause(self, msg):
        # msg.end_of_speech = True
        # self.publisher.publish(msg)
        rospy.loginfo("ASR PAUSED")
        self.closed = True
        self._buff.put(None)
        return EmptyResponse()

    def resume(self, req):
        rospy.loginfo("ASR RESUMED")
        self.closed = False
        return EmptyResponse()

    def signal_handler(self, signal, frame):
        print('You pressed Ctrl+C!')
        self.closed = True
        self._buff.put(None)
        self.exit = True


if __name__ == "__main__":
    rospy.init_node("mummer_asr")
    language_code = 'en-US'  # a BCP-47 language tag

    client = speech.SpeechClient()
    # TODO add speech context with keywords from vocabulary
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
        speech_contexts=[speech.types.SpeechContext(
            phrases=['Ioannis Papaioannou'])],
        )
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True,
        single_utterance=False)

    with AudioStream(rospy.get_name(), RATE, CHUNK) as a:
        signal.signal(signal.SIGINT, a.signal_handler)
        startTime = rospy.Time.now()

        # Initial state TODO: make it into a ros launch parameter??
        a.closed = True

        while not a.exit:
            if not a.closed:
                rospy.loginfo("Listening")
                responses = client.streaming_recognize(streaming_config, a.generator())
                try:
                    a.listen_print_loop(responses)
                except Exception as e:
                    print e
                    continue

