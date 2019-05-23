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
from threading import Thread

# Audio recording parameters
RATE = 48000
CHUNK = int(RATE / 10)  # 100ms


class AudioStream(object):
    def __init__(self, name, rate, chunk):
        rospy.loginfo("Starting {}.".format(name))
        rospy.Subscriber(rospy.get_param("~topic", "noise_filter_node/result"), AudioBuffer, self.callback, queue_size=1)
        rospy.Service('~pause', Empty, self.pause)
        rospy.Service('~resume', Empty, self.resume)
        rospy.Service('~stop', Empty, self.stop_asr)
        rospy.Service('~start', Empty, self.start_asr)
        # rospy.Subscriber("/naoqi_driver_node/head_touch", HeadTouch, self.resume, queue_size=1)

        self.publisher = rospy.Publisher("~result", MummerAsr, queue_size=1)

        rospy.loginfo("done")

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True
        self.running = True
        self.exit = False
        self.timer_thread = None

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

    def timer(self):
        rospy.sleep(30.)
        rospy.logwarn("1 minute")

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

            if response.speech_event_type == 1:
                rospy.logwarn("!Restarting ASR stream!")
                self._buff.put(None)

            if not response.results:
                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
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
                if not self.closed and self.running:
                    self.publisher.publish(msg)
                    num_chars_printed = len(transcript)

            else:
                print (transcript + overwrite_chars)
                print "confidence: ", str(result.alternatives[0].confidence)
                #print 'alternatives: {}'.format(result.alternatives)
                num_chars_printed = 0
                msg.final = transcript + overwrite_chars
                msg.confidence = result.alternatives[0].confidence
                if not self.closed and self.running:
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
        # self.timer_thread = Thread(target=self.timer)
        # self.timer_thread.start()
        return EmptyResponse()

    def start_asr(self, req):
        rospy.loginfo("ASR STARTED")
        self.running = True
        self._buff.put(None)
        return EmptyResponse()

    def stop_asr(self, req):
        rospy.loginfo("ASR STOPPED")
        self.running = False
        self._buff.put(None)
        return EmptyResponse()

    def signal_handler(self, signal, frame):
        print('You pressed Ctrl+C!')
        self.closed = True
        self._buff.put(None)
        self.exit = True


if __name__ == "__main__":
    rospy.init_node("mummer_asr")
    language_code = rospy.get_param("~target_language", 'en-UK')  # a BCP-47 language tag

    client = speech.SpeechClient()
    # TODO add speech context with keywords from vocabulary
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        max_alternatives=5,
        language_code=language_code,
        speech_contexts=[speech.types.SpeechContext(
            phrases=['123 Tuulilasi Ideapark', 'ABC asema', 'Aitoleipä', 'Aleksi 13', 'Alko', 'Apteekki Ideapark', 'Arnolds', 'Avain- ja suutarityö Helsky', 'Ballot', 'Beefking Steakhouse', 'Bella Roma', 'Bik Bok', 'Björn Borg', 'BR Lelut', 'Brothers', 'Budget Sport', 'BURGER KING', 'Cafe Delisa, Liisan Leipomo', 'Cafe Linkosuo Autoareena', 'Carlings', 'CHANGE Lingerie', 'Clas Ohlson', 'Click Shoes', 'Coyote Bar & Grill', 'Cubus', 'Daddys Diner', 'Digiman', 'DinSko', 'Dna Kauppa Oy', 'Dressmann', 'Dressmann XL', 'EasyFit Ideapark', 'Elisan Myyntipiste', 'Emotion', 'Esprit', 'Eurokangas', 'Farkkujen Tehtaanmyymälä', 'Faunatar', 'Fifi et Fido', 'Finnlandia Kalustemaailma', 'Flying Tiger Copenhagen', 'Fonum', 'Fortum', 'Gant', 'Gerry Weber', 'Gina Tricot', 'Glitter', 'Guess Outlet', 'H&M', 'Hairlekiini', 'Halonen', 'Hemtex', 'Hesburger', 'Hiustalo Outlet', 'Hoitola Manna', 'Hovikebab', 'Ideapesu', 'ideaPrint', 'Iittala outlet Lempäälä', 'Ilopilleri', 'Infopiste', 'Instrumentarium', 'Intersport Ideapark', 'It’s Pure Ideapark Lempäälä', 'Jack & Jones', 'Jesper Junior', 'Juvesport', 'Kalastus-Suomi', 'Kamux', 'KappAhl', 'Kauneus- ja jalkahoitopalvelu SK', 'Kauppahuone Riveri', 'Kicks', 'Kirjapörssi', 'KOOKENKÄ', 'Kotipizza', 'Kristyle', 'Kukka & puutarhakauppa Mäkelä TULOSSA', 'Kukka Kesäpiha', 'Kultajousi', 'Kultatukku', 'Kulttuurikeskus PiiPoo', 'Lagoon Fish Foot Spa', 'Life', 'LINDA MODE', 'Lindex', 'Linkosuo Cafe', 'LUMO-puoti', 'LähiTapiola', 'Mango', 'Marc O’Polo', 'Marimekko Outlet', 'Maskun Kalustetalo', 'Minimani', 'Mje-Fix Oy', 'Mobila', 'Mummola Lahjapuoti', 'Musti ja Mirri', 'MyBag', 'Name It', 'Netrauta.fi', 'New Hairstore', 'New Yorker', 'Nissen', 'Nordea Ideapark', 'NP', 'Oma Säästöpankki', 'Partioaitta', 'Pelaamo', 'Pentik', 'Pirkanmaan FysioCenter', 'Pirkanmaan Tanssiopisto', 'Power', 'Prisma', 'Prisma Ideapark parturi-kampaamo', 'Pukimo', 'Ravintola Pancho Villa', 'Rax Buffet', 'Reima', 'Rils', 'Rolls Express', 'Saunalahti Ideapark', 'Shoe House', 'Sievi Shop', 'Sinelli', 'Spice Ice', 'Stadium', 'Sticky Wingers', 'Ståhlberg Home Bakery & Cafe', 'Subway', 'SunWok', 'Suomalainen Kirjakauppa', 'Suomen Kultapörssi', 'Superdry', 'Swamp', 'Talenom', 'Teknikmagasinet', 'Telia Kauppa', 'Thai Papaya', 'The Body Shop', 'Ti-Ti Nallen Koti', 'Timanttiset', 'Tommy Hilfiger', 'Top Sport', 'Toys R US', 'Unikulma', 'Ur & Penn', 'Vaihtoplus', 'Vero Moda', 'Vila', 'Your Face', 'Zizzi', 'Zones by Särkänniemi'])],
        )
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True,
        single_utterance=True)

    with AudioStream(rospy.get_name(), RATE, CHUNK) as a:
        signal.signal(signal.SIGINT, a.signal_handler)
        startTime = rospy.Time.now()

        # Initial state TODO: make it into a ros launch parameter??
        a.closed = True

        while not a.exit:
            if not a.closed and a.running:
                rospy.loginfo("Listening")
                responses = client.streaming_recognize(streaming_config, a.generator())
                try:
                    a.listen_print_loop(responses)
                except Exception as e:
                    print e
                    continue
            else:
                time.sleep(0.01)

