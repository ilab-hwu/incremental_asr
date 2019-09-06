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
        self.__is_alive()

    def __is_alive(self):
        from threading import Thread
        from std_msgs.msg import String

        pub = rospy.Publisher("~is_alive", String, queue_size=1)

        def publish():
            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                pub.publish(str(rospy.Time.now().to_sec()))
                r.sleep()

        t = Thread(target=publish)
        t.start()

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
            phrases=["New Yorker","new yorker","Apteekki","pharmacy","apteekki","Instrumentarium","instrumentarium","Kicks","kicks","Tekniskamagasinet","tekniskamagasinet","Saaga","saaga","Luckiefun","luckiefun","Iittala Outlet","iittala outlet","SubWay","Sub Way","subway","sub way","Finnlandia","finnlandia","Spice Ice","spice ice","Cafe de Lisa","cafe de lisa","Kahvila Ilopilleri","kahvila ilopilleri","Netrauta","netrauta","Superdry","superdry","Halonen","halonen","Zones by Sarkanniemi","Zones","zones by sarkanniemi","zones","Zizzi","zizzi","Clas Ohlson","clas ohlson","Eurokangas","eurokangas","HairStore","Hair Store","hairstore","hair store","Bella Roma and Daddy's Diner","Daddy's Diner","Bella Roma","bella roma and daddy's diner","daddy's diner","bella roma","Bella Roma and Daddys Diner","Bella Roma and Daddy s Diner","Daddys Diner","Daddy s Diner","bella roma and daddys diner","bella roma and daddy s diner","daddys diner","daddy s diner","Lumo-puoti","Lumo puoti","lumo-puoti","lumo puoti","Kotipizza and Rolls Expert","Kotipizza Rolls Expert","Kotipizza","Rolls Expert","kotipizza and rolls expert","kotipizza rolls expert","kotipizza","rolls expert","Flying Tiger Copenhagen","Flying Tiger","flying tiger copenhagen","flying tiger","Masku","masku","Ideapark Sport Areena","Sport Areena","ideapark sport areena","sport areena","Top-Sport","Top Sport","Top- Sport","top-sport","top sport","top- sport","Ballot","ballot","Easy Fit Traingin","easy fit traingin","Ti-Ti Nallen Koti","Ti Ti Nallen Koti","Ti- Ti Nallen Koti","ti-ti nallen koti","ti ti nallen koti","ti- ti nallen koti","Aleksi 13","Aleksi","aleksi 13","aleksi","Partioaitta","partioaitta","Prisma","prisma","Jesper Junior","Jesper","jesper junior","jesper","Minimani","minimani","Oma Saastopankki","oma saastopankki","Kalastus Suomi","kalastus suomi","Kultajousi","kultajousi","CM Hiustalo","C M Hiustalo","cm hiustalo","c m hiustalo","LahiTapiola Service desk","LahiTapiola","Lahi Tapiola Service desk","Lahi Tapiola","lahitapiola service desk","lahitapiola","lahi tapiola service desk","lahi tapiola","Glitter","glitter","Pentik","pentik","Farkkujen Tehtaanmyymala","farkkujen tehtaanmyymala","Bjorn Borg","bjorn borg","Rax","rax","Tommy Hilfiger","tommy hilfiger","Pelaamo","pelaamo","Stahlberg Home Bakery Cafe","Stahlberg","Bakery Cafe","stahlberg home bakery cafe","stahlberg","bakery cafe","Change","change","Hennes Mauritz","H and M","H&M","H& M","hennes mauritz","h and m","h&m","h& m","Lindex","lindex","Avain- ja suutarityo Helsky","Avain  ja suutarityo Helsky","avain- ja suutarityo helsky","avain  ja suutarityo helsky","Reima","reima","Arnold's","arnold's","Arnolds","Arnold s","arnolds","arnold s","Sinelli","sinelli","atm","Vila","Villa","vila","villa","Hesburger","hesburger","Name It","name it","Logistiikkakeskus","logistiikkakeskus","Tanssiopisto","tanssiopisto","Click Shoe","click shoe","Dressmann XL","Dressmann X L","dressmann xl","dressmann x l","Information desk","information desk","Burger King","burger king","HoviKebab","Hovi Kebab","hovikebab","hovi kebab","Pizza Hut","pizza hut","Brother Clothing","brother clothing","Idea-kappeli","Idea kappeli","idea-kappeli","idea kappeli","PiiPoo","Pii Poo","piipoo","pii poo","Intersport","intersport","Putiikki_WOW","Putiikki WOW","Putiikki_ W O W","Putiikki W O W","putiikki_wow","putiikki wow","putiikki_ w o w","putiikki w o w","My Bag","my bag","Bik Bok","bik bok","Suomalainen Kirjakauppa","suomalainen kirjakauppa","Din Sko","din sko","Ten Art","ten art","Mummola Lahjapuoti","mummola lahjapuoti","pukimo","Espresso House","espresso house","atm","River and Co","river and co","Hairlekiini","hairlekiini","Esprit","esprit","Nissen","nissen","Kukkakauppa","kukkakauppa","Coyote Grill","coyote grill","Marco Polo","marco polo","Guess","guess","Carlings","Carling","carlings","carling","Power","power","Elisan","Elisa","elisan","elisa","Pancho Villa","pancho villa","Cafe Linkosuo","cafe linkosuo","Shoe Store","shoe store","Kid Zone","kid zone","Your Face and Rils","Rils","Your Face","your face and rils","rils","your face","emotion","Forex Bank","forex bank","Life","life","Sticky Wingers","sticky wingers","Linda Mode","linda mode","Marimekko Outlet","marimekko outlet","Body Shop","body shop","KappAhl","Kapp Ahl","kappahl","kapp ahl","DNA","D N A","dna","d n a","Lagoon Fish Foot Spa","lagoon fish foot spa","Saunalahti","saunalahti","Musti ja Mirri","musti ja mirri","Gant","gant","Juvesport","juvesport","vallila shop","Easyfit","easyfit","Timanttiset","timanttiset","Alko","alko","Cubus","cubus","Hemtex","hemtex","KOO Kenka","K O O Kenka","koo kenka","k o o kenka","Unikulma","unikulma","Hanko Sushi","hanko sushi","swamp","Faunatar","faunatar","Digiman","digiman","Stadium","stadium","Makelan Kauppapuutarna","Makelan","makelan kauppapuutarna","makelan","NP","N P","np","n p","Mango","mango","Aitoleipa","aitoleipa","Fonum","fonum","Budget Sport","Budget","budget sport","budget","Vero Moda","vero moda","only","Dressman","dressman","only","Kultaporssi","kultaporssi","Donna Rosa","donna rosa","Gina Tricot","gina tricot","Kauneus- ja jalkahoitola Manna","hoitola Manna","Kauneus  ja jalkahoitola Manna","kauneus- ja jalkahoitola manna","hoitola manna","kauneus  ja jalkahoitola manna"])]
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

