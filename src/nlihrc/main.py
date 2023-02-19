"""Main file"""
from pathlib import Path
import time
import rospy

from nlihrc.udpclient import UDPReceiver
from nlihrc.speech import SpeechRecognizer
from nlihrc.text import TextClassifier
from nlihrc.robot import CommandGenerator
from nlihrc.misc import Command
from std_msgs.msg import String


def main_speech(config):
    """Speech Recognition Server"""
    rospy.init_node("nlihrc_speech", anonymous=True, log_level=rospy.INFO)
    # Get config
    rate = config['speech']['rate']
    chunk = config['speech']['chunk']
    port = config['network']['port']
    ip = config['network']['ip']
    modelpath = config['speech']['modelpath']
    # UDP Receiver (Handles Android App comm.)
    udp = UDPReceiver(chunk, ip, port)
    # Speech Recognizer (Handles speech to text)
    rec = SpeechRecognizer(modelpath, rate)
    
    # Start udp thread
    udp.start()
    # Main program loop
    rospy.loginfo(f"Speech server online. Listening at {udp.host_ip = }, {port = }")
    try:
        while not rospy.is_shutdown():
            # Get audio data
            if udp.q.empty():
                continue
            data = udp.q.get()
            # Speech to text conversion
            words, number = rec.speech_to_text(data)
            if len(words) > 0:
                rospy.loginfo(f'Recognized words: {" ".join(words)}')
                if number is not None:
                    rospy.loginfo(f'Recognized number: {number}')
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down speech server")
    finally:
        rospy.loginfo("Closing UDP thread")
        udp.close_thread = True
        udp.join()
        while not udp.q.empty():
            udp.q.get()

class TextSub:
    def __init__(self) -> None:
        self.topic = "command"
        self.sub = rospy.Subscriber(self.topic, String, self.callback)
        self.text = None

    def callback(self, msg):
        """Ros subscriber callback"""
        if msg.data != "":
            self.text = msg.data

    def clear(self):
        self.text = None


def main_text(config):
    rospy.init_node("nlihrc_text", anonymous=True, log_level=rospy.INFO)
    ros_sub = TextSub()
    textclassifier = TextClassifier()
    rospy.loginfo(f"Text server online. Listening for String msg at /{ros_sub.topic}")
    while not rospy.is_shutdown():
        if ros_sub.text is not None:
            t1 = time.time()
            cmd = textclassifier.find_match(ros_sub.text)
            rospy.loginfo(f"{ros_sub.text = } and classified {cmd = }")
            rospy.loginfo(f"Time taken: {time.time() - t1:}")
            ros_sub.clear()


class RobotSub:
    def __init__(self) -> None:
        self.topic = "command"
        self.sub = rospy.Subscriber(self.topic, String, self.callback)
        self.cmd = None
        self.number = None

    def callback(self, msg):
        """Ros subscriber callback"""
        data_items = msg.data.split(',')
        number = None
        cmd_index = None
        if len(data_items) not in [1, 2]:
            return
        if len(data_items) == 1:
            cmd_index = data_items[0]
        elif len(data_items) == 2:
            cmd_index, number = data_items
            if data_items[-1] != '':
                number = int(number)
        cmd = Command(int(cmd_index))

        self.cmd = cmd
        self.number = number
    
    def clear(self):
        self.cmd = None
        self.number = None    

def main_robot(config):
    rospy.init_node("nlihrc_robot", anonymous=True, log_level=rospy.INFO)
    cmdgen = CommandGenerator(config)
    ros_sub = RobotSub()
    rospy.loginfo(f"Robot server online. Listening for String msg of format 'cmd,number' at /{ros_sub.topic}")
    while not rospy.is_shutdown():
        if ros_sub.cmd is not None:
            cmdgen.run(ros_sub.cmd, ros_sub.number)
            ros_sub.clear()


def main_app(config):
    """Main app that combines all modules"""
    rospy.init_node("nlihrc", anonymous=True, log_level=rospy.INFO)
    # Get config
    rate = config['speech']['rate']
    chunk = config['speech']['chunk']
    port = config['network']['port']
    ip = config['network']['ip']
    modelpath = config['speech']['modelpath']
    # UDP Receiver (Handles Android App comm.)
    udp = UDPReceiver(chunk, ip, port)
    # Speech Recognizer (Handles speech to text)
    rec = SpeechRecognizer(modelpath, rate)
    # Text classififiers (Handles text to command)
    textclassifier = TextClassifier()
    # Command generator (Handles robot manipulation based on commands)
    cmdgen = CommandGenerator(config)
    # Start udp thread
    udp.start()
    # Main program loop
    rospy.loginfo(f"Speech server online. Listening at {udp.host_ip = }, {port = }")
    try:
        while not rospy.is_shutdown():
            # Get audio data
            if udp.q.empty():
                continue
            data = udp.q.get()
            # Speech to text conversion
            words, number = rec.speech_to_text(data)
            if len(words) > 0:
                rospy.loginfo(f'Recognized words: {" ".join(words)}')
                if number is not None:
                    rospy.loginfo(f'Recognized number: {number}')
            else:
                continue
            # Text to command classification
            sentence = ' '.join(words)
            cmd = textclassifier.find_match(sentence, 0.95)
            if cmd is None:
                rospy.logwarn(f"Couldn't classify given {sentence = } to any command")
                continue
            # Command to robot
            cmdgen.run(cmd, number)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down app server")
    finally:
        rospy.loginfo("Closing UDP thread")
        udp.close_thread = True
        udp.join()
        while not udp.q.empty():
            udp.q.get()
