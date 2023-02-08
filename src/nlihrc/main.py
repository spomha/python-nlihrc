"""Main file"""
from pathlib import Path
import rospy

from nlihrc.udpclient import UDPReceiver
from nlihrc.speech import SpeechRecognizer
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

def _robot_sub(msg, runner):
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
    runner.run(cmd, number)

def main_robot(config):
    rospy.init_node("nlihrc_robot", anonymous=True, log_level=rospy.INFO)
    cmdgen = CommandGenerator(config)
    topic = "command"
    rospy.Subscriber(topic, String, _robot_sub, cmdgen)
    rospy.loginfo(f"Robot server online. Listening for String msg of format 'cmd,number' at /{topic}")
    rospy.spin()
