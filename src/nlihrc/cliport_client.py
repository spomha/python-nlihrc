"""Client side for CLIPORT"""
import json
import rospy
from std_msgs.msg import String

class CliportClient:

    def __init__(self) -> None:
        
        self.pub = rospy.Publisher("/cliport/in", String, queue_size=3)
        self.sub = rospy.Subscriber("/cliport/out", String, self.sub_callback)

        self.data = None


    def sub_callback(self, msg):
        self.data = json.loads(msg.data)
        print(self.data)
    
    def publish(self, sentence):
        """Send language input to server"""
        self.pub.publish(sentence)
