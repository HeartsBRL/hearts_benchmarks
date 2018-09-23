import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose2D, Twist, Pose
#TODO UNCOMMENT: from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState, DevicesState, TabletState
import std_srvs.srv
from collections import Counter

class GenericController(object):
    def __init__(self):
        self.tts_pub = rospy.Publisher("/hearts/tts", String, queue_size=10)
        print('tts pub is up')#TODO remove
        
    def say(self, text):
        ''' Publish text to tts_pub where text is then spoken aloud by tiago'''
        rospy.loginfo("saying \"" + text + "\"")
        print("saying \"" + text + "\"")#TODO remove
        rospy.sleep(1)
        self.tts_pub.publish(text)
        rospy.sleep(5)
        
    ## things for navigation
