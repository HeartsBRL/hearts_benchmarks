import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose2D, Twist, Pose
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState, DevicesState, TabletState
import std_srvs.srv
from collections import Counter

class GenericController(object):
    def __init__(self):
        # init subscribers
        rospy.Subscriber("/hearts/navigation/status", String, self.navigation_callback)#tbm2

        # init publishers
        self.tts_pub =           rospy.Publisher("/hearts/tts", String, queue_size=10) #tbm2
        self.pub_location_goal = rospy.Publisher('/hearts/navigation/goal/location',
                                                                String, queue_size=10) #tbm2
        self.pub_motion =        rospy.Publisher("motion_name", String, queue_size=10)

        #init variables
        self.speech = None

    ######################## SPEECH PRODUCTION FROM TBM2 ########################################
    def say(self, text):
        ''' Publish text to tts_pub where text is then spoken aloud by tiago'''
        rospy.loginfo("saying \"" + text + "\"")
        rospy.sleep(1)
        self.tts_pub.publish(text)
        rospy.sleep(5)

    ########################### LISTEN TO SPEECH ################################################
    def toggle_stt(self,status):
        '''
        Turns stt on or off.
        how to use: self.toggle_stt('on') to turn on stt, self.toggle_stt('off') to turn off stt'''

        if status == 'on' :
            self.speech = None
            self.sub_cmd=rospy.Subscriber("/hearts/stt", String, self.stt_callback)
            print('***** Listening for speech, converting speech to text')
        else:
            self.sub_cmd.unregister()
            print('***** NOT! Listening for speech')

        return

    ### extract content of speech recognition
    def stt_callback(self,data): #from tbm3
        '''
        to use in code use self.speech
        e.g. answer = self.speech
        '''
        speech = str(data)
        speech = speech.lower()

        # remove the ROS msg "Data:" value from speech
        item = 'data:'
        if item in speech:
            speech = speech.replace(item
            rospy.loginfo('*** Heard speech:\n'+speech+'\n')

        # check that text has been returned
        if "bad_recognition" in speech:
            self.say("Sorry, no words were recognised.")

        self.speech = speech


    ################## NAVIGATION FUNCTIONS ###################################

    def move_to_location(self, target_location, num_retries):
        '''
        Publish location to move to to /hearts/navigation/goal/location and wait
        for update on /hearts/navigation/status (either "Success" or "Failure")

        If move not succesful, retry given number of times, changing orientation
        by 1 radian on each retry.
        '''
        rospy.loginfo("moving to \"" + target_location + "\" (" + str(count) + ")")
        msg = String()
        msg.data = location
        self.pub_location_goal.publish(msg)

        arrive_success = self.wait_to_arrive(num_retries)

        if arrive_success == False:
            rospy.loginfo("ERROR movement to \"" + target_location + "\" has failed")
            self.say("Sorry, I am unable to move to "+target_location)
            #TODO should an actual error be thrown here?
            return False
        else:
            self.say("I have arrived at the "+target_location+" location")
            return True

    def wait_to_arrive(self, num_retries):
        rospy.loginfo("Checking Navigation Status")
        sub = rospy.Subscriber("/hearts/navigation/status", String, self.navigation_callback)
        self.nav_status = "Active"

        while self.nav_status == "Active":
            rospy.sleep(1)

        sub.unregister()
        if self.nav_status == "Fail" and num_retries > 0:
            t = Twist()
            t.angular.z = 1.0
            self.pub_twist.publish(t)
            rospy.sleep(1)
            self.wait_to_arrive(num_retries - 1) # recurse

        return self.nav_status == "Success"

    def navigation_callback(self, msg):
        self.nav_status = msg.data

    ###########################################################################

    def move_to_pose(self, pose_name):
        rospy.loginfo("moving to pose " + pose_name )
        msg = String()
        msg.data = pose_name
        self.pub_motion.publish(msg)

        #TODO some sort of making sure you actually get there before moving on
