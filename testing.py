import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose2D, Twist, Pose
#TODO UNCOMMENT: from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState, DevicesState, TabletState
import std_srvs.srv
from collections import Counter

class GenericController(object):
    def __init__(self):
        # init subscribers
        rospy.Subscriber("/hearts/navigation/status", String, self.location_result_callback)#tbm2
    
        # init publishers 
        self.tts_pub =           rospy.Publisher("/hearts/tts", String, queue_size=10) #tbm2
        self.pub_location_goal = rospy.Publisher('/hearts/navigation/goal/location', 
                                                                String, queue_size=10) #tbm2
    ######################## FROM TBM2 ########################################        
    def say(self, text):
        ''' Publish text to tts_pub where text is then spoken aloud by tiago'''
        rospy.loginfo("saying \"" + text + "\"")
        rospy.sleep(1)
        self.tts_pub.publish(text)
        rospy.sleep(5)
    ###########################################################################
    
    ## things for navigation
    
    ######################## FROM TBM2 ########################################
    def move_to(self, location, count):
        ''' 
        Publish location to move to to /hearts/navigation/goal/location and wait
        for update on /hearts/navigation/status 
        
        If move not succesful, retry count number of times, changing orientation by 
        1 radian on each retry.
        '''
        rospy.loginfo("moving to \"" + location + "\" (" + str(count) + ")")
        msg = String()
        msg.data = location
        self.pub_location_goal.publish(msg)
        
        self.location_result = "Active"
        while self.location_result == "Active":
            rospy.sleep(1)
        
        if self.location_result != "Success" and count > 0:
            t = Twist()
            t.angular.z = 1.0
            self.pub_twist.publish(t)
            rospy.sleep(1)
            self.move_to(location, count - 1) #recurse
        
        if self.location_result != "Success":
            rospy.loginfo("ERROR movement to \"" + location + "\" has failed")
            self.say("Sorry, I am unable to move to "+location)
            #TODO should an actual error be thrown here?
            return False
        else:
            return True
            
    def location_result_callback(self, data):
        '''
        Trigged by subscriber: /hearts/navigation/status 
        
        Contains data on succes/failure/active of ... 
        '''#TODO does succes mean movement completed?
        rospy.loginfo(data.data)
        self.location_result = data.data
    ###########################################################################
    
    ######################## FROM TBM3 ########################################
    def move_to_location(self, target_location):
		rospy.loginfo("Moving to a location")
		
		self.move_to_loc_pub.publish(target_location)

		self.wait_to_arrive(5)
        
		self.say("I have arrived at the "+target_location+" location")
		
    def wait_to_arrive(self, count): #when this function is called, must specify no of counts before it breaks out of infinite loop
		rospy.loginfo("Checking Navigation Status")
		sub = rospy.Subscriber("/hearts/navigation/status", String, self.navigation_callback)
		self.nav_status = "Active"

		while self.nav_status == "Active":
			rospy.sleep(1)

		sub.unregister()
		if self.nav_status == "Fail" and count > 0:
			t = Twist()
			t.angular.z = 1.0
			self.pub_twist.publish(t)
			rospy.sleep(1)
			self.wait_to_arrive(count - 1) # recurse
			return self.nav_status == "Success"
			
    def navigation_callback(self, msg):
		self.nav_status = msg.data
    ###########################################################################		
    def move_to(self, target_location, num_retries):
		rospy.loginfo("Moving to a location")
		
		self.move_to_loc_pub.publish(target_location)

		arrive_success = self.wait_to_arrive(num_retries)
        
        if arrive_success == False:
            rospy.loginfo("ERROR movement to \"" + location + "\" has failed")
            self.say("Sorry, I am unable to move to "+location)
            #TODO should an actual error be thrown here?
            return False
        else:
		    self.say("I have arrived at the "+target_location+" location")
		    return True
		
    def wait_to_arrive(self, num_retries): #when this function is called, must specify no of counts before it breaks out of infinite loop
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
