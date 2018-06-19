#!/usr/bin/env python
'''
Required modules:
	Mapping
	text to speech
	speech to text

General Structure:
if case w4- follow person
Go to location i
if/when fail happens
	handle based on case
		case w1- update map and go other direction
		case w2- ID obstacle and ask for it to move (or wait for it to move)
		case w5- ID door handle and then push open door
when success happens
Say success

Todo: basic update to current program structure
Todo: Automatically update the map with closed door
	Dynamic map switching
Todo: follow human using route planning

Todo: At competition record map with places labeled 1-6 w/o 4 (3=5)

'''
########################### Includes ####################################################

import rospy
from rospy.rostime import Duration
import time
from math import cos, sin

#from navigation_camera_mgr_example import NavigationCameraMgr
import std_srvs.srv
from pal_navigation_msgs.srv import Acknowledgment

from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
from turtlesim.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from move_base_msgs.msg import MoveBaseActionFeedback
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState

############################# Controller Class ############################################
class Controller():
    def __init__(self):
        # Movement Parameters
        self.head_lr = 0.0
        self.head_ud = -0.5
        self.head_move_step_size = .15
        self.turn_step_size = 0.2
        self.move_step_size = 0.2

        self.handle_camera_direction(['center'])
        self.outfolder = rospy.get_param('output_path')
        self.tbm1_commands_dict = {
            "move": ["forward", "backward"],
            "turn": ["right", "left"],
            "go": self.rooms,
            "look": ["up", "down", "right", "left"],
            "here": [],
            "see": self.objects,}

        self.map_both_open = "map_1"
        self.map_A_open = "map_2"
        self.map_B_open = "map_3"

        self.waypoint = 1
        self.repeat = 0

        ### Publishers - sends out goal locations, movement/turns, and speech
        self.pubGoal = rospy.Publisher('hearts/navigation/goal/location', String, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)
        self.pub_pic = rospy.Publisher('/hearts/camera/snapshot', String, queue_size = 10)
        self.pub_head = rospy.Publisher('/head_controller/command',JointTrajectory, queue_size = 10)
        self.pub_move = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)  
        self.pub_dummy = rospy.Publisher('/move_base/feedback', MoveBaseActionFeedback, queue_size = 10)
              
        ### Subscribers - must listen for speech commands, location
        #rospy.Subscriber("hearts/navigation/goal/location", String, self.locGoal_callback)
        #rospy.Subscriber("/turtle1/pose", Pose, self.currentPose_callback)
        #rospy.Subscriber("hearts/navigation/pose/location", String, self.current_pose)
        #rospy.Subscriber("roah_rsbb/benchmark", Benchmark, self.benchmark_callback)
        rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)
        rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.current_pose_callback)
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
        rospy.Subscriber('move_base/status', GoalStatusArray, self.nav_status_callback)
        #TODO subscribe to succeed or fail
        self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        self.execute = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty)
        self.change_map = rospy.ServiceProxy('/pal_map_manager/change_map', Acknowledgment)
        #ack = change_map("map_1")


        # Disable head manager
        head_mgr = NavigationCameraMgr()
        head_mgr.head_mgr_as("disable")
        
    def begin(self):
        for self.waypoint in range(1,7):
            if self.waypoint == 4:
                self.ID_person()
                self.follow_person()
            else:
                self.go_to_target(self.waypoint)
            self.continue_on = False #If true, will continue to next waypoint
            while not self.continue_on: #loop to wait for nav to complete and handle errors
                while self.waiting_for_return==True:
                    rospy.sleep(1)
                if self.reached_goal==False:
                    self.handle_fail(self.waypoint)
                else:
                    self.continue_on = True
            self.pub_talk.publish("Waypoint completed, moving on")

    def handle_fail(self):
        if self.wapoint == 1:
            # Todo Update map
            self.go_to_target(1)
            

    def go_to_target(self,w):
        self.pubGoal.publish(str(w))
        self.reached_goal==False
        return

    def ID_person(self):
        #get from zeke or call zeke's functions
        return
    def follow_person(self):
        #get from zeke or call zeke's functions
        return

    def nav_status_callback(self,data):
        length_status = len(data.status_list)
        if length_status > 0:
            status = data.status_list[length_status-1].status
            if status == 4 or status == 5 or status == 9: #Failed to reach goal
                self.reached_goal
        
        if self.waypoint == 1:
            if status == 4 or status == 5 or status == 9: #Failed to reach goal
                if self.repeat<1:
                    #switch maps 
                    rospy.loginfo('Obstacle encountered, switching maps')
                    ack = change_map(self.map_B_open)
                    rospy.loginfo(ack)
                    rospy.loginfo('retrying destination')
                    self.pubGoal.publish(str(self.waypoint))
                    self.repeat+=1
            if status == 1:
                self.reached_goal = True
            if self.reached_goal or self.repeat>0:
                self.
        self.waiting_for_return = False

########################### Callbacks ##############################
    def benchmark_state_callback(self, data):
        if data.benchmark_state == BenchmarkState.STOP:
            rospy.loginfo("STOP")
        elif data.benchmark_state == BenchmarkState.PREPARE:
            rospy.loginfo("PREPARE")
            try:
                time.sleep(5)
                self.prepare()
            except:
                rospy.loginfo("Failed to reply PREPARE")
        elif data.benchmark_state == BenchmarkState.EXECUTE:
            rospy.loginfo("EXECUTE")

    def hearCommand_callback(self,data):
        rospy.loginfo('Heard a command')
        raw = str(data)
        lower = raw.lower()
        text = lower.split('~')[0]
        speech = text.split(':')[1]
        words = speech.split(' ')[1:]
        rospy.loginfo(speech)
        words = [x for x in words if x!='the']
        possible_verbs = self.tbm1_commands_dict.keys()
        if words[0] in possible_verbs:
            valid_command = True
            verb = words[0]
            subject = words[1:]
        else:
            valid_command=False
            self.pub_talk.publish("Invalid command please try again")
            return
        if verb == "move" or verb =="turn":
            self.handle_moves(verb,subject)
        elif verb == "go":
            self.handle_destinations(subject)
        elif verb == "look":
            self.handle_camera_direction(subject)
        elif verb == "see":
            self.handle_picture_taking(subject) 
        elif verb == "look":
            self.handle_camera_direction(subject)    
        elif verb == "end":
            self.execute()
        return 

    def current_pose_callback(self, data):
        self.current_pose = data.feedback.base_position.pose
        return

    def handle_destinations(self,subject):
        rospy.loginfo('Heard a destination')
        destination_list = [x for x in subject if x!="to"]
        destination = ' '.join(destination_list)
        self.pubGoal.publish(destination)
        return

    def handle_moves(self,verb,subject):
        rospy.loginfo('Moving')
        rospy.loginfo(verb)
        rospy.loginfo(subject[0])
        #rospy.loginfo(subject[1])
        if len(subject)>1:
            if subject[1] == "more":
                distance = 5
            elif subject[1] == "less":
                distance = 1
            else:
                distance = 1
        else:
            distance = 2
        rospy.loginfo(distance)
        d=0
        t0 = rospy.get_time()
        while(d<distance):
            if(verb == 'move' and subject[0] == 'forward'):
                self.send_directions(self.move_step_size,0)
            elif(verb=='move' and subject[0] == 'backward'):
                self.send_directions(-1*self.move_step_size,0)
            elif(verb == 'turn' and subject[0] == 'left'):
                self.send_directions(0,self.turn_step_size)
            elif(verb == 'turn' and subject[0] == 'right'):
                self.send_directions(0,-1*self.turn_step_size)
            elif(verb == 'turn' and subject[0] == 'around'):
                self.send_directions(0,-1*self.turn_step_size)
                distance = 20
            d = rospy.get_time()-t0
          
    def send_directions(self,straight,turn):
        rospy.loginfo('Sending out a direction')
        t = Twist()
        t.linear.x = straight
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = turn

        self.pub_move.publish(t)
        rospy.sleep(0.01)
    
    def object_position(self):
        ####Read in base position
        d = 1 #distance from object
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        t = self.current_pose.orientation.w
        x2 = round(x + d*cos(t),2)
        y2 = round(y + d*sin(t),2)
        z2 = .5
        l = '['+str(x2)+','+str(y2)+','+str(z2)+']'
        return(l)







if __name__ == '__main__':
    rospy.init_node('visit_my_home', anonymous=True)
    rospy.loginfo("Visit my home controller has started")
    controller = Controller()
    rospy.spin()
