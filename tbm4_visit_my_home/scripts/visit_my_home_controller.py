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
from actionlib_msgs.msg import GoalStatusArray

############################# Controller Class ############################################
class Controller():
    def __init__(self):
        # Movement Parameters
        self.ok_to_start = False
        self.head_lr = 0.0
        self.head_ud = -0.5
        self.head_move_step_size = .15
        self.turn_step_size = 0.2
        self.move_step_size = 0.2
        self.destination=0

        #self.handle_camera_direction(['center'])
        self.outfolder = rospy.get_param('output_path')
        self.tbm1_commands_dict = {
            "move": ["forward", "backward"],
            "turn": ["right", "left"],
            "look": ["up", "down", "right", "left"],
            "here": []}

        self.waypoint = 1

        ### Publishers - sends out goal locations, movement/turns, and speech
        self.pubGoal = rospy.Publisher('hearts/navigation/goal/location', String, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)
        self.pub_pic = rospy.Publisher('/hearts/camera/snapshot', String, queue_size = 10)
        self.pub_head = rospy.Publisher('/head_controller/command',JointTrajectory, queue_size = 10)
        self.pub_move = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)  
        self.pub_dummy = rospy.Publisher('/move_base/feedback', MoveBaseActionFeedback, queue_size = 10)
              
        ### Subscribers - must listen for speech commands, location
        rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)
        rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.current_pose_callback)
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
        rospy.Subscriber('/hearts/navigation/status', String, self.nav_status_callback)

        #TODO subscribe to succeed or fail
        self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        self.execute = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty)
        self.change_maps = rospy.ServiceProxy('/pal_map_manager/change_map',Acknowledgment)
        self.start_track = rospy.ServiceProxy('/start_person_tracking',std_srvs.srv.Trigger)
        self.end_track = rospy.ServiceProxy('/stop_person_tracking',std_srvs.srv.Empty)
        self.detect_obstacle = rospy.ServiceProxy('/start_person_detection',std_srvs.srv.Trigger)

        # Disable head manager
        rospy.sleep(10.)
        self.log_speak("End initialize, ready for start")
        return(None)
        
    def begin(self):
        while not rospy.is_shutdown():
            if self.ok_to_start:
                for self.waypoint in range(1,6):
                    self.log_speak("starting waypoint "+str(self.waypoint))
                    self.destination = self.waypoint #for handling waypoint 2
                    if self.waypoint == 1:
                        self.destination = .5
                        self.go_to_target(self.destination)
                        rospy.sleep(5)
                        self.log_speak("Person detected, I will go around")

                        self.destination = .6
                        self.go_to_target(self.destination)
                        self.go_to_target(self.waypoint)

                    elif self.waypoint == 2:
                        self.destination = 1.5
                        self.go_to_target(self.destination)
                        obstacle_type = self.detect_obstacle()
                        if obstacle_type.message == '1': #person
                            self.log_speak("Person detected, please move")
                        if obstacle_type.message == '2': #door
                            self.log_speak("Door detected, please open")
                        if obstacle_type.message == '3': #small obstacle
                            self.log_speak("Small obstacle detected, please move it")
                        rospy.sleep(10.)
                        self.go_to_target(self.waypoint)

                    elif self.waypoint == 3:
                        self.go_to_target(self.waypoint)

                    elif self.waypoint == 4:
                        self.ID_person()

                    else: #waypoint 5
                        self.destination = 4.5
                        self.go_to_target(self.destination)
                        rospy.sleep(5)
                        self.log_speak("knock knock, would someone please open the door")
                        rospy.sleep(5)
                        self.go_to_target(self.waypoint)
                        self.ok_to_start = False

                    self.continue_on = False # True when reached target or abandoned target
                    self.log_speak("Waypoint "+str(self.waypoint)+" completed, moving on")
            else:
                rospy.sleep(1)            

    def handle_fail(self):
        if self.ok_to_start:
            if self.waypoint == 1:
                self.go_to_target(1)
            if self.waypoint == 2:
                self.go_to_target(1.5)
                #rospy.sleep(5)
                self.log_speak("A obstacle is in the way. Please remove it")
                rospy.sleep(5.)
                self.go_to_target(2)
            if self.waypoint == 5:
                self.go_to_target(4.5)
            if self.waypoint == 3:
                self.continue_on = True
        return

    def go_to_target(self,w):
        self.pubGoal.publish(str(w))
        rospy.loginfo("Publish destination string:"+str(w))
        self.continue_on = False
        while not self.continue_on:
            rospy.sleep(1.)
        return

    def ID_person(self):
        rospy.loginfo("Looking for a person")
        #get from zeke or call zeke's functions
        self.log_speak("Please stand one meter directly in front of me")
        rospy.sleep(2.)
        self.log_speak("Please say cheese")
        success = self.start_track()
        while not success:
            self.log_speak("Please stand a bit closer")
            rospy.sleep(3.)
            success = self.start_track()
        self.log_speak("Found you. Please tell me the word stopping when we reach the destination")
        self.log_speak("If I am lost, say train me again")
        self.log_speak("Lead onward")
        while not self.continue_on:
            rospy.sleep(1)
        return

    def nav_status_callback(self,data):
        status = data.data
        if status == "Success":
            self.continue_on = True
        elif status == "Fail":
            self.handle_fail()
        return

########################### Callbacks ##############################
    def benchmark_state_callback(self, data):
        if data.benchmark_state == BenchmarkState.STOP:
            rospy.loginfo("STOP")
        elif data.benchmark_state == BenchmarkState.PREPARE:
            rospy.loginfo("PREPARE")
            try:
                time.sleep(5.)
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
        if words[0] == "starting":
            self.ok_to_start = True
        if words[0] == "stopping":
            _ = self.end_track()
            self.log_speak("Heard Stop. Thanks for leading me. I will go inside now")
            self.continue_on = True
        if words[0] == "train":
            _ = self.end_track()
            self.start_track()
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

    def log_speak(self, text):
        rospy.loginfo(text)
        self.pub_talk.publish(text)
        rospy.sleep(len(text)/5)






if __name__ == '__main__':
    rospy.init_node('visit_my_home', anonymous=True)
    rospy.loginfo("Visit my home controller has started")
    controller = Controller()
    controller.begin()
    rospy.spin()
