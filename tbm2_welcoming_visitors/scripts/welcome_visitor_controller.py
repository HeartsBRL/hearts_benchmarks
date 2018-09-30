#!/usr/bin/env python

# https://github.com/studioimaginaire/phue
# https://github.com/rockin-robot-challenge

import rospy
import time
from std_msgs.msg      import Empty, String
from geometry_msgs.msg import Pose2D, Twist, Pose
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState, DevicesState, TabletState
import std_srvs.srv
from collections import Counter

from python_support_library.generic_controller import GenericController

# activation: doorbell press detected
# notification:
# visitor: postman, doctor, deliman, unknown

class ControllerTBM2(GenericController):
    def __init__(self):
        # init the generic stuff from GenericController
        super(ControllerTBM2, self).__init__() 
        
        # init tbm2 specific stuff
        
        # init service proxies        
        self.start_track = rospy.ServiceProxy('/start_person_tracking', std_srvs.srv.Trigger)
        self.end_track =   rospy.ServiceProxy('/stop_person_tracking',  std_srvs.srv.Trigger)
        self.prepare =     rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        
        # init publishers
        self.pub_twist = rospy.Publisher('/mobile_base_controller/cmd_vel', 
                                         Twist, 
                                         queue_size=10)       

        # init subscribers
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
        rospy.Subscriber("roah_rsbb/benchmark",       Benchmark,      self.benchmark_callback)
        rospy.Subscriber("roah_rsbb/devices/bell",    Empty,          self.bell_callback)

        # init vars
        self.location_result = ""
        self.current_pose = None
        self.has_scan_changed = False

    def scan_changed_callback(self, msg):
        '''Trigged by subscriber: /scan_change '''
        self.has_scan_changed = (msg.data == "yes")
    def wait_for_scan_changed(self):
        '''Starts /scan_change subscriber and waits until a scan changes '''
        self.has_scan_changed = False
        rospy.Subscriber("/scan_change", String, self.scan_changed_callback)
        
        while not self.has_scan_changed: #TODO add timer to request person to move to better viewing position if difficulty detecting? Might affect dr kimble code...
            rospy.sleep(1)
           
        # TODO: kill subscriber 
    
    def detect_visitor(self):
        #TODO figure out what this all does and comment it
        self.votes = [ ]
        sub = rospy.Subscriber("/hearts/face/user", String, self.face_callback)
        rospy.loginfo("subscribed to topic /hearts/face/user")
        rospy.sleep(20) #TODO this is a huge delay for every time a face needs to be detected!
        sub.unregister()
        rospy.loginfo("unsubscribed from topic /hearts/face/user - votes = " + str(len(self.votes)))
        counts = Counter(self.votes)
        rospy.loginfo(str(counts))
        visitors = counts.most_common(2)
        likliest_visitor = None
        if len(visitors) == 2:
            visitor1 = visitors[0]
            visitor2 = visitors[1]
            
            if visitor1[1] != visitor2[1]:
                likliest_visitor = visitor1[0]
        elif len(visitors) == 1:
            likliest_visitor = visitors[0][0]
        
        if likliest_visitor is not None: 
            rospy.loginfo("visitor = " + likliest_visitor)
        
        return likliest_visitor
        #TODO deliman and plumber will not have known faces, but must be identified in other ways, not just turned away
        
    def face_callback(self, msg):
        rospy.loginfo("face_callback: " + msg.data)
        self.votes.append(msg.data)         

    #def voice_callback(self, data):
    #    rospy.loginfo("voice_callback: " + data.data)
    #    self.current_voice = data.data
  
    def benchmark_state_callback(self, data):
        '''
        Trigged by subscriber: roah_rsbb/benchmark/state
        
        Receive instructions from judges code to prepare, execute and stop.
        '''
        if data.benchmark_state == BenchmarkState.STOP:
            rospy.loginfo("STOP") #TODO does this actually do anything/meant to?
        elif data.benchmark_state == BenchmarkState.PREPARE:
            rospy.loginfo("PREPARE")
            try:
                time.sleep(5)
                self.prepare()
            except:
                rospy.loginfo("Failed to reply PREPARE")
        elif data.benchmark_state == BenchmarkState.EXECUTE:
            rospy.loginfo("EXECUTE") #TODO does this actually do anything/meant to?

    def benchmark_callback(self, data):
        '''Trigged by subscriber: roah_rsbb/benchmark '''
        rospy.loginfo("benchmark_callback")
 
    #def leaving_callback(self, data):
    #    self.leaving = True

    def bell_callback(self, data):
        '''
        Trigged by subscriber: roah_rsbb/devices/bell 
        
        Start actions to interact with visitor:
        -> move to front door
        -> request visitor opens door (self opening not implemented yet)
        -> visual recognition of visitor
        -> call visitor-dependant functions
        
        '''
        rospy.loginfo("bell_callback")
        
        self.say("I am coming")
        
        if self.move_to_location("entrance", 3) == False:
            self.say("I am unable to move to the front door")
            return
            
        # TODO open door
        
        # TODO detect door is opened
            
        self.say("please look towards the camera so that I can recognise you")
        
        visitor = None
        while visitor is None or visitor == "":
            visitor = self.detect_visitor()
        
        if visitor == "postman":
            rospy.loginfo("detected postman")
            self.process_face_postman()
        elif visitor == "deliman":
            rospy.loginfo("detected deliman")
            self.process_face_deliman()
        elif visitor == "doctor":
            rospy.loginfo("detected doctor")
            self.process_face_doctor()
        elif visitor == "unknown":
            rospy.loginfo("detected unrecognized person")
            self.process_face_unrecognized()  
        

        
    #def wait_until_left(self):
    #    while self.leaving == False:
    #        rospy.sleep(1)
    #    self.leaving = False

    #def loop(self):
    #
    #   rate = rospy.Rate(10)
    #   while not rospy.is_shutdown():
    #       rate.sleep()



    def process_face_postman(self):
        # Door should be open already
        
        # 3.b speak to the postman
        self.say("Hello postman, I will receive the post mail, please stand back while I move my arm") 
        
        #TODO check postman has stood back/area is clear for movement
        
        # move to receive pose
        self.move_to_pose("give_receive")
        self.move_to_pose("open_gripper")
        
        self.say("Please place the parcel in my hand and I will close it when you say ready") #TODO maybe a better set of words to use?
        #TODO wait for response
        #TODO loop until "ready" said? or timeout to repeat command?
        
        # close gripper
        self.move_to_pose("close_gripper")
        #TODO some sort of check to make sure object is in gripper
        
        
        self.say("Thank you for the parcel, I will give it to Granny Annie now")
        
        # move arm closer to make navigation easier
        self.move_to_pose("hold_close")
        
        # 6. bid postman farewell
        self.say("Thank you for visiting. Goodbye!")
        
        #TODO ask post man to shut door before they go or figure out way to do it autonomously         
        #TODO wait til door is shut?
        
        #TODO navigate to granny annie, keep far enough away so as to not hit granny annie when offering parcel
        
        self.say("Hello granny annie, the postman has bought you a parcel, I will pass it to you now")
        
        # move arm to offer parcel
        self.move_to_pose("give_receive")

        
        self.say("Have you got hold of it?")
        #TODO wait for response
        #TODO release if yes
        self.move_to_pose("open_gripper")
        
        # move arm close in again so easier to move back to base
        self.move_to_pose("hold_close") #TODO maybe tuck arm instead?

        #TODO say something before leaving?

        # 7. return to base
        if self.move_to_location("home", 3) == False:
            return

    def process_face_deliman(self):
        self.say("Hello deliman")
        self.say("I will receive the breakfast")

        # 5. speak to the deliman, instruct to follow robot: "Please follow me"
        self.say("Please follow me to the kitchen")

        # 6. move to kitchen
        if self.move_to_location("kitchen", 3) == False:
            return

        # 7. speak to the deliman, instruct to leave breakfast box on the table: "Please leave the breakfast box on the table"
        self.say("Please leave the breakfast box on the table") #TODO check if this will still be a breakfast box
        
        # 8. wait
        # TODO check if it has been done?
        rospy.sleep(5)
        
        # 9. speak to the deliman, instruct to follow robot: "Please follow me"
        self.say("Thank you, please follow me back to the door")

        # 10. move to front door
        if self.move_to_location("hallway", 3) == False:
            return

        # 11. bid deliman farewell
        self.say("Thank you for visiting. Goodbye!")
        #TODO close door/ask deliman to close door
        
        rospy.sleep(5)
        
        # 12. return to base
        if self.move_to_location("home", 3) == False:
            return

    #def current_pose_callback(self, pose):
    #    self.current_pose = pose

    def process_face_doctor(self):
        # 1. speak to the doctor, "Hi Dr. Kimble, I am coming to open the door."
        self.say("Hello Doctor Kimble, please come in and shut the door behind you. I will guide you to Granny Annie") #TODO check what names are appropriate?

        # 4. move to bedroom
        if self.move_to_location("outside bedroom", 3) == False:
            return

        # 5. speak to doctor, advise robot will wait
        self.say("Please enter, I will wait here. Stand facing me when you are ready to leave.")
    
        # 6. wait until doctor exits the bedroom
        rospy.sleep(3) #TODO is this here to give dr time to move out of sight?
        self.wait_for_scan_changed() # waits to detect face again
        self.say("Now you are done. I will follow you to the door. Please lead the way.")
        rospy.sleep(2)
        #TODO confirm they are actually done?
        
        # 9. move to hallway 
        if self.move_to_location("hallway", 3) == False: #TODO follow dr, don't just run them over!
            return
            
        # 10. bid farewell
        self.say("Thank you for visiting. Please close the door behind you. Goodbye!")

        #TODO any way to detect door is closed?
        rospy.sleep(2)
        # 11. return to base
        if self.move_to_location("home", 3) == False:
            return

    def process_face_unrecognized(self):
        # 1. speak to visitor, "Sorry, I don't know you. I cannot open the door."
        self.say("Sorry, I don't recognize you. I cannot let you in. Please close the door.") # TODO door will be opened first, so shut door now
        
        if self.move_to_location("home", 3) == False:
            return

if __name__ == '__main__':
    rospy.init_node("task_controller", anonymous=True)
    rospy.loginfo("initialized controller node")
    nowcontroller = ControllerTBM2()
    rospy.spin()
