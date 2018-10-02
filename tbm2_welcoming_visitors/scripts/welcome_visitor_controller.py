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

from aggregator.msg import Face_recog_verdict, Tracking_info# My personal messages DANIEL

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

        self.pub_face = rospy.Publisher('/recognising_visitor/controller',
            Face_recog_verdict,
            queue_size=10)#DANIEL

        self.pub_tracking = rospy.Publisher('/tracking_visitor/controller',
            Tracking_info,
            queue_size=10)#DANIEL

        # init subscribers
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
        rospy.Subscriber("roah_rsbb/benchmark",       Benchmark,      self.benchmark_callback)
        rospy.Subscriber("roah_rsbb/devices/bell",    Empty,          self.bell_callback)



        # init vars
        self.location_result = ""
        self.current_pose = None
        self.has_scan_changed = False
        self.recognition = None #DANIEL
        self.vision_status = None
        self.called = False

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

#####################VISION FACE RECOGNITION#######################

    def detect_visitor_face(self): #DANIEL
        subD = rospy.Subscriber("/recognising_visitor/vision",    Face_recog_verdict,self.recognition_callback) #DANIEL
        rospy.loginfo("subscribed to topic /recognising_visitor/vision")
        while self.recognition is None:
            rospy.sleep(0.1)
        subD.unregister()


        return self.recognition
        #TODO deliman and plumber will not have known faces, but must be identified in other ways, not just turned away

    #DANIEL
    def recognition_callback(self, msg):
        rospy.loginfo("face_callback: " + msg.best_pick)
        self.recognition = msg.best_pick

############VISION PEOPLE TRACKING############################

    def track_visitor(self): #DANIEL
        subT = rospy.Subscriber("/tracking_visitor/vision",    Tracking_info,self.tracking_callback) #DANIEL
        rospy.loginfo("subscribed to topic /tracking_visitor/vision")
        while self.visitor_status is None: #TODO When the plumber finished for example or when the visitor goes out of sight
            rospy.sleep(0.1)
            print "."
        rospy.loginfo("unsubscribed to topic /tracking_visitor/vision")
        subT.unregister()

        return self.visitor_status
        #TODO deliman and plumber will not have known faces, but must be identified in other ways, not just turned away

    #DANIEL
    def tracking_callback(self, msg):
        rospy.loginfo("tracking_callback: " + msg.decision)
        self.visitor_status = msg.decision


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
        if self.called == False:
            self.called = True#TODO uncomment this:
            #TODO uncomment this: self.say("I am coming")

            #if self.move_to_location("entrance", 3) == False:
            #    self.say("I am unable to move to the front door")
                #return #TODO uncomment this

            # TODO open door
            #TODO uncomment this:self.say("please open the door")

        # TODO detect door is opened
        rospy.sleep(3)

            #TODO raise torso - use pose movement

        self.say("please look at me so that I can recognise you")

            #TODO use new calls
        visitor = None
        Recog_visitor_msg = Face_recog_verdict() #DANIEL
        Recog_visitor_msg.scan_time = rospy.Duration(10)
        self.pub_face.publish(Recog_visitor_msg)
        visitor = self.detect_visitor_face()


        if visitor == "postman":
            rospy.loginfo("detected postman")
            self.process_face_postman()
        elif visitor == "doctor":
            rospy.loginfo("detected doctor")
            self.process_face_doctor()
        elif visitor == "Unknown":
            rospy.loginfo("do not recognize face")
            self.say("I do not recognise your face. Who are you?")

            #listen for reply, act accordingly
            self.toggle_stt('on')
            answer = self.speech

            if 'deliman' in answer or 'deli man' in answer:
                self.toggle_stt('off')
                rospy.loginfo("detected deliman")
                self.process_face_deliman()

            elif 'plumber' in answer:
                self.toggle_stt('off')
                rospy.loginfo("detected plumber")
                self.process_face_plumber()

            elif 'dr' in answer or 'doctor' in answer or 'kimble' in answer:
                self.toggle_stt('off')
                rospy.loginfo("detected plumber")
                self.process_face_doctor()

            elif 'postman' in answer or 'post man' in answer:
                self.toggle_stt('off')
                rospy.loginfo("detected plumber")
                self.process_face_postman()

            else:
                rospy.loginfo("visitor not recognize")

        else:
            rospy.loginfo("no faces detected")

##############################################NEEDS INCLUSION IN ROBOT BEHAVIOUR###############################
    def function_to_be_called_by_process_face_methods(self): # DANIEL
        Tracking_visitor_msg = Tracking_info() #DANIEL
        Tracking_visitor_msg.tracking_flag = True #CHANGE WITH AUDIO RESPONSE OF TASK FINISHED
        self.pub_tracking.publish(Tracking_visitor_msg)
        visitor = self.track_visitor()
####################################################################################################

    def process_face_postman(self):
        # Door should be open already

        # 3.b speak to the postman
        self.say("Hello postman, I will receive the post mail, please stand back while I move my arm")

        #TODO check postman has stood back/area is clear for movement
        rospy.sleep(2)


        # move to receive pose
        self.move_to_pose("give_receive")
        self.move_to_pose("open_gripper")

        self.say("Please place the parcel in my hand and I will close it when you say ready") #TODO maybe a better set of words to use?
        #TODO wait for response

        #TODO loop until "ready" said? or timeout to repeat command?
        self.toggle_stt('on')
        answer = None
        while 'ready' not in answer:
            answer = self.speech
            rospy.sleep(0.1)
            if answer is not None and 'ready' not in answer:
                self.say("Please say ready")
                answer = None
        self.toggle_stt('off')


        # close gripper
        self.move_to_pose("close_gripper")
        #TODO some sort of check to make sure object is in gripper

        self.say("Thank you for the parcel, I will give it to Granny Annie now")

        # move arm closer to make navigation easier
        self.move_to_pose("hold_close")

        # 6. bid postman farewell         #TODO figure out way to do it autonomously
        self.say("Thank you for visiting. Please close the door behind you. Goodbye!")

        #TODO wait til door is shut?
        rospy.sleep(5)

        #TODO navigate to granny annie, keep far enough away so as to not hit granny annie when offering parcel
        if self.move_to_location("bedroom", 3) == False:
            return

        # move arm to offer parcel
        self.move_to_pose("give_receive")

        self.say("Hello Grannie Annie. Please take the parcel in my hand, please tell me when you are ready and I will let go.") #TODO maybe a better set of words to use?

        #TODO loop until "ready" said? or timeout to repeat command?
        self.toggle_stt('on')
        answer = None
        while 'ready' not in answer:
            answer = self.speech
            rospy.sleep(0.1)
            if answer is not None and 'ready' not in answer:
                self.say("Please tell me when you are ready.")
                answer = None
        self.toggle_stt('off')

        #TODO release if yes
        self.say("I am letting go now.")
        self.move_to_pose("open_gripper")

        # move arm close in again so easier to move back to base
        self.move_to_pose("hold_close") #TODO maybe tuck arm instead?

        #TODO say something before leaving?
        self.say("See you later Grannie Annie.")

        # 7. return to base
        if self.move_to_location("home", 3) == False:
            return

    def process_face_deliman(self):
        self.say("Hello deliman, please come in and close the door behind you.")

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
        if self.move_to_location("entrance", 3) == False:
            return

        # 11. bid deliman farewell
        self.say("Thank you for visiting. Please close the door behind you. Goodbye!")
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
        #TODO implement guiding behavior
        if self.move_to_location("doctor_lead", 3) == False:
            return

        # 5. speak to doctor, advise robot will wait
        self.say("Please enter, I will wait here. Stand facing me when you are ready to leave.")

        # 6. wait until doctor exits the bedroom
        rospy.sleep(3) #TODO is this here to give dr time to move out of sight?
        self.wait_for_scan_changed() # waits to detect face again

        #TODO confirm they are actually done?
        self.say("Please tell me when you are ready to leave.")
        self.toggle_stt('on')
        answer = None
        while 'ready' not in answer:
            answer = self.speech
            rospy.sleep(0.1)
            if answer is not None and 'ready' not in answer:
                self.say("Please tell me when you are ready")
                answer = None
        self.toggle_stt('off')

        self.say("Now you are done, I will follow you to the door. Please lead the way.")
        rospy.sleep(3)


        # 9. move to hallway
        if self.move_to_location("entrance", 3) == False: #TODO follow dr, don't just run them over!
            return

        # 10. bid farewell
        self.say("Thank you for visiting. Please close the door behind you. Goodbye!")

        #TODO any way to detect door is closed?
        rospy.sleep(2)
        # 11. return to base
        if self.move_to_location("home", 3) == False:
            return


    #PLUMBER HERE #DANIEL 1st OCT 2018
    def process_face_plumber(self):
        # ask plumber where they would like to go
        self.say("Hello Plumber, which room would you like to go to?")

        #listen for answer
        self.toggle_stt('on')
        while self.speech == None:
            room = self.speech
            rospy.sleep(0.1)
        self.toggle_stt('off')


        #go to the room, or not
        if 'bathroom' in room or 'bath room' in room:
            self.say("Please follow me to the bathroom.")
            #TODO implement following behaviour
            # for now just navigate to the location
            if self.move_to_location("plumber_bathroom", 3) == False:
                return
            #wait for plumber to finish

            #wait for answer
            #TODO confirm they are actually done?
            self.say("Please tell me when you are ready to leave.")
            self.toggle_stt('on')
            answer = None
            while 'ready' not in answer:
                answer = self.speech
                rospy.sleep(0.1)
                if answer is not None and 'ready' not in answer:
                    self.say("Please tell me when you are ready")
                    answer = None
            self.toggle_stt('off')

            self.say("Now you are done, I will follow you to the door. Please lead the way.")
            rospy.sleep(3)
            if self.move_to_location("entrance", 3) == False:
                            return


        elif 'kitchen' in room:
            self.toggle_stt('off')
            self.say("Please follow me to the kitchen.")
            #TODO implement 'following' behaviour

            # for now just navigate to the location
            if self.move_to_location("kitchen", 3) == False:
                return

            #wait for plumber to finish
            self.say("Please tell me when you are ready to leave.")
            self.toggle_stt('on')
            answer = None
            while 'ready' not in answer:
                answer = self.speech
                rospy.sleep(0.1)
                if answer is not None and 'ready' not in answer:
                    self.say("Please tell me when you are ready")
                    answer = None
            self.toggle_stt('off')

            self.say("Now you are done, I will follow you to the door. Please lead the way.")
            rospy.sleep(3)
            if self.move_to_location("entrance", 3) == False:
                return

        else:
            self.say("I am sorry I cannot take you there.")



        if self.move_to_location("home", 3) == False:
            return

'''
    def process_face_unrecognized(self):
        # 1. speak to visitor, "Sorry, I don't know you. I cannot open the door."
        self.say("Sorry, I don't recognize you. I cannot let you in. Please close the door.") # TODO door will be opened first, so shut door now

        if self.move_to_location("home", 3) == False:
            return
'''

#Main
if __name__ == '__main__':
    rospy.init_node("task_controller", anonymous=True)
    rospy.loginfo("initialized controller node")
    newcontroller = ControllerTBM2()
    newcontroller.move_to_location("kitchen", 3)
    rospy.spin()
