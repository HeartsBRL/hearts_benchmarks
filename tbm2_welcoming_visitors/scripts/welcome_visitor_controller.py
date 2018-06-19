#!/usr/bin/env python

# https://github.com/studioimaginaire/phue
# https://github.com/rockin-robot-challenge

import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose2D, Twist, Pose
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState, DevicesState, TabletState
import std_srvs.srv
from collections import Counter

# activation: doorbell press detected
# notification:
# visitor: postman, doctor, deliman, unknown

class Controller():
    def __init__(self):
        self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        #self.pub_task = rospy.Publisher('hearts/controller/task', String, queue_size=10)
        self.pub_location_goal = rospy.Publisher('/hearts/navigation/goal/location', String, queue_size=10)
        self.pub_twist = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)       

        # subscribers
        #rospy.Subscriber("hearts/voice/cmd",           String, self.voice_callback)
        rospy.Subscriber("/hearts/navigation/status", String, self.location_result_callback)
        #rospy.Subscriber("roah_devices/state",        DevicesState, self.state_callback)
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
        rospy.Subscriber("roah_rsbb/benchmark",        Benchmark, self.benchmark_callback)
        #rospy.Subscriber("roah_rsbb/benchmark",        String, self.benchmark_callback)
        rospy.Subscriber("roah_rsbb/devices/bell",     Empty, self.bell_callback)
        #rospy.Subscriber("hearts/front_door/leaving",     Empty, self.leaving_callback)
        
        self.start_track = rospy.ServiceProxy('/start_person_tracking',std_srvs.srv.Trigger)
        self.end_track = rospy.ServiceProxy('/stop_person_tracking',std_srvs.srv.Trigger)
        
        self.tts_pub = rospy.Publisher("/hearts/tts", String, queue_size=10)
        self.location_result = ""
        self.current_pose = None
        #self.leaving = False

        #self.loop()

        self.has_scan_changed = False

    def scan_changed_callback(self, msg):
        self.has_scan_changed = (msg.data == "yes")

    def wait_for_scan_changed(self):
        self.has_scan_changed = False
        rospy.Subscriber("/scan_change", String, self.scan_changed_callback)
        
        while not self.has_scan_changed:
            rospy.sleep(1)
           
        # TODO: kill subscriber 
    
    def detect_visitor(self):
        self.votes = [ ]
        sub = rospy.Subscriber("/hearts/face/user", String, self.face_callback)
        rospy.loginfo("subscribed to topic /hearts/face/user")
        rospy.sleep(20)
        sub.unregister()
        rospy.loginfo("unsubscribed from topic /hearts/face/user - votes = " + str(len(self.votes)))
        counts = Counter(self.votes)
        rospy.loginfo(str(counts))
        visitors = counts.most_common(2)
        visitor = None
        if len(visitors) == 2:
            visitor1 = visitors[0]
            visitor2 = visitors[1]
            
            if visitor1[1] != visitor2[1]:
                visitor = visitor1[0]
        elif len(visitors) == 1:
            visitor = visitors[0][0]
        
        if not visitor is None: 
            rospy.loginfo("visitor = " + visitor)
        
        return visitor
        
    def face_callback(self, msg):
        rospy.loginfo("face_callback: " + msg.data)
        self.votes.append(msg.data)         

    #def voice_callback(self, data):
    #    rospy.loginfo("voice_callback: " + data.data)
    #    self.current_voice = data.data
 
    #def location_result_callback(self, data):
    #    rospy.loginfo("location_result_callback") 
  
#   def state_callback(self, data):
#        rospy.loginfo("state_callback")

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

    def benchmark_callback(self, data):
        rospy.loginfo("benchmark_callback")
 
    #def leaving_callback(self, data):
    #    self.leaving = True

    def bell_callback(self, data):
        rospy.loginfo("bell_callback")
        
        self.say("I am coming")
        
        if self.move_to("entrance", 3) == False:
            self.say("I am unable to move to the front door")
            return
            
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

    #def state_callback(self, data):
    #    rospy.loginfo("state_callback: " + data.data)
    #    if self.detect_faces == False and self.bell < data.bell:
    #        self.bell = data.bell
    #        # bell pressed, detect face
    #        self.detect_faces = True
    
    def say(self, text):
        rospy.loginfo("saying \"" + text + "\"")
        rospy.sleep(1)
        self.tts_pub.publish(text)
        rospy.sleep(5)
        
    def move_to(self, location, count):
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
            self.move_to(location, count - 1)
            
        return self.location_result == "Success"
        
    #def wait_until_left(self):
    #    while self.leaving == False:
    #        rospy.sleep(1)
    #    self.leaving = False

    def loop(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def location_result_callback(self, data):
        rospy.loginfo(data.data)
        self.location_result = data.data

    def process_face_postman(self):
                
        # 3.b speak to the postman, request him to open the door:  "I am here, please open the door"
        self.say("I will receive the post mail, please open the door")
 
        # 4. detect door is open
        rospy.sleep(3)

        # 5. speak to postman, instruct to leave parcel on hallway floor
        self.say("Please leave the parcel on the floor")
        rospy.sleep(2)
        
        # 6. bid postman farewell
        self.say("Thank you for visiting. Goodbye!")

        # 7. return to base
        if self.move_to("home", 3) == False:
            self.say("I am unable to move to the base")
            return

    def process_face_deliman(self):
        
        # 3. speak to the deliman, request him to open the door: "I am here, please open the door"
        self.say("I will receive the breakfast, please open the door")

        # 4. detect door is open
        rospy.sleep(3)

        # 5. speak to the deliman, instruct to follow robot: "Please follow me"
        self.say("Hello! Please follow me")

        # 6. move to kitchen
        if self.move_to("home", 3) == False:
            self.say("I am unable to move to the kitchen")
            return

        # 7. speak to the deliman, instruct to leave breakfast box on the table: "Please leave the breakfast box on the table"
        self.say("Please leave the breakfast box on the table")
# topic: /y/compressed
        # 8. wait
        # TODO
        rospy.sleep(5)
        
        # 9. speak to the deliman, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 10. move to front door
        if self.move_to("hallway", 3) == False:
            self.say("I am unable to move to the front door")
            return

        # 11. bid deliman farewell
        self.say("Thank you for visiting. Goodbye!")

        # 12. return to base
        if self.move_to("home", 3) == False:
            self.say("I am unable to move to the base")
            return

    def current_pose_callback(self, pose):
        self.current_pose = pose

    def process_face_doctor(self):
        # 1. speak to the doctor, "Hi Dr. Kimble, I am coming to open the door."
        self.say("Hello Dr. Kimble, please open the door")

        # 2. detect door is open
        rospy.sleep(3)

        # 3. speak to the doctor, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 4. move to bedroom
        if self.move_to("outside bedroom", 3) == False:
            self.say("I am unable to move to the bedroom")
            return

        # 5. speak to doctor, advise robot will wait
        self.say("Please enter. Stand facing me when you are done.")
    
        # 6. wait until doctor exits the bedroom
        rospy.sleep(3)
        self.wait_for_scan_changed()
        self.say("Now you are done. I will follow you to the door. Please lead the way.")
                
        #self.say("Now you are done.")
        rospy.sleep(2)
        
        # 9. move to hallway
        if self.move_to("hallway", 3) == False:
            self.say("I am unable to move to the front door")
            return
            
        # 10. bid farewell
        self.say("Thank you for visiting. Goodbye!")

        # 11. return to base
        if self.move_to("home", 3) == False:
            self.say("I am unable to move to the base")
            return

    def process_face_unrecognized(self):
        # 1. speak to visitor, "Sorry, I don't know you. I cannot open the door."
        self.say("Sorry, I don't know you. I cannot open the door")
        
        if self.move_to("home", 3) == False:
            self.say("I am unable to move to the base")
            return

if __name__ == '__main__':
    rospy.init_node("task_controller", anonymous=True)
    rospy.loginfo("initialized controller node")
    controller = Controller()
    rospy.spin()
