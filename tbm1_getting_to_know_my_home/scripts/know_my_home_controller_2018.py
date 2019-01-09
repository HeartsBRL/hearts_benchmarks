#!/usr/bin/env python
##############################################################################################
# Author  : Joe Daly
# Created : Sept 2018
# Purpose :
#
##############################################################################################
# Updates :
#
#
##############################################################################################

import rospy
import time
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
#from turtlesim.msg import Pose
from math import cos, sin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rospy.rostime import Duration
#from move_base_msgs.msg import MoveBaseActionFeedback
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState
import std_srvs.srv

import python_support_library.text_colours as TC
prt =TC.tc()

from navigation_camera_mgr_example import NavigationCameraMgr

from python_support_library.generic_controller import GenericController

class ControllerTBM1(GenericController):
    def __init__(self):
        # init the generic stuff from GenericController
        super(ControllerTBM1, self).__init__()

        #init tbm1 specific stuff

        ### init publishers - sends out goal locations, movement/turns, and speech
        self.pubGoal  =  rospy.Publisher('hearts/navigation/goal/location', String, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)
        self.pub_pic  =  rospy.Publisher('/hearts/camera/snapshot', String, queue_size = 10)
        self.pub_head = rospy.Publisher('/head_controller/command',JointTrajectory, queue_size = 10)
        self.pub_move = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
        #self.pub_dummy = rospy.Publisher('/move_base/feedback', MoveBaseActionFeedback, queue_size = 10)

        ### init subscribers - must listen for speech commands, location
        #rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.current_pose_callback)
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)

        self.prepare   = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        self.execute   = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty)
        self.outfolder = rospy.get_param('output_path')


        # init vars
        self.objects   = ['coke','water','juice','apple']


        # Dining_table onwards are not movable furniture
        self.furniture = ['Trash_bin','Plant','Side_table_1','Dining_chair','Kitchen_chair','Coffee_table','Arm_Chair_1','Arm_Chair_2',
        'Dining_table','Kitchen_table','Kitchen_counter','Side_table_2','Bathroom_drawer','Closet','Bed','Bookshelf','Sofa','TV_Table']
        self.rooms = ['Entrance_hall','Hallway','Kitchen', 'Bedroom', 'Living_room', 'Dining_room', 'hall']


        #list of doors to be done prior to competition
        self.doors = ['bedroom','entrance','bathroom']



        # disable head manager
        head_mgr = NavigationCameraMgr()
        head_mgr.head_mgr_as("disable")



    def run(self):

        self.say("I am ready to get know my home")
        # detected, word = self.stt_detect_words(["yes", "yeah", "ya", "ye", "yay", "yo"], 3)
        # start with doors
        self.process_doors()

        # then check FURNITURE
        self.process_furniture()

        # then look for OBJECTS
        self.process_objects()

        # finally look for TRASH
        #self.process_trash()




    def benchmark_state_callback(self, data):
        '''
        Trigged by subscriber: roah_rsbb/benchmark/state

        Receive instructions from judges code to prepare, execute and stop.

        '''
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


    def process_doors(self):
        ''' Ask whether any doors have changed, go to the door,
        take a picture and log it in the semantic map. '''

        rospy.loginfo("Door process")
        thing_id = None
        room1 = None
        room2 = None


        # ask about doors
        self.say("Have any doors changed?")
        #detected, word = self.stt_detect_words(["yes", "yeah", "ya", "ye", "yay", "yo"], 3)

        detected = True
        word = "entrance"

        ######TESTING TESTING 1,2,1,2,1,2,
        if detected: # if the function returns true
            self.say("Which door has changed?")
            #detected, word = self.stt_detect_words(self.doors, 3)
            #TODO catch the None before it causes problems
            self.say("Okay I will look at the "+str(word)+" door")
            if detected:
                open_status = 'closed'
                if word == 'bedroom':
                    self.say("Please tell me which other room connects to the changed bedroom door")
                    detected, word = self.stt_detect_words(["hall", "living"], 3)
                    if detected:
                        if word == 'hall':
                            thing_id = 'Door_bedroom_1'
                            room1 = 'bedroom'
                            room2 = 'hall'

                        elif word == 'living':
                            thing_id = 'Door_bedroom_2'
                            room1 = 'bedroom'
                            room2 = 'living_room'

                elif word == 'entrance':
                            thing_id = 'door_entrance'
                            room1 = 'hall'
                            room2 = 'outside'

                elif word == 'bathroom':
                            thing_id = 'Door_bathroom'
                            room1 = 'bathroom'
                            room2 = 'kitchen'

            #navigate to door TODO check with organisers whether this is necessary
            # if self.move_to_location(word, 3) == False:
            #     return

            #take a picture of the changed object
            print ("Taking Picture now")
            self.pub_pic.publish(thing_id+'.jpg')



            # update semantic_map with door detected
            line1 = self.linewriter('type',[thing_id,'door'])
            line2 = self.linewriter('connects',[thing_id,room1,room2])
            line3 = self.linewriter('isOpen',[thing_id,open_status])
            to_write = [line1,line2,line3]
            f = open(self.outfolder+'semantic_map.txt','a+')
            for line in to_write:
                f.write(line+'\n')
                rospy.loginfo(line)
            f.close()


        else:
            self.say("I heard nothing")




    def process_objects(self):
        rospy.loginfo("Object process")
#     ################### OBJECTS ###################
#         object_count = 0

#         if object_count < 2:
#             # go to the next possible object location
#             self.move_to_location([location_i])

#             # compare expected vision output to actual vision output




#             # if expected is not same as actual vision output
#            # object_count = object_count +1

#                 # take photo, save to memory stick

#                 # update semantic_map with object detected
        thing_id = 'biscuits'
        item = 'biscuits'
        room = 'bedroom'
        furniture = 'wardrobe'
        position_string = 'inside'


        line1 = self.linewriter('type',[thing_id, item])
        line2 = self.linewriter('in',[thing_id, room])
        line3 = self.linewriter('on',[thing_id, furniture])
        line4 = self.linewriter('position ',[thing_id, position_string])
        line6 = self.linewriter('picture',[thing_id,thing_id+'.jpg'])
        to_write = [line1,line2,line3,line4,line6]
        f = open(self.outfolder+'semantic_map.txt','a+')
        for line in to_write:
            f.write(line+'\n')
            rospy.loginfo(line)
        f.close()



#             # elif object_count = object_count



#         else:
#             # do the next thing

    def process_furniture(self):
        rospy.loginfo("Furniture")
#         ################### FURNITURE ###################
#             # ask user for which furniture has changed
        #
        # self.say("Has any furniture changed?")
        # detected, word = self.stt_detect_words(["yes", "yeah", "ya", "ye", "yay", "yo"], 3)
        # if detected: # if the function returns true
        #     self.say("Which furniture has changed?")
        #     detected, furniture = self.stt_detect_words(self.furniture, 3)
        #     #TODO catch the None before it causes problems
        #     self.say("Which room is is in now?")
        #     detected, room = self.stt_detect_words(self.rooms, 3)

        thing_id = "chair"
        room = "kitchen"

        line1 = self.linewriter('type',[thing_id,thing_id])
        line2 = self.linewriter('in',[thing_id,room])
        to_write = [line1,line2]
        f = open(self.outfolder+'semantic_map.txt','a+')
        for line in to_write:
            f.write(line+'\n')
            rospy.loginfo(line)
        f.close()



#             #ask for the room where the furniture is
#             self.say("Please tell me which room the"+furniture+"has been moved to.")



#             # go to the room
#             self.move_to_location()

#             # take photo, save to memory stick




#         ################### TRASH ###################

#         # scan floor for trash (i.e. small object detected on the floor)

#         # if object detected on the floor

#            # update semantic_map





# ################### SEMANTIC_MAP ###################
    def linewriter(self, descriptor, options_list):
        text = descriptor + '('
        for option in options_list:
            text = text+option+', '
        text = text[:-2]+').'
        return(text)


###################### INIT ########################
if __name__ == '__main__':
    rospy.init_node('task_controller', anonymous=True)
    rospy.loginfo("know my home controller has started")
    newcontroller = ControllerTBM1()
    #newcontroller.move_to_location("home", 3)
    newcontroller.run()
    rospy.spin()
