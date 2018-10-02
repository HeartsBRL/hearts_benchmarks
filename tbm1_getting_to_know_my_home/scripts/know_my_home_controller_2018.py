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
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
from turtlesim.msg import Pose
from math import cos, sin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rospy.rostime import Duration
from move_base_msgs.msg import MoveBaseActionFeedback
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState
import std_srvs.srv

from navigation_camera_mgr_example import NavigationCameraMgr

from python_support_library.generic_controller import GenericController

class ControllerTBM1(GenericController):

	def __init__(self):
    	# init the generic stuff from GenericController
        super(ControllerTBM1, self).__init__()


        ### Publishers - sends out goal locations, movement/turns, and speech
        self.pubGoal = rospy.Publisher('hearts/navigation/goal/location', String, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)
        self.pub_pic = rospy.Publisher('/hearts/camera/snapshot', String, queue_size = 10)
        self.pub_head = rospy.Publisher('/head_controller/command',JointTrajectory, queue_size = 10)
        self.pub_move = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
        self.pub_dummy = rospy.Publisher('/move_base/feedback', MoveBaseActionFeedback, queue_size = 10)

        ### Subscribers - must listen for speech commands, location
        rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.current_pose_callback)
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)

        self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        self.execute = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty)

        self.outfolder = rospy.get_param('output_path')

        self.objects = ['Coke','Water','Juice','Apple',
		 'Lemon','Rice','Pringles','Kleenex',
		 'Sponge','Soap','Whiteboard Cleaner','Cup',
		 'Glass','Candle','Reading Glasses']

		# Dining_table onwards are not movable furniture
        self.furniture = ['Trash_bin','Plant','Side_table_1','Dining_chair','Kitchen_chair','Coffee_table','Arm_Chair_1','Arm_Chair_2',
		'Dining_table','Kitchen_table','Kitchen_counter','Side_table_2','Bathroom_drawer','Closet','Bed','Bookshelf','Sofa','TV_Table']
		self.rooms = ['Entrance_hall','Hallway','Kitchen', 'Bedroom', 'Living_room', 'Dining_room', 'hall']
        self.doors = ['Door_bedroom_1','Door_bedroom_2','Door_entrance','Door_bathroom']

		#Commands no longer allowed
		self.tbm1_commands_dict = {
            "move": ["forward", "backward"],
            "turn": ["right", "left"],
            "go": self.rooms,
            "look": ["up", "down", "right", "left"],
            "here": [],
            "see": self.objects,}


        # Disable head manager
        head_mgr = NavigationCameraMgr()
        head_mgr.head_mgr_as("disable")

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




	def search_for_changes(self,subject):
		################### DOORS ###################
		door_timer = 0
		door_count = 0

		if door_count < 1:
		    # go to the next possible door location

		    # compare expected sensor output to actual output

		    # if expected is not same as actual sensor output door_count = door_count +1

			    # record the location of door
				if location = Door_bedroom_1
					thing_id = 'Door_bedroom_1'
					room1 = 'bedroom'
					room2 = 'living_room'
				if location = Door_bedroom_2
					thing_id = 'Door_bedroom_2'
					room1 = 'bedroom'
					room2 = 'hall'
				if location = Door_bathroom
					thing_id = 'Door_bathroom'
					room1 = 'bathroom'
					room2 = 'kitchen'
				else:
					thing_id = 'door_entrance'
					room1 = 'hall'
					room2 = 'outside'

				# update semantic_map with door detected
				line1 = self.linewriter('type',[thing_id,'door'])
				line2 = self.linewriter('connects',[thing_id,room1,room2])
				line3 = self.linewriter('isOpen',[thing_id,open_status])
				to_write = [line1,line2,line3]

		else:
		    # go to home position


	################### OBJECTS ###################
		object_count = 0

		if object_count < 2:
		    # go to the next possible object location
			self.move_to_location([location_i])

		    # compare expected vision output to actual vision output

		    # if expected is not same as actual vision output object_count = object_count +1

		        # take photo, save to memory stick

		        # update semantic_map with object detected
				line1 = self.linewriter('type',[thing_id, item])
				line2 = self.linewriter('in',[thing_id, room])
				line3 = self.linewriter('on',[thing_id, furniture])
				line4 = self.linewriter('position ',[thing_id, position_string])
				line6 = self.linewriter('picture',[thing_id,thing_id+'.jpg'])
				to_write = [line1,line2,line3,line4,line6]



		    # elif object_count = object_count



		else:
		    # do the next thing


		################### FURNITURE ###################
		    # ask user for which furniture has changed
			self.say("Please tell me one item of furniture that has changed?") # uses the speech function in generic_controller

			#listen for answer,
			self.toggle_stt('on')
			self.stt_callback()



			#assign furniture label to the heard reply

			#ask for the room where the furniture is
			self.say("Please tell me which room the"+furniture+"has been moved to.")


			# go to the room
			self.move_to_location()

		    # compare expected vision output to actual vision output

		    # if expected is not same as actual vision output object_count = object_count +1

		    # take photo, save to memory stick



		################### TRASH ###################

		# scan floor for trash (i.e. small object detected on the floor)

		# if object detected on the floor

		   # update semantic_map



################### SEMANTIC_MAP ###################
    def linewriter(self, descriptor, options_list):
        text = descriptor + '('
        for option in options_list:
            text = text+option+', '
        text = text[:-2]+').'
        return(text)


###################### INIT ########################
if __name__ == '__main__':
    rospy.init_node('know_my_home', anonymous=True)
    rospy.loginfo("know my home controller has started")
    controller = ControllerTBM1()
    rospy.spin()
