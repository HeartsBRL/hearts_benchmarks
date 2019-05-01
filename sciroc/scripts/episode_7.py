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
from math import cos, sin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rospy.rostime import Duration
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

        self.outfolder = rospy.get_param('output_path')


        # init vars




        # disable head manager
        head_mgr = NavigationCameraMgr()
        head_mgr.head_mgr_as("disable")



    def run(self):

        self.say("I am ready to serve some goodies")
        
        
        ### move from preparation area to start location 
        
        
        ### request data           ### hub communication protocol tba 
        
        
        ### receive data from hub  ### hub communication protocol tba 
        
        
        ### leave start area
        
        
        ### choose order
        
        
        ### annouce order
        
        
        ### go to shelf, adjust height
        
        
        
        ### locate object
        
        
        
        ### annouce
        
                
        ### grab object
        
        
        
        ### announce
        
        
        
        ### move to delivery area
        
        
        
        ### announce
        
        
        
        ### deposit object
        
        
        
        ### announce
        
        
        
        ### if not last order and X amount of time left, go back to choose order step  
        
        
        
        ### go to finish area
        
        
        ### announce end of episode  
        
        ### send info to hub           ### hub communication protocol tba 








###################### INIT ########################
if __name__ == '__main__':
    rospy.init_node('task_controller', anonymous=True)
    rospy.loginfo("episode 7 controller has started")
    newcontroller = ControllerTBM1()
    #newcontroller.move_to_location("home", 3)
    newcontroller.run()
    rospy.spin()
