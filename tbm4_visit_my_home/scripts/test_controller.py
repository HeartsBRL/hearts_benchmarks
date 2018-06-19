#!/usr/bin/env python
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
from pal_navigation_msgs.srv import Acknowledgment
#from navigation_camera_mgr_example import NavigationCameraMgr
#/pal_map_manager/change_map "input: 'my_office_map'"
change_map = rospy.ServiceProxy('/pal_map_manager/change_map', Acknowledgment)
ack = change_map("map_1")
print(ack)

