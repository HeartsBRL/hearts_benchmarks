#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, String
import std_srvs.srv
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
from roah_rsbb_comm_ros.srv import Percentage


# def location_pose_2D(req):
# 	return std_srvs.srv.EmptyResponse()

# def callback_1(req):
# 	print("\n***** Service Action: RIGHT LIGHT BEDROOM switch_1 on\n")
# 	rc = std_srvs.srv.EmptyResponse()
# 	return rc

# def callback_2(req):
# 	print("\n***** Service Action: RIGHT LIGHT BEDROOM switch_1 off\n")
# 	rc = std_srvs.srv.EmptyResponse()
# 	return rc	

# def callback_3(req):
# 	print("\n***** Service Action: LEFT LIGHT BEDROOM switch_2 on\n")
# 	rc = std_srvs.srv.EmptyResponse()
# 	return rc	

# def callback_4(req):
# 	print("\n***** Service Action: LEFT LIGHT BEDROOM switch_2 off\n")
# 	rc = std_srvs.srv.EmptyResponse()
# 	return rc	

# def callback_5(req):
# 	print("\n***** Service Action: BOTH LIGHTs BEDROOM switch_3 on\n")
# 	rc = std_srvs.srv.EmptyResponse()
# 	return rc	

# def callback_6(req):
# 	print("\n***** Service Action: BOTH LIGHTs BEDROOM switch_3 off\n")
# 	rc = std_srvs.srv.EmptyResponse()
# 	return rc	

# def callback_7(req):
# 	print("\n***** Service Action: CLOSE BLINDS ma\n")
# 	rc = std_srvs.srv.EmptyResponse()
# 	return rc	

# def callback_8(req):
# 	print("\n***** Service Action: OPEN BLINDS max\n")
# 	rc = std_srvs.srv.EmptyResponse()
# 	return rc		

# def callback_9(req):
# 	print("\n***** Service Action: OPEN BLINDS set "+str(req)+"\n")
# 	#rc = std_srvs.srv.EmptyResponse() ## this dosn't work!! need to return empty list to avoid error msgs.
# 	rc =[]
# 	return rc																																																																																																																																																																																																																																																																																																																																								


# def callback_10(req):
#     print("\n***** Service Action: Dimmer set "+str(req)+"\n")
#     #rc = std_srvs.srv.EmptyResponse() ## this dosn't work!! need to return empty list to avoid error msgs.
#     rc =[]
#     return rc           

def callback_11(req):
	print("\n***** Service Action: TABLET MAP"+str(req)+"\n")
	rc = std_srvs.srv.EmptyResponse()
	return rc           


def service():
	rospy.init_node('a_node')
	# s  = rospy.Service('/roah_rsbb/tablet/map', std_srvs.srv.Empty, location_pose_2D)
	# s1 = rospy.Service('/roah_rsbb/devices/switch_1/on',  std_srvs.srv.Empty  , callback_1)
	# s2 = rospy.Service('/roah_rsbb/devices/switch_1/off', std_srvs.srv.Empty  , callback_2)
	# s3 = rospy.Service('/roah_rsbb/devices/switch_2/on',  std_srvs.srv.Empty  , callback_3)
	# s4 = rospy.Service('/roah_rsbb/devices/switch_2/off', std_srvs.srv.Empty  , callback_4)
	# s5 = rospy.Service('/roah_rsbb/devices/switch_3/on',  std_srvs.srv.Empty  , callback_5)
	# s6 = rospy.Service('/roah_rsbb/devices/switch_3/off', std_srvs.srv.Empty  , callback_6)	
	# s7 = rospy.Service('/roah_rsbb/devices/blinds/max',   std_srvs.srv.Empty  , callback_7)	
	# s8 = rospy.Service('/roah_rsbb/devices/blinds/min',   std_srvs.srv.Empty  , callback_8)	
	# s9 = rospy.Service('/roah_rsbb/devices/blinds/set',   Percentage          , callback_9)
	# s10 = rospy.Service('/roah_rsbb/devices/dimmer/set',  Percentage         , callback_10)  
	s11 = rospy.Service('/roah_rsbb/tablet/map',          std_srvs.srv.Empty , callback_11)   	
	print("using latest service definition")
	rospy.spin()





service()