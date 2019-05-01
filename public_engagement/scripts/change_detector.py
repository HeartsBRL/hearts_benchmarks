#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan
import numpy as np

rospy.init_node("change_detector", anonymous=True)

##Publishers
p = rospy.Publisher('/scan_change', String, queue_size = 10)

#Subscribers

previous_scan = []
threshold = 20
#store last laser scan - ranges only?
def call_back(data):
	global previous_scan
	global current_scan
	global threshold
	current_scan = data.ranges
	total = 0
	if previous_scan == []:
		previous_scan = current_scan
		return()
	else:
		for i in range(len(current_scan)):
		    if not np.isnan(current_scan[i]) and not np.isinf(current_scan[i]) and not np.isnan(previous_scan[i]) and not np.isinf(previous_scan[i]):
    			d = abs(current_scan[i]-previous_scan[i])
	    		total = total+d
	#rospy.loginfo(total)
	if total>threshold:
		p.publish('yes')
	else:
		p.publish('no')
	previous_scan = current_scan
	return()
rospy.Subscriber('/scan', LaserScan, call_back )


rospy.spin()




#take next laser scan 

#measures difference between s1 and s2

#if difference is above threshold value

#change detected


#publish to topics /detect change

