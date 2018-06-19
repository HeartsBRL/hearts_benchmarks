#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String, UInt8
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState
from geometry_msgs.msg import Pose2D
import std_srvs.srv


class Comms():

	def __init__(self):

		rospy.Subscriber('/roah_rsbb/goal', Pose2D, self.WaypointCallback)
		rospy.Subscriber('/roah_rsbb/benchmark/state', BenchmarkState, self.BenchStatusCallback)
		rospy.Subscriber('/hearts/controller/task', String, self.TaskCallback)

		self.pubGoal = rospy.Publisher('/roah_rsbb/reached_waypoint', UInt8, queue_size=1)

		self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
		self.execute = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty) 
		
	def TaskCallback(self, data):
		if data.data == "execute":
			try:
				self.execute()
			except:
				rospy.loginfo("Service unavail")
		elif data.data == "prepare":
			try:
				self.prepare()
				self.pubGoal.publish(0)
				time.sleep(2)
				self.pubGoal.publish(1)
				time.sleep(2)
				self.pubGoal.publish(2)
			except:
				rospy.loginfo("Service unavail")

	def BenchStatusCallback(self, data):

		rospy.loginfo("ROAH_COMMS: BenchStatusCallback")

		if data.benchmark_state == BenchmarkState.STOP:
			rospy.loginfo("STOP")
		elif data.benchmark_state == BenchmarkState.PREPARE:
			rospy.loginfo("PREPARE")
		elif data.benchmark_state == BenchmarkState.EXECUTE:
			rospy.loginfo("EXECUTE")

	def WaypointCallback(self, data):

		rospy.loginfo("ROAH_COMMS: WaypointCallback")
		rospy.loginfo(data)


if __name__ == '__main__':
	rospy.init_node('hearts_roah_comms', anonymous=True)
	rospy.loginfo("ROAH_COMMS: Started")
	c = Comms()
	rospy.spin()

