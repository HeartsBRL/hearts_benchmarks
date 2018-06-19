#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String, UInt8
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState
from geometry_msgs.msg import Pose2D
import std_srvs.srv


class Comms():

	def __init__(self):

		self.currentGoal = 0
		self.lastReached = 0

		rospy.Subscriber('/roah_rsbb/goal', Pose2D, self.WaypointCallback)
		rospy.Subscriber('/roah_rsbb/benchmark/state', BenchmarkState, self.BenchStatusCallback)

		rospy.Subscriber('/hearts/navigator/status', String, self.WaypointReached)

		self.pubGoal = rospy.Publisher('hearts/navigator/goal', Pose2D, queue_size=1)
		self.pubReachedWaypoint = rospy.Publisher('/roah_rsbb/reached_waypoint', UInt8, queue_size=1)

		self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
		self.execute = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty)

	def BenchStatusCallback(self, data):

		rospy.loginfo("fbm2_controller: BenchStatusCallback")

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

	def WaypointCallback(self, data):

		rospy.loginfo("fbm2_controller: WaypointCallback")
		rospy.loginfo(data)

		self.currentGoal = self.currentGoal + 1
		rospy.loginfo("fbm2_controller: currentGoal=" + str(self.currentGoal))
		#self.prepare()
		rospy.loginfo("fbm2_controller: publishing goal")
		self.pubGoal.publish(data)

	def WaypointReached(self, data):

		if(self.lastReached == -1):
			rospy.loginfo("fbm2_controller: not atempting to go goal, ignoring")

		else:
			if(data.data[:8] == "Reached:"):
				self.lastReached = self.lastReached + 1
				rospy.loginfo("fbm2_controller: lastReached=" + str(self.lastReached))
				self.pubReachedWaypoint.publish(self.lastReached)
				try:
			 		self.execute()
				except:
					rospy.loginfo("Failed to reply EXECUTE")

			if(data.data[:7] == "Failed:"):
				self.lastReached = self.lastReached + 1
				rospy.loginfo("fbm2_controller: failed to get to: lastReached=" + str(self.lastReached))
				self.pubReachedWaypoint.publish(self.lastReached)
				try:
			 		self.execute()
				except:
					rospy.loginfo("Failed to reply EXECUTE")				

if __name__ == '__main__':
	rospy.init_node('fbm2_controller', anonymous=True)
	rospy.loginfo("fbm2_controller: Started")
	c = Comms()
	rospy.spin()
