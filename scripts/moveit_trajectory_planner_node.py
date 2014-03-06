#!/usr/bin/env python

import time
import rospy
from WorldManager import WorldManager
from ReachabilityChecker import ReachabilityChecker
from MovementExecutor import MovementExecutor

#HAO = False
#def Hao():
#	import hao
#	if HAO:
#		hao = HAO()
#		return False


if __name__ == '__main__':
	time.sleep(10) #wait until moveit is loaded
	rospy.init_node('moveit_trajectory_planner')

	#start up WorldManager
	wm = WorldManager()

	#start up ReachabilityChecker
	rc = ReachabilityChecker()

	#start up MovementExecutor
	me = MovementExecutor()

	#SPIN!
	print ""
	print "========== Trajectory Planner is running"
	print ""
	rospy.spin()