#!/usr/bin/env python

import time
import rospy
from WorldManager import WorldManager
from ReachabilityChecker import ReachabilityChecker
from MovementExecutor import MovementExecutor


if __name__ == '__main__':
	moveGroupName = rospy.get_param('moveGroupName')

	#time.sleep(10) #wait until moveit is loaded
	rospy.init_node('moveit_trajectory_planner')

	#start up WorldManager
	wm = WorldManager()

	#start up ReachabilityChecker
	rc = ReachabilityChecker(moveGroupName)

	#start up MovementExecutor
	me = MovementExecutor(moveGroupName)

	#SPIN!
	print ""
	print "========== Trajectory Planner is running"
	print ""
	rospy.spin()