#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg

#additional imports for the service node
import roslib; roslib.load_manifest('moveit_trajectory_planner')
from moveit_trajectory_planner.srv import *

class ReachabilityChecker:

  def __init__(self):
    #setting up enviornment
    moveit_commander.roscpp_initialize(sys.argv)
    self.group = moveit_commander.MoveGroupCommander("mico_arm")

    #init the server node
    s = rospy.Service('moveit_trajectory_planner/check_reachability', LocationInfo, self.handle_reachability_callback)

  def handle_reachability_callback(self, req):

    self.group.set_pose_target(req.targetPose)
    plan = self.group.plan()

    if not plan.joint_trajectory.joint_names:
      return LocationInfoResponse(False) #this means the list is empty and therefore there is no path
    else:
      return LocationInfoResponse(True)