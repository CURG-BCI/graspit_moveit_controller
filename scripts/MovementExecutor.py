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

class MovementExecutor:

  def __init__(self):
    #setting up enviornment
    moveit_commander.roscpp_initialize(sys.argv)
    self.group = moveit_commander.MoveGroupCommander("mico_arm")

    #init the server node
    s = rospy.Service('moveit_trajectory_planner/execute_movement', LocationInfo, self.handle_execute_movement)

  def handle_execute_movement(self, req):

    self.group.set_pose_target(req.targetPose)
    self.group.go(wait=True)
    return LocationInfoResponse(True)