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


class WorldManager:

  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.scene = moveit_commander.PlanningSceneInterface()

    #init the server node
    s1 = rospy.Service('moveit_trajectory_planner/add_box', BoxInfo, self.handle_add_box)
    s2 = rospy.Service('moveit_trajectory_planner/remove_box', BoxName, self.handle_remove_box)

  def handle_add_box(self, req):
    box_dimensions = (req.sizeX,req.sizeY,req.sizeZ);
    self.scene.add_box(req.name, req.pose, box_dimensions)
    return BoxInfoResponse()

  def handle_remove_box(self, req):
    self.scene.remove_world_object(req.name)
    return BoxNameResponse()
