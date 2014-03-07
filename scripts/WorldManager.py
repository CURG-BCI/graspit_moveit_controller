#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from extended_planning_scene_interface import ExtendedPlanningSceneInterface
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg

#additional imports for the service node
import roslib; roslib.load_manifest('moveit_trajectory_planner')
from moveit_trajectory_planner.srv import *


class WorldManager:

  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.scene = ExtendedPlanningSceneInterface()
    #moveit_commander.PlanningSceneInterface()

    #init the server node
    s1 = rospy.Service('moveit_trajectory_planner/add_box', BoxInfo, self.handle_add_box)
    s2 = rospy.Service('moveit_trajectory_planner/add_autoscaled_mesh', MeshInfo, self.handle_add_autoscaled_mesh)
    s3 = rospy.Service('moveit_trajectory_planner/remove_object', ObjectName, self.handle_remove_object)

  def handle_add_box(self, req):
    box_dimensions = (req.sizeX,req.sizeY,req.sizeZ);
    self.scene.add_box(req.name, req.pose, box_dimensions)
    return BoxInfoResponse()

  def handle_add_autoscaled_mesh(self, req):
    """
    Adds a mesh, but makes sure that the mesh is scaled to meters if given a mesh that 
    is in millimeters. 
    """
    self.scene.add_mesh_autoscaled(req.name, req.pose, req.filename)
    return MeshInfoResponse()

  def handle_remove_object(self, req):
    self.scene.remove_world_object(req.name)
    return ObjectNameResponse()
