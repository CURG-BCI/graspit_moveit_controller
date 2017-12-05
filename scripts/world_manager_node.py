#!/usr/bin/env python
import os
import time
import moveit_commander

import geometry_msgs.msg
from graspit_msgs.srv import *

from world_manager_helpers.extended_planning_scene_interface import ExtendedPlanningSceneInterface
from world_manager_helpers.model_rec_manager import ModelRecManager
from copy import deepcopy

import moveit_msgs
import moveit_msgs.srv
from moveit_msgs.msg import PlanningSceneComponents
import actionlib
import graspit_msgs.msg
import rospy
import roslib

from moveit_trajectory_planner.srv import *

roslib.load_manifest('moveit_trajectory_planner')

class WorldManager:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.scene = ExtendedPlanningSceneInterface()

        self.model_manager = ModelRecManager()

        self.planning_scene_service_proxy = rospy.ServiceProxy("/get_planning_scene", moveit_msgs.srv.GetPlanningScene)

        self._run_recognition_as = actionlib.SimpleActionServer("recognize_objects_action",
                                                                graspit_msgs.msg.RunObjectRecognitionAction,
                                                                execute_cb=self._run_recognition_as_cb,
                                                                auto_start=False)
        self._run_recognition_as.start()
        rospy.loginfo("World Manager Node is Up and Running")

    def _run_recognition_as_cb(self, goal):
        print("_run_recognition_as_cb")

        print("about to remove_all_objects_from_planner()")
        self.remove_all_objects_from_planner()
        print("finished remove_all_objects_from_planner()")

        self.model_manager.refresh()

        print("about to add_all_objects_to_planner()")
        self.add_all_objects_to_planner()
        print("finished add_all_objects_to_planner()")

        _result = graspit_msgs.msg.RunObjectRecognitionResult()
        print("graspit_msgs.msg.RunObjectRecognitionResult()")

        for model in self.model_manager.model_list:
            print("frame: " + model.detected_frame)
            object_info = graspit_msgs.msg.ObjectInfo(model.object_name, model.model_name, model.get_world_pose())
            _result.object_info.append(object_info)
        print("finished for loop")

        self._run_recognition_as.set_succeeded(_result)
        return []

    def get_body_names_from_planner(self):
        rospy.wait_for_service("/get_planning_scene", 5)
        components = PlanningSceneComponents(
            PlanningSceneComponents.WORLD_OBJECT_NAMES + PlanningSceneComponents.TRANSFORMS)

        ps_request = moveit_msgs.srv.GetPlanningSceneRequest(components=components)
        ps_response = self.planning_scene_service_proxy.call(ps_request)

        body_names = [co.id for co in ps_response.scene.world.collision_objects]

        return body_names

    def remove_all_objects_from_planner(self):

        body_names = self.get_body_names_from_planner()

        while len(body_names) > 0:
            print("removing bodies from the planner, this can potentially take several tries")
            for body_name in body_names:
                self.scene.remove_world_object(body_name)

            body_names = self.get_body_names_from_planner()

    def add_all_objects_to_planner(self):
        self.add_obstacles()
        for model in self.model_manager.model_list:

            moveit_block_pose = deepcopy(model.detected_block.pose_stamped)

            block_dimensions = (model.detected_block.edge_length, model.detected_block.edge_length, model.detected_block.edge_length)

            self.scene.add_box(model.detected_block.unique_block_name, moveit_block_pose, block_dimensions)

    def add_table(self):

        frame_id = "/root"
        rospy.loginfo("adding table in planning frame: " + str(frame_id))
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = frame_id
        table_x = rospy.get_param('/table_x', 1.45)
        table_y = rospy.get_param('/table_y', .74)
        table_z = rospy.get_param('/table_z', .05)
        table_world_x_offset = rospy.get_param('/table_world_x_offset', -.05)
        table_world_y_offset = rospy.get_param('/table_world_y_offset', -.05)
        table_world_z_offset = rospy.get_param('/table_world_z_offset', -.0525)
        box_pose.pose.position.x = table_world_x_offset
        box_pose.pose.position.y = table_world_y_offset
        box_pose.pose.position.z = table_world_z_offset
        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.y = 0
        box_pose.pose.orientation.z = 0
        box_pose.pose.orientation.w = 0
        box_dimensions = (table_x, table_y, table_z)

        self.scene.add_box("table", box_pose, box_dimensions)
        rospy.loginfo("table added")

    def add_base(self):

        time.sleep(1)

        frame_id = "/root"
        rospy.loginfo("adding table in planning frame: " + str(frame_id))
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = frame_id
        base_x = rospy.get_param('/base_x', 0.2)
        base_y = rospy.get_param('/base_y', 0.15)
        base_z = rospy.get_param('/base_z', 0.01)
        base_robot_x_offset = rospy.get_param('/base_robot_x_offset', 0)
        base_robot_y_offset = rospy.get_param('/base_robot_y_offset', 0)
        base_robot_z_offset = rospy.get_param('/base_robot_z_offset', -0.02)
        box_pose.pose.position.x = base_robot_x_offset
        box_pose.pose.position.y = base_robot_y_offset
        box_pose.pose.position.z = base_robot_z_offset
        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.y = 0
        box_pose.pose.orientation.z = 0
        box_pose.pose.orientation.w = 0
        box_dimensions = (base_x, base_y, base_z)

        self.scene.add_box("robot_base", box_pose, box_dimensions)
        rospy.loginfo("base added")

    def add_obstacles(self):
        # self.add_table()
        # self.add_base()
        pass


if __name__ == '__main__':

    try:
        rospy.init_node('world_manager_node')

        world_manager = WorldManager()
        world_manager.add_obstacles()
        loop = rospy.Rate(30)
        while not rospy.is_shutdown():
            world_manager.model_manager.rebroadcast_object_tfs()
            loop.sleep()
    except rospy.ROSInterruptException:
        pass
