#!/usr/bin/env python
import os
import time
import sys

import roslib
import rospy
import moveit_commander

import geometry_msgs.msg
from graspit_msgs.srv import *
from std_srvs.srv import Empty

from world_manager_helpers.extended_planning_scene_interface import ExtendedPlanningSceneInterface
from world_manager_helpers.model_rec_manager import ModelManager, ModelRecManager
from world_manager_helpers.object_filename_dict import file_name_dict
import tf
import tf.transformations

roslib.load_manifest('moveit_trajectory_planner')
import ipdb
import moveit_msgs
import moveit_msgs.srv
from moveit_msgs.msg import PlanningSceneComponents
import actionlib
import graspit_msgs.msg
import rospy

from moveit_trajectory_planner.srv import *


class WorldManager:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.NEW_MODEL_REC = True
        self.scene = ExtendedPlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        self.model_manager = ModelRecManager(self.NEW_MODEL_REC)

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

        experiment_type = rospy.get_param('/experiment_type')
        print("experiment type: " + experiment_type)
        if not experiment_type == "block":
            print("about to filter_objects_from_model_list()")
            self.filter_objects_from_model_list()
            print("finished filter_objects_from_model_list()")

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

    def filter_objects_from_model_list(self):
        new_model_list = []
        for model in self.model_manager.model_list:
            model.detected_frame = "/camera_depth_optical_frame"
            pose = model.get_world_pose()

            x_filter_limit_min = -0.5
            x_filter_limit_max = -0.1

            y_filter_limit_min = 0.0
            y_filter_limit_max = 0.32

            if (x_filter_limit_min <= pose.position.x <= x_filter_limit_max) and ( y_filter_limit_min <= pose.position.y <= y_filter_limit_max):
                new_model_list.append(model)

        self.model_manager.model_list = new_model_list


    def add_all_objects_to_planner(self):
        self.add_obstacles()
        for model in self.model_manager.model_list:
            model_name = model.model_name.strip('/')
            print "Adding " + str(model_name) + "To Moveit"
            filename = file_name_dict[model_name]
            if os.path.isfile(filename):

                stamped_model_pose = geometry_msgs.msg.PoseStamped()
                stamped_model_pose.header.frame_id = "/world"
                stamped_model_pose.pose = model.get_world_pose()

                try:
                    self.scene.add_mesh_autoscaled(model.object_name, stamped_model_pose, filename)
                except:
                    continue

            else:
                rospy.logwarn('File doesn\'t exist - object %s, filename %s' % (model.object_name, filename))

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

        self.scene.attach_box(world_manager.robot.get_link_names()[0], "table", box_pose, box_dimensions)
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

        self.scene.attach_box(world_manager.robot.get_link_names()[1], "robot_base", box_pose, box_dimensions)
        rospy.loginfo("base added")

    def add_walls(self):

        back_wall_pose = geometry_msgs.msg.PoseStamped()
        back_wall_pose.header.frame_id = '/world'
        wall_dimensions = [1.45, .05, 1.0]
        back_wall_pose.pose.position = geometry_msgs.msg.Point(**{'x': -0.1, 'y': -0.3, 'z': 0.1})
        back_wall_pose.pose.orientation = geometry_msgs.msg.Quaternion(**{'x': 0,
                                                                          'y': 0,
                                                                          'z': 0,
                                                                          'w': 0})

        self.scene.add_box("back_wall", back_wall_pose, wall_dimensions)

    def add_bin_wall(self, w, d, h, x, y, z, wall_name):
        time.sleep(0.5)
        bin_pose = geometry_msgs.msg.PoseStamped()
        bin_pose.header.frame_id = '/root'
        bin_dimensions = [w, d, h]
        bin_pose.pose.position = geometry_msgs.msg.Point(**{'x': x, 'y': y, 'z': z})
        bin_pose.pose.orientation = geometry_msgs.msg.Quaternion(**{'x': 0,
                                                                         'y': 0,
                                                                         'z': 0,
                                                                         'w': 0})
        self.scene.add_box(wall_name, bin_pose, bin_dimensions)

    def add_bin(self):

        # back_bin_pose = geometry_msgs.msg.PoseStamped()
        # back_bin_pose.header.frame_id = '/world'
        # back_bin_dimensions = [0.5, .01, .2]
        # back_bin_pose.pose.position = geometry_msgs.msg.Point(**{'x': .62, 'y': -0.29, 'z': 0.16})
        # back_bin_pose.pose.orientation = geometry_msgs.msg.Quaternion(**{'x': 0,
        #                                                                  'y': 0,
        #                                                                  'z': 0,
        #                                                                  'w': 0})
        # right_bin_pose = geometry_msgs.msg.PoseStamped()
        # right_bin_pose.header.frame_id = '/world'
        # right_bin_dimensions = [0.01, .25, .2]
        # right_bin_pose.pose.position = geometry_msgs.msg.Point(**{'x': .36, 'y': -0.415, 'z': 0.16})
        # right_bin_pose.pose.orientation = geometry_msgs.msg.Quaternion(**{'x': 0,
        #                                                                   'y': 0,
        #                                                                   'z': 0,
        #                                                                   'w': 0})

        # self.scene.add_box("back_bin_wall", back_bin_pose, back_bin_dimensions)
        # self.scene.add_box("right_bin_wall", right_bin_pose, right_bin_dimensions)
        h = 0.085
        z = h/2
        l_1 = 0.537
        l_2 = 0.254
        thickness = 0.01
        x = -0.2
        y = -l_1 / 2

        #assuming these are meters

        self.add_bin_wall(thickness, l_2, h,
                         y, x - l_2/2, z, "left_bin_wall")
        self.add_bin_wall(thickness, l_2, h,
                          y + l_1, x - l_2/2, z, "right_bin_wall")
        self.add_bin_wall(l_1, thickness, h,
                          y + l_1/2, x - l_2 - thickness, z, "top_bin_wall")
        self.add_bin_wall(l_1, thickness, h,
                          y + l_1/2, x + thickness, z, "bottom_bin_wall")

        self.add_bin_wall(thickness, 0.254, 0.152,
                         y + l_1/2,  x - l_2/2, 0.076, "center_bin_wall")
        rospy.sleep(2.)

    def add_obstacles(self):
        self.add_table()
        experiment_type = rospy.get_param('/experiment_type')
        # TO ADD WALLS FOR BOX & BLOCKS EXPERIMENT:
        # if experiment_type == "block":
        #     self.add_bin()
        # self.add_bin()
        self.add_walls()
        self.add_base()


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
