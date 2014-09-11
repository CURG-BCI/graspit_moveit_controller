#!/usr/bin/env python
import os
import time

import roslib
import rospy
import moveit_commander

import geometry_msgs.msg
from moveit_trajectory_planner.srv import *
from std_srvs.srv import Empty

from world_manager_helpers.extended_planning_scene_interface import ExtendedPlanningSceneInterface
from world_manager_helpers.model_rec_manager import ModelManager, ModelRecManager
from world_manager_helpers.object_filename_dict import file_name_dict
import tf
import tf.transformations
roslib.load_manifest('moveit_trajectory_planner')
import ipdb

class WorldManager:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = ExtendedPlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        #model_rec_manager for all the objects in the enviornment
        self.model_manager = ModelRecManager()
        #a cache of all of the object names in the enviorment, for use with remove_all_objects
        self.body_name_cache = []

        self.add_box_server = rospy.Service('moveit_trajectory_planner/add_box',
                                            BoxInfo,
                                            self.handle_add_box)

        self.add_autoscaled_mesh_server = rospy.Service('moveit_trajectory_planner/add_autoscaled_mesh',
                                                        MeshInfo,
                                                        self.handle_add_autoscaled_mesh)

        self.remove_object_server = rospy.Service('moveit_trajectory_planner/remove_object',
                                                  ObjectName,
                                                  self.handle_remove_object)

        self.refresh_model_list_server = rospy.Service('model_manager/refresh_model_list',
                                                       Empty,
                                                       self.refresh_model_list)

        self.reload_model_list_server = rospy.Service('model_manager/reload_model_list',
                                                      Empty,
                                                      self.reload_model_list)

    def handle_add_box(self, req):
        box_dimensions = (req.sizeX, req.sizeY, req.sizeZ)
        self.scene.add_box(req.name, req.pose, box_dimensions)
        return BoxInfoResponse()

    def handle_add_autoscaled_mesh(self, req):
        """
        Adds a mesh, but makes sure that the mesh is scaled to meters if given a mesh that 
        is in millimeters. 
        """
        if os.path.isfile(req.filename):
            self.scene.add_mesh_autoscaled(req.name, req.pose, req.filename)
            rospy.loginfo(self.__class__.__name__ + '::handle_add_autoscaled_mesh::name -- %s filename -- %s'%(req.name, req.filename) )
        else:
            rospy.logwarn('File doesn\'t exist - object %s, filename %s'%(req.name, req.filename))

        return MeshInfoResponse()

    def handle_remove_object(self, req):
        self.scene.remove_world_object(req.name)
        return ObjectNameResponse()

    def refresh_model_list(self, empty_msg):                
        self.model_manager.refresh()

        #self.remove_object_publisher.publish('ALL') #commented lines for graspit
        #self.publish_table_models()
        self.remove_all_objects_from_planner()
        self.add_all_objects_to_planner()
        #need to return [] for empty response
        return []

    def reload_model_list(self, empty_msg):
        self.model_manager.read()

        #self.remove_object_publisher.publish('ALL') #commented lines for graspit
        #self.publish_table_models()
        self.remove_all_objects_from_planner()
        self.add_all_objects_to_planner()
        return []

    def get_model_list(self, empty_msg):
        self.world_manager.read()

    def remove_all_objects_from_planner(self):
        """
        @brief - Clears all models from moveit enivornment (from name cache)

        FIXME - Add additional obstacles for camera post and computers around robot
        and some way of annotating that they shouldn't be removed.
        """
        for index in range(len(self.body_name_cache)):
            body_name = self.body_name_cache[index]
            self.scene.remove_world_object(body_name)
            #del self.body_name_cache[index]
            #del body_name

    def add_all_objects_to_planner(self):
        """
        @brief - Adds all of the models in the model_rec_manager to moveit enviornment and adds names to cache
        """
        for model in self.model_manager.model_list:
            model_name = model.model_name.strip('/')
            filename = file_name_dict[model_name]
            if os.path.isfile(filename):
                stampedModelPose = geometry_msgs.msg.PoseStamped()
                stampedModelPose.header.frame_id = "/world"  #"/camera_link" #self.robot.get_planning_frame()
                rospy.loginfo(self.__class__.__name__ +
                              ':: Adding model %s -- frame_id %s -- '%(model_name, stampedModelPose.header.frame_id) +
                              ' filename %s '%(filename))


                stampedModelPose.pose = model.get_world_pose()
                self.scene.add_mesh_autoscaled(model.object_name, stampedModelPose, filename)
            else:
                rospy.logwarn('File doesn\'t exist - object %s, filename %s'%(model.object_name, filename))

            self.body_name_cache.append(model.model_name)


def add_table(world_manager):

    time.sleep(1)
    rospy.wait_for_service('moveit_trajectory_planner/add_box')
    frame_id = "/world"
    rospy.loginfo("adding table in planning frame: " + str(frame_id))
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame_id
    table_x = .92
    table_y = 1.22
    table_z = .01
    table_world_x_offset = -.68
    table_world_y_offset = -.19
    table_world_z_offset = 0
    box_pose.pose.position.x = table_x/2.0 + table_world_x_offset
    box_pose.pose.position.y = table_y/2.0 + table_world_y_offset
    box_pose.pose.position.z = table_z/2.0 + table_world_z_offset
    box_pose.pose.orientation.x = 0
    box_pose.pose.orientation.y = 0
    box_pose.pose.orientation.z = 0
    box_pose.pose.orientation.w = 0
    box_dimensions = (table_x, table_y, table_z)

    world_manager.scene.add_box("table", box_pose, box_dimensions)
    rospy.loginfo("table added")

def add_walls(world_manager):

    back_wall_pose = geometry_msgs.msg.PoseStamped()
    back_wall_pose.header.frame_id = '/staubli_rx60l_link1'
    wall_dimensions =  [0.92, 1.22, 0.05]
    back_wall_pose.pose.position = geometry_msgs.msg.Point(**{'x': 0.35, 'y': -0.09, 'z': -0.22})
    back_wall_pose.pose.orientation = geometry_msgs.msg.Quaternion(**{'x': -0.7128395185,
                                                                      'y': 0.0414512120285,
                                                                      'z': 0.699774446448,
                                                                      'w': 0.0213855555098})
    left_wall_pose = geometry_msgs.msg.PoseStamped()
    left_wall_pose.header.frame_id = '/staubli_rx60l_link1'
    left_wall_dimensions =  [0.92, 1.22, 0.05]
    left_wall_pose.pose.position = geometry_msgs.msg.Point(**{'x': -0.56,
                                                              'y': -0.91,
                                                              'z': -0.09})

    left_wall_pose.pose.orientation = geometry_msgs.msg.Quaternion(**{'x': -0.482199130451,
                                                                      'y': 0.516804171535,
                                                                      'z': 0.487346836672,
                                                                      'w': 0.512728493124})
    
    right_wall_pose = geometry_msgs.msg.PoseStamped()
    right_wall_pose.header.frame_id = '/staubli_rx60l_link1'
    right_wall_dimensions =  [0.92, 1.22, 0.05]
    right_wall_pose.pose.position = geometry_msgs.msg.Point(**{'x': -0.6,
                                                              'y': 0.68,
                                                               'z': -0.31})

    right_wall_pose.pose.orientation = geometry_msgs.msg.Quaternion(**{'x': -0.50728105048,
                                                                       'y': 0.487102614313,
                                                                       'z': 0.482034934632,
                                                                       'w': 0.522531626553})
    world_manager.scene.add_box("left_wall", left_wall_pose, wall_dimensions)
    world_manager.scene.add_box("right_wall", right_wall_pose, wall_dimensions)
    world_manager.scene.add_box("back_wall", back_wall_pose, wall_dimensions)


if __name__ == '__main__':

    try:
        rospy.init_node('world_manager_node')

        world_manager = WorldManager()
        add_table(world_manager)
        add_walls(world_manager)

        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            world_manager.model_manager.rebroadcast_object_tfs()
            loop.sleep()
    except rospy.ROSInterruptException: pass
