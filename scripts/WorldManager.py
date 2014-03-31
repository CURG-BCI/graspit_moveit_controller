#!/usr/bin/env python
import os
import sys
import copy
import rospy
import moveit_commander
from extended_planning_scene_interface import ExtendedPlanningSceneInterface
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from model_rec_manager import *
from object_filename_dict import file_name_dict #dictionary for all the mesh filenames


#additional imports for the service node
import roslib; roslib.load_manifest('moveit_trajectory_planner')
from moveit_trajectory_planner.srv import *
from std_srvs.srv import Empty

class WorldManager:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = ExtendedPlanningSceneInterface()

        #init the server node
        s1 = rospy.Service('moveit_trajectory_planner/add_box', BoxInfo, self.handle_add_box)
        s2 = rospy.Service('moveit_trajectory_planner/add_autoscaled_mesh', MeshInfo, self.handle_add_autoscaled_mesh)
        s3 = rospy.Service('moveit_trajectory_planner/remove_object', ObjectName, self.handle_remove_object)

        #model_rec_manager for all the objects in the enviornment
        self.model_manager = ModelRecManager()
        self.body_name_cache = [] #a cache of all of the object names in the enviorment, for use with remove_all_objects

        s4 = rospy.Service('model_manager/refresh_model_list', Empty, self.refresh_model_list)
        s5 = rospy.Service('model_manager/reload_model_list', Empty, self.reload_model_list)

    def handle_add_box(self, req):
        box_dimensions = (req.sizeX,req.sizeY,req.sizeZ);
        self.scene.add_box(req.name, req.pose, box_dimensions)
        return BoxInfoResponse()

    def handle_add_autoscaled_mesh(self, req):
        """
        Adds a mesh, but makes sure that the mesh is scaled to meters if given a mesh that 
        is in millimeters. 
        """
        if(os.path.isfile(req.filename)):
            self.scene.add_mesh_autoscaled(req.name, req.pose, req.filename)
        else:
            warn('File doesn\'t exist - object %s, filename %s'%(object_name, filename))
        return MeshInfoResponse()

    def handle_remove_object(self, req):
        self.scene.remove_world_object(req.name)
        return ObjectNameResponse()

    ###########################################################################
    #new stuff - from GraspExecutor()
    def refresh_model_list(self, empty_msg):                
        self.model_manager.refresh()
        self.model_manager()

        #self.remove_object_publisher.publish('ALL') #commented lines for graspit
        #self.publish_table_models()
        self.remove_all_objects_from_planner()
        self.add_all_objects_to_planner()

    def reload_model_list(self, empty_msg):
        self.model_manager.read()
        self.model_manager()

        #self.remove_object_publisher.publish('ALL') #commented lines for graspit
        #self.publish_table_models()
        self.remove_all_objects_from_planner()
        self.add_all_objects_to_planner()

    def remove_all_objects_from_planner(self):
        """
        @brief - Clears all models from moveit enivornment (from name cache)

        FIXME - Add additional obstacles for camera post and computers around robot
        and some way of annotating that they shouldn't be removed.
        """
        for body_name in self.body_name_cache[:]:
            self.scene.remove_world_object(body_name)
            del body_name_cache[index]
            del body_name

    def add_all_objects_to_planner(self):
        """
        @brief - Adds all of the models in the model_rec_manager to moveit enviornment and adds names to cache
        """
        for model in self.model_manager.model_list:
            if(os.path.isfile(file_name_dict[model.model_name.strip('/')])):
                stampedModelPose = geometry_msgs.msg.PoseStamped()
                stampedModelPose.header.frame_id = self.robot.get_planning_frame()
                stampedModelPose.pose = model.pose
                self.scene.add_mesh_autoscaled(model.object_name.strip('/'), stampedModelPose, file_name_dict[model.model_name.strip('/')])
            else:
                warn('File doesn\'t exist - object %s, filename %s'%(object_name, filename))

            self.body_name_cache.append(model.object_name.strip('/'))
