#!/usr/bin/env python
import os
import time

import roslib
import rospy
import moveit_commander

import geometry_msgs.msg
from trajectory_planner_msgs.srv import *
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
import trajectory_planner_msgs.msg

import actionlib
import graspit_msgs.msg


class WorldManager:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = ExtendedPlanningSceneInterface()
        """
        :type self.robot :moveit_commander.RobotCommander
        """
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
                                                      self.return_current_model_list)
        self.planning_scene_service = rospy.ServiceProxy("/get_planning_scene", moveit_msgs.srv.GetPlanningScene)
        self.update_planning_scene_from_moveit()
        self.force_use_moveit = rospy.get_param("use_moveit", 1)

        self._run_recognition_as = actionlib.SimpleActionServer("recognize_objects_action",
                                                                graspit_msgs.msg.RunObjectRecognitionAction,
                                                                execute_cb=self._run_recognition_as_cb,
                                                                auto_start=False)
        self._run_recognition_as.start()

        
        
    def update_planning_scene_from_moveit(self):
        """ Update the current cached scene directly from moveit.
            Returns true if it the service call succeeds. False implies moveit hasn't started correctly. 
        """
        try:
            rospy.wait_for_service("/get_planning_scene", 5)
            components = PlanningSceneComponents(PlanningSceneComponents.WORLD_OBJECT_NAMES + PlanningSceneComponents.TRANSFORMS)
            """
            :type ps_response :moveit_msgs.srv.GetPlanningSceneResponse
            """
            ps_request = moveit_msgs.srv.GetPlanningSceneRequest(components=components)
            ps_response = self.planning_scene_service.call(ps_request)
            
            self.body_name_cache = [co.id for co in ps_response.scene.world.collision_objects]
            rospy.loginfo("body name cache:%s"%(', '.join(self.body_name_cache)))
            return True
        except Exception as e:

            raise e
            rospy.logerror("Failed to find service /get_planning_scene %s and force use moveit enabled"%(e))
            rospy.shutdown()
            rospy.logwarn("Failed to find service /get_planning_scene %s"%(e))
            return False
            
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

    def _run_recognition_as_cb(self, goal):
        rospy.loginfo("Received Run Recognition Goal")

        self.remove_all_objects_from_planner()
        self.model_manager.refresh()

        #self.remove_object_publisher.publish('ALL') #commented lines for graspit
        #self.publish_table_models()

        self.add_all_objects_to_planner()
        #need to return [] for empty response.
        _result = trajectory_planner_msgs.msg.RunObjectRecognitionResult()
        for model in self.model_manager.model_list:
            model_name = model.model_name
            object_name = model.object_name
            object_pose = model.get_world_pose()
            object_info = graspit_msgs.msg.ObjectInfo(object_name, model_name, object_pose)
            rospy.loginfo("Object Info: " + str(object_info))
            _result.object_info.append(object_info)

        self._run_recognition_as.set_succeeded(_result)
        return []


    def return_current_model_list(self, empty_msg):
        #self.model_manager.read()

        #self.remove_object_publisher.publish('ALL') #commented lines for graspit
        #self.publish_table_models()
        if not len(self.model_manager.model_list):
            self.refresh_model_list(empty_msg)
            return []
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
        # update the body name cache
        self.update_planning_scene_from_moveit()
        
        for index in range(len(self.body_name_cache)):
            body_name = self.body_name_cache[index]
            rospy.logwarn("removing body: body_name %s"%(body_name))
            self.scene.remove_world_object(body_name)
            #del self.body_name_cache[index]
            #del body_name
        self.body_name_cache = []

        # read the obstacles such as the table and any safety walls
        self.add_obstacles()

    def add_all_objects_to_planner(self):
        """
        @brief - Adds all of the models in the model_rec_manager to moveit enviornment and adds names to cache
        """
        # Clear moveit's model list
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
                
                #self.scene.remove_world_object(model.object_name)
            else:
                rospy.logwarn('File doesn\'t exist - object %s, filename %s'%(model.object_name, filename))


    def add_table(self):
        
        time.sleep(1)
        rospy.wait_for_service('moveit_trajectory_planner/add_box')
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


    def add_mico_base(self):
        
        time.sleep(1)
        rospy.wait_for_service('moveit_trajectory_planner/add_box')
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
        back_wall_pose.header.frame_id = '/staubli_rx60l_link1'
        wall_dimensions = [0.92, 1.22, 0.05]
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
        self.scene.add_box("left_wall", left_wall_pose, wall_dimensions)
        self.scene.add_box("right_wall", right_wall_pose, wall_dimensions)
        self.scene.add_box("back_wall", back_wall_pose, wall_dimensions)


    def add_obstacles(self):
        self.add_table()
        self.add_walls()
        #self.add_mico_base()
        
if __name__ == '__main__':

    try:
        rospy.init_node('world_manager_node')

        world_manager = WorldManager()
        world_manager.add_obstacles()

        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            world_manager.model_manager.rebroadcast_object_tfs()
            loop.sleep()
    except rospy.ROSInterruptException: pass
