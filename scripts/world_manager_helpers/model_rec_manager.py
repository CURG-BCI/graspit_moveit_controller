"""
@package - model_rec_manager
@brief - calls model_rec and then manages the resulting models. Broadcast the pointclouds and TF
"""

import roslib

import rospy
from numpy import linalg

import tf
import tf.transformations
import tf_conversions.posemath as pm
import numpy as np

import block_recognition
import block_recognition.srv
import graspit_msgs.srv

roslib.load_manifest("moveit_trajectory_planner")


class ModelManager(object):
    def __init__(self, detected_block, tf_broadcaster, tf_listener):
        self.model_name = detected_block.mesh_filename
        self.object_name = detected_block.unique_block_name

        self.pose = detected_block.pose_stamped.pose

        self.detected_block = detected_block

        self.bc = tf_broadcaster
        self.listener = tf_listener
        self.detected_frame = "/kinect2_rgb_optical_frame"

    def broadcast_tf(self):
        tf_pose = pm.toTf(pm.fromMsg(self.pose))

        self.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), self.object_name, "/kinect2_rgb_optical_frame")

    def get_dist(self):
        self.broadcast_tf()
        self.listener.waitForTransform(self.detected_frame, self.object_name, rospy.Time(0), rospy.Duration(10))
        (trans, rot) = self.listener.lookupTransform(self.detected_frame, self.object_name, rospy.Time(0))
        return linalg.norm(trans)

    def __len__(self):
        return self.get_dist()

    def get_world_pose(self):
        self.broadcast_tf()
        self.listener.waitForTransform("/world", self.object_name, rospy.Time(0),rospy.Duration(10))
        return pm.toMsg(pm.fromTf(self.listener.lookupTransform("/world", self.object_name, rospy.Time(0))))


class ModelRecManager(object):

    def __init__(self):
        self.__publish_target = True
        self.model_list = list()

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.model_name_server = rospy.Service('/get_object_info', graspit_msgs.srv.GetObjectInfo, self.get_object_info)

    def refresh(self):
        # clear out old models
        self.model_list = list()

        find_objects_srv = rospy.ServiceProxy('/objrec_node/find_blocks', block_recognition.srv.FindObjects)

        resp = find_objects_srv()

        for detected_block in resp.detected_blocks:
            rospy.loginfo("Adding ModelManager for object {} with unique_name: {}".format(detected_block.mesh_filename, detected_block.unique_block_name))

            self.model_list.append(ModelManager(detected_block, self.tf_broadcaster, self.tf_listener))

    def rebroadcast_object_tfs(self):
        for model in self.model_list:
            model.broadcast_tf()

    def get_model_names(self):
        return [model.model_name for model in self.model_list]

    def get_object_info(self, req):
        resp = graspit_msgs.srv.GetObjectInfoResponse()
        for model in self.model_list:
            model_name = model.model_name
            object_name = model.object_name
            object_pose = model.get_world_pose()
            object_info = graspit_msgs.msg.ObjectInfo(object_name, model_name, object_pose)
            resp.object_info.append(object_info)
        return resp

