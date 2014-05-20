"""
@package - model_rec_manager
@brief - calls model_rec and then manages the resulting models. Broadcast the pointclouds and TF

"""

import roslib; roslib.load_manifest("moveit_trajectory_planner" )
import rospy
from numpy import dot,  linalg
from numpy import mat

import tf
import tf.transformations
import tf_conversions.posemath as pm

import model_rec2, model_rec2.srv
import sensor_msgs, sensor_msgs.msg
import graspit_msgs.srv

import StringIO


class ModelManager(object):
    def __init__(self, model_name, point_cloud_data, pose):
        self.model_name = model_name
        self.object_name = model_name
        self.point_cloud_data = point_cloud_data
        self.pose = pose
        self.bc = ModelRecManager.tf_broadcaster
        self.listener = ModelRecManager.tf_listener
        
    def broadcast_tf(self):
        tf_pose = pm.toTf(pm.fromMsg(self.pose))
        self.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), self.object_name, "/camera_rgb_optical_frame")
            
    def get_dist(self):
        self.broadcast_tf()
        self.listener.waitForTransform("/world", self.object_name, rospy.Time(0), rospy.Duration(10))
        (trans, rot) = self.listener.lookupTransform("/world", self.object_name, rospy.Time(0))
        return linalg.norm(trans)
    
    def __len__(self):
        return self.get_dist()
    
    def get_world_pose(self):
        self.broadcast_tf()
        self.listener.waitForTransform("/world", self.object_name, rospy.Time(0),rospy.Duration(10))            
        return pm.toMsg(pm.fromTf(self.listener.lookupTransform("/world", self.object_name, rospy.Time(0))))

    def __getstate__(self):
        state = {}
        state['model_name'] = self.model_name
        state['object_name'] = self.object_name
        buff = StringIO.StringIO()
        self.point_cloud_data.serialize(buff)
        buff.seek(0)
        state['point_cloud_data'] = buff.readlines()
        state['pose'] = pm.toMatrix(pm.fromMsg(self.pose))
        return state

    def __setstate__(self, state):
        self.model_name = state['model_name']
        self.object_name = state['object_name']

        buff = StringIO.StringIO()
        buff.writelines(state['point_cloud_data'])
        buff.seek(0)
        self.point_cloud_data = sensor_msgs.msg.PointCloud2()
        self.point_cloud_data.deserialize(buff.buf)
        self.pose = pm.toMsg(pm.fromMatrix(state['pose']))
        self.bc = ModelRecManager.tf_broadcaster
        self.listener = ModelRecManager.tf_listener


class ModelRecManager(object):
            
    def __init__(self):
        self.__publish_target = True
        self.model_list = list()

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
            
        ModelRecManager.tf_listener = self.tf_listener
        ModelRecManager.tf_broadcaster = self.tf_broadcaster
        self.model_name_server = rospy.Service('/get_object_info', graspit_msgs.srv.GetObjectInfo, self.get_object_info)

    def refresh(self):
        find_objects_srv = rospy.ServiceProxy('/recognize_objects', model_rec2.srv.FindObjects)
        resp = find_objects_srv()
        self.model_list = list()
        for i in range(len(resp.object_name)):
            self.model_list.append(ModelManager(resp.object_name[i],
                                                resp.pointcloud[i],
                                                resp.object_pose[i]))
        self.uniquify_object_names()

        for model in self.model_list:
            model.model_name = model.model_name
            model.point_cloud_data.header.frame_id = '/' + model.object_name



    def publish_target_pointcloud(self):
        self.model_list.sort(key=ModelManager.get_dist)
        x = self.model_list[0]
        tf_pose = pm.toTf(pm.fromMsg(x.pose))
        x.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), "/object", "/camera_rgb_optical_frame")
        x.listener.waitForTransform("/world", "/object", rospy.Time(0), rospy.Duration(5))
        x.point_cloud_data.header.frame_id = "/object"
        pub = rospy.Publisher('/object_pointcloud', sensor_msgs.msg.PointCloud2)
        pub.publish(x.point_cloud_data)

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
        
    def uniquify_object_names(self):
        object_name_dict = {}
        for model in self.model_list:
            if model.object_name in object_name_dict:
                object_name_dict[model.object_name].append(model)
            else:
                object_name_dict[model.object_name] = [model]

        model_names = dict(object_name_dict)
        
        for model_list in object_name_dict.values():
            if len(model_list) > 1:
                for model_num, model in enumerate(model_list):
                    test_name = model.object_name
                    while test_name in model_names:
                        test_name = "%s_%i" % (model.object_name, model_num)
                        model_num += 1
                    model.object_name = test_name
                    model_names[test_name] = model

