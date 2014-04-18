#!/usr/bin/env python
import moveit_commander
import moveit_msgs.msg
import convert_graspit_msg
from sklearn import *
import rospy
import random
import moveit_trajectory_planner.srv

import graspit_msgs.msg
import itertools
import tf_conversions.posemath as pm
from numpy import dot
from numpy import array, unique
from math import acos


import roslib
roslib.load_manifest('moveit_trajectory_planner')
import actionlib
import graspit_msgs.srv
import sys

class ReachabilityChecker(object):
    def __init__(self, analyze_grasp_topic="/moveit_trajectory_planner/check_reachability",
                 demo_pose_topic="/graspit/analyze_demo_pose", move_group_name='StaubliArm',
                 grasp_approach_tran_frame='wam/bhand/approach_tran'):
        """
        :type analyze_grasp_topic: string
        :param analyze_grasp_topic: topic to subscribe to for analyze grasp requests

        :type demo_pose_topic: string
        :param demo_pose_topic: topic to set service for responding to demonstration pose analysis requests

        :type move_group_name: string
        :param move_group_name

        :type grasp_approach_tran_frame: string
        :param grasp_approach_tran_frame: The frame of of the pose of the approach direction of the grasp.
        """

        self.data = set()
        self.retrain_threshold = 1
        self.model = neighbors.NearestNeighbors(n_neighbors=20, algorithm='kd_tree')
        self.max_data_len = 5000
        self.array_data = []
        self.grasp_classes = []


        self.analyze_grasp_service = rospy.Service(analyze_grasp_topic, moveit_trajectory_planner.srv.LocationInfo, self.analyze_grasp)
        self.analyze_pose_service = rospy.Service(demo_pose_topic, graspit_msgs.srv.AnalyzePose, self.analyze_demonstration_pose)

        #setting up enviornment
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(move_group_name)
        self.pick_plan_client = actionlib.SimpleActionClient('/move_group/pickup', moveit_msgs.msg.PickupAction)
        self.support_surface_ = ""
        self.planning_time_ = 2
        self.grasp_approach_tran_frame = grasp_approach_tran_frame
        rospy.loginfo(self.__class__.__name__ + " is inited")

    def construct_pickup_goal(self, graspit_grasp_msg,object_name):
        """
        :type graspit_grasp_msg: moveit_msgs.msg.Grasp
        """
        goal = moveit_msgs.msg.PickupGoal()

        goal.target_name = object_name
        goal.group_name = self.group.get_name()
        goal.end_effector = self.group.get_end_effector_link()
        goal.allowed_planning_time = self.planning_time_
        goal.support_surface_name = self.support_surface_
        goal.planner_id = ""
        goal.allow_gripper_support_collision = False
        #for now, no path constraints
        #goal.path_constraints = *path_constraints_;
        goal.possible_grasps = [graspit_grasp_msg]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.look_around = False
        goal.planning_options.replan_delay = 10.0
        goal.planning_options.planning_scene_diff = True
        #goal.planning_options.robot_state.is_diff = True

        return goal

    def handle_reachability_callback(self, graspit_grasp_msg):
        """

        :type graspit_grasp_msg: graspit_msgs.msg.Grasp
        """
        moveit_grasp_msg = convert_graspit_msg.graspit_grasp_to_moveit_grasp(graspit_grasp_msg,
                                                                             self.grasp_approach_tran_frame)

        try:
            self.pick_plan_client.wait_for_server(rospy.Duration(3))
        except Exception as e:
            rospy.logerr("ReachabilityChecker::handle_reachability_callback::Failed to reach pick action server"
                         " with err: %s"%(e.message))

        pickup_goal = self.construct_pickup_goal(moveit_grasp_msg, graspit_grasp_msg.object_name)
        self.pick_plan_client.send_goal(pickup_goal)

        success = self.pick_plan_client.wait_for_result(rospy.Duration(3))
        result = []
        if success:
            result = self.pick_plan_client.get_result()
            result = moveit_msgs.msg.PickupResult(result)
            success = result.error_code.val == result.error_code.SUCCESS

        return success, result

    def data_to_array(self):
        self.array_data = array([pm.toMatrix(pm.fromMsg(grasp_msg[0].final_grasp_pose))[:3, 3]
                                 for grasp_msg in self.data])

    def analyze_demonstration_pose(self, demo_pose):

        """

        :type demo_pose: graspit_msgs.srv.AnalyzePoseRequest

        :rtype response: graspit_msgs.srv.AnalyzePoseResponse
        """
        response = graspit_msgs.srv.AnalyzePoseResponse()

        if len(self.data) == 0:
            return 1.0

        demo_pose = pm.toMatrix(pm.fromMsg(demo_pose.pose))
        demo_position = demo_pose[:3, 3]

        distances, indices = self.model.kneighbors(demo_position)
        indices = unique(indices)
        nbrs = [t for t in itertools.compress(self.data, indices)]
        valid_nbrs = []
        for n in nbrs:
            pose = pm.toMatrix(pm.fromMsg(n[0].final_grasp_pose))
            if acos(dot(pose[:3, 2], demo_pose[:3, 2]) < .52):
                valid_nbrs.append(n)

        if len(valid_nbrs):
            response.success_probability = len([n for n in valid_nbrs if n[1] & 1])/(1.0*len(valid_nbrs))
        else:

            response.success_probability = 0

        return response

    def train_model(self, grasp, grasp_class):

        self.data.add((grasp, grasp_class))
        if len(self.data) > self.max_data_len:
            self.sparcify_data()

        if not len(self.data) % self.retrain_threshold:
            self.data_to_array()
            self.model.fit(self.array_data)

    def sparcify_data(self):
        self.data = random.sample(self.data, len(self.data)/2.0)

    def analyze_grasp(self, location_info_req):
        """
        @param location_info_req: grasp message to analyze
        :type location_info_req: moveit_msgs.srv.LocationInfoRequest
        @return: Whether the grasp is expected to succeed
        @rtype: bool
        """

        rospy.loginfo(self.__class__.__name__ + " received analyze grasp request: " + str(location_info_req))
        grasp_msg = location_info_req.grasp
        success, result = self.handle_reachability_callback(grasp_msg)
        #success == None implies the analysis itself failed to run. Do nothing for now.
        if success == []:
            return False

        gs = graspit_msgs.msg.GraspStatus()
        if not success:
            if result != []:
                gs.status_msg = "Grasp %i unreachable: %i %i"%(grasp_msg.secondary_qualities[0],
                                                               success, result.error_code.val)
            rospy.loginfo(gs.status_msg)

        if grasp_msg.secondary_qualities:
            gs.grasp_identifier = grasp_msg.secondary_qualities[0]

        if result:
            gs.grasp_status = success or result.error_code.val
        else:
            gs.grasp_status = success

        self.train_model(grasp_msg, gs.grasp_status)

        response = moveit_trajectory_planner.srv.LocationInfoResponse(success)
        rospy.loginfo(self.__class__.__name__ + " finished analyze grasp request: " + str(response))
        return response


if __name__ == '__main__':

    try:
        rospy.init_node('graspit_reachability_checker')

        reachability_checker = ReachabilityChecker()

        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            loop.sleep()
    except rospy.ROSInterruptException: pass