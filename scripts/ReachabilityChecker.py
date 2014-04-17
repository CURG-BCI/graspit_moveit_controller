#!/usr/bin/env python

import moveit_commander
import moveit_msgs.msg

import convert_graspit_msg
import graspit_msgs

from sklearn import *
import rospy
import random
import graspit_msgs.msg
import itertools
import tf_conversions.posemath as pm
from numpy import dot
from numpy import array, unique
from math import acos
import std_msgs.msg

import roslib
roslib.load_manifest('moveit_trajectory_planner')
from moveit_trajectory_planner.srv import *
import actionlib
import graspit_msgs.srv


class ReachabilityChecker(object):
    def __init__(self, analyze_grasp_topic="/graspit/analyze_grasps",
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
        self.analyze_grasp_subscriber = rospy.Subscriber(name=analyze_grasp_topic,
                                                         data_class=graspit_msgs.msg.Grasp,
                                                         callback=self.analyze_grasp,
                                                         queue_size=1)

        self.analyze_pose_subscriber = rospy.Server(demo_pose_topic,
                                                    graspit_msgs.srv.AnalyzePose,
                                                    self.analyze_demonstration_pose, queue_size=1)

        self.grasp_analysis_publisher = rospy.Publisher(analyze_grasp_topic + "_results", graspit_msgs.msg.GraspStatus)
        self.demo_pose_analysis_publisher = rospy.Publisher(demo_pose_topic + "_results", std_msgs.msg.Float32)

        #setting up enviornment
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(move_group_name)
        self.pick_plan_client = actionlib.SimpleActionClient('/move_group/pickup', moveit_msgs.msg.PickupAction)
        self.support_surface_ = ""
        self.planning_time_ = 2
        self.grasp_approach_tran_frame = grasp_approach_tran_frame

    def construct_pickup_goal(self, graspit_grasp_msg):
        """
        :type graspit_grasp_msg: moveit_msgs.msg.Grasp
        """
        goal = moveit_msgs.msg.PickupGoal()

        goal.target_name = graspit_grasp_msg.object_name
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
        goal.planning_options.can_replan = False
        goal.planning_options.look_around = False
        goal.planning_options.replan_delay = 10.0
        goal.planning_options.planning_scene_diff = True
        goal.planning_options.robot_state.is_diff = True

        return goal

    def handle_reachability_callback(self, graspit_grasp_msg):
        """

        :type graspit_grasp_msg: graspit_msgs.msg.Grasp
        """
        moveit_grasp_msg = convert_graspit_msg.graspit_grasp_to_moveit_grasp(graspit_grasp_msg,
                                                                             self.grasp_approach_tran_frame)
        try:
            self.pick_plan_client.wait_for_server(rospy.Duration(0))
        except Exception as e:
            rospy.logerr("ReachabilityChecker::handle_reachability_callback::Failed to reach pick action server"
                         " with err: %s"%(e.message))

        pickup_goal = self.construct_pickup_goal(moveit_grasp_msg)
        self.pick_plan_client.send_goal(pickup_goal)

        success = self.pick_plan_client.wait_for_result(rospy.Duration(10))
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

    def analyze_grasp(self, grasp_msg):
        """

        @param grasp_msg: grasp message to analyze
        @type grasp_msg: graspit_msgs.msg.Grasp
        @return: None
        """
        success, result = self.handle_reachability_callback(grasp_msg)
        #success == None implies the analysis itself failed to run. Do nothing for now.
        if success == []:
            return

        gs = graspit_msgs.msg.GraspStatus()
        if not success:
            if result != []:
                gs.status_msg = "Grasp %i unreachable: %i %i"%(grasp_msg.secondary_qualities[0],
                                                               success, result.error_code.val)
            print gs.status_msg
        gs.grasp_identifier = grasp_msg.secondary_qualities[0]
        gs.grasp_status = success | result.error_code.val

        self.train_model(grasp_msg, gs.grasp_status)
        self.grasp_analysis_publisher.publish(gs)
