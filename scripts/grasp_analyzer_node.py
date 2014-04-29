#!/usr/bin/env python

import rospy
import roslib

import graspit_msgs.msg
import moveit_trajectory_planner.srv
import graspit_msgs.srv
from grasp_analyzer_helpers.demonstration_pose_analyzer import DemonstrationPoseAnalyzer
from common_helpers.grasp_reachability_analyzer import GraspReachabilityAnalyzer

import sys
import moveit_commander
import ipdb
roslib.load_manifest('moveit_trajectory_planner')


class GraspAnalyzerNode(object):

    def __init__(self,
                 analyze_grasp_topic="/moveit_trajectory_planner/check_reachability",
                 demo_pose_topic="/graspit/analyze_demo_pose", move_group_name='StaubliArm',
                 grasp_approach_tran_frame='/approach_tran'):
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

        self.analyze_grasp_service = rospy.Service(analyze_grasp_topic,
                                                   moveit_trajectory_planner.srv.LocationInfo,
                                                   self.analyze_grasp_reachability)

        self.analyze_pose_service = rospy.Service(demo_pose_topic,
                                                  graspit_msgs.srv.AnalyzePose,
                                                  self.analyze_demonstration_pose)

        self.demonstration_pose_analyzer = DemonstrationPoseAnalyzer()

        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander(move_group_name)

        self.grasp_reachability_analyzer = GraspReachabilityAnalyzer(group, grasp_approach_tran_frame)

        rospy.loginfo(self.__class__.__name__ + " is inited")

    def analyze_demonstration_pose(self, demo_pose):
        """
        :type demo_pose: graspit_msgs.srv.AnalyzePoseRequest
        :rtype response: graspit_msgs.srv.AnalyzePoseResponse
        """
        response = graspit_msgs.srv.AnalyzePoseResponse()
        success_probability = self.demonstration_pose_analyzer.analyze_pose(demo_pose)
        response.success_probability = success_probability

        return response

    def analyze_grasp_reachability(self, location_info_req):
        """
        @param location_info_req: grasp message to analyze
        :type location_info_req: moveit_msgs.srv.LocationInfoRequest
        @return: Whether the grasp is expected to succeed
        @rtype: bool
        """
        rospy.loginfo(self.__class__.__name__ + " received analyze grasp request: " + str(location_info_req))

        grasp_msg = location_info_req.grasp
        success, result = self.grasp_reachability_analyzer.query_moveit_for_reachability(grasp_msg)

        self.demonstration_pose_analyzer.train_model(grasp_msg, success)

        response = moveit_trajectory_planner.srv.LocationInfoResponse(success)

        rospy.loginfo(self.__class__.__name__ + " finished analyze grasp request: " + str(response))
        return response


if __name__ == '__main__':

    try:
        rospy.init_node('grasp_analyzer_node')

        grasp_analyzer_node = GraspAnalyzerNode()

        loop = rospy.Rate(10)
        ipdb.set_trace()
        while not rospy.is_shutdown():
            loop.sleep()
    except rospy.ROSInterruptException: pass
