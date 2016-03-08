#!/usr/bin/env python

import sys

import rospy
import moveit_msgs.msg
import moveit_msgs.msg
import control_msgs.msg
import actionlib
import moveit_commander

import graspit_msgs.msg
from grasp_execution_helpers import (barrett_manager, jaco_manager, execution_stages, robot_interface2)
from common_helpers.grasp_reachability_analyzer import GraspReachabilityAnalyzer

import common_helpers.GraspManager


class GraspExecutionNode():

    def __init__(self, node_name='grasp_execution_node', manual_mode=False):

        rospy.init_node(node_name)

        self.use_robot_hw = rospy.get_param('use_robot_hw', False)
        self.grasp_approach_tran_frame = rospy.get_param('/approach_tran_frame', '/approach_tran')
        self.trajectory_display_topic = rospy.get_param('trajectory_display_topic', "/move_group/display_planned_path")
        self.grasp_listener_topic = rospy.get_param('grasp_listener_topic', "/graspit/grasps")
        self.move_group_name = rospy.get_param('/arm_name', 'StaubliArm')
        self.reachability_planner_id = self.move_group_name + rospy.get_param('grasp_executer/planner_config_name',
                                                                              'SBLkConfigDefault2')

        self.trajectory_action_client_topic = rospy.get_param('trajectory_action_client_topic', '/setFollowTrajectory')

        if not manual_mode:
            self.grasp_listener = rospy.Subscriber(self.grasp_listener_topic,
                                                   graspit_msgs.msg.Grasp,
                                                   callback=self.handle_grasp_msg,
                                                   queue_size=1)

        display_trajectory_publisher = rospy.Publisher(self.trajectory_display_topic, moveit_msgs.msg.DisplayTrajectory)

        if self.move_group_name == 'StaubliArm':
            hand_manager = common_helpers.GraspManager.GraspManager(barrett_manager, self.move_group_name)
        else:
            hand_manager = common_helpers.GraspManager.GraspManager(jaco_manager, self.move_group_name)

        trajectory_action_client = actionlib.SimpleActionClient(self.trajectory_action_client_topic,
                                                                control_msgs.msg.FollowJointTrajectoryAction)

        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander(self.move_group_name)
        
        grasp_reachability_analyzer = GraspReachabilityAnalyzer(group, self.grasp_approach_tran_frame)
        grasp_reachability_analyzer.planner_id = self.reachability_planner_id

        self.robot_interface = robot_interface2.RobotInterface(trajectory_action_client=trajectory_action_client,
                                                               display_trajectory_publisher=display_trajectory_publisher,
                                                               hand_manager=hand_manager,
                                                               group=group,
                                                               grasp_reachability_analyzer=grasp_reachability_analyzer)

        self.execution_pipeline = execution_stages.GraspExecutionPipeline(self.robot_interface)

        self.last_grasp_time = 0

        rospy.loginfo(self.__class__.__name__ + " is initialized")

    def handle_grasp_msg(self, grasp_msg):

        rospy.loginfo("GraspExecutor::process_grasp_msg::" + str(grasp_msg))

        status = graspit_msgs.msg.GraspStatus.SUCCESS
        status_msg = "grasp_succeeded"

        # #Home Arm
        success = self.robot_interface.home_arm()
        if not success:
            grasp_status_msg = "Failed to Home the Arm"
            status = graspit_msgs.msg.GraspStatus.ROBOTERROR
            rospy.logerr(grasp_status_msg)
            return status, status_msg

        #Open Hand
        success = self.robot_interface.hand_manager.open_hand()
        if not success:
            grasp_status_msg = "Failed to Home the Arm"
            status = graspit_msgs.msg.GraspStatus.ROBOTERROR
            rospy.logerr(grasp_status_msg)
            return status, status_msg

        #Generate Pick Plan
        success, pick_plan = self.robot_interface.generate_pick_plan(grasp_msg)
        if not success:
            grasp_status_msg = "MoveIt Failed to plan pick"
            status = graspit_msgs.msg.GraspStatus.ROBOTERROR
            rospy.logerr(grasp_status_msg)
            return status, status_msg

        #Execute Plan on actual robot
        if self.use_robot_hw and success:
            success, status_msg = self.execution_pipeline.run(grasp_msg, pick_plan)

        return status, status_msg


if __name__ == '__main__':

    try:
        ge = GraspExecutionNode()
        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            loop.sleep()

    except rospy.ROSInterruptException:
        pass
