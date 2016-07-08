#!/usr/bin/env python

import sys

import rospy
import moveit_msgs.msg
import moveit_msgs.msg
import control_msgs.msg
import actionlib
import moveit_commander

import graspit_msgs.msg
import graspit_msgs.srv
from grasp_execution_helpers import (barrett_manager, jaco_manager, execution_stages, robot_interface)
from common_helpers.grasp_reachability_analyzer import GraspReachabilityAnalyzer

import common_helpers.GraspManager




class GraspExecutionNode():

    def __init__(self, node_name='grasp_execution_node', manual_mode=False):

        rospy.init_node(node_name)

        self.use_robot_hw = rospy.get_param('use_robot_hw', False)
        self.grasp_approach_tran_frame = rospy.get_param('/approach_tran_frame', '/approach_tran')
        self.trajectory_display_topic = rospy.get_param('trajectory_display_topic', "/move_group/display_planned_path")
        self.grasp_listener_topic = rospy.get_param('grasp_listener_topic', "/graspit/grasps")
        self.move_group_name = rospy.get_param('/arm_name', 'manipulator')
        self.reachability_planner_id = self.move_group_name + rospy.get_param('grasp_executer/planner_config_name',
                                                                              'SBLkConfigDefault2')

        display_trajectory_publisher = rospy.Publisher(self.trajectory_display_topic, moveit_msgs.msg.DisplayTrajectory)

        self.trajectory_action_client = actionlib.SimpleActionClient('/mico_arm_driver/controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        hand_manager = common_helpers.GraspManager.GraspManager(jaco_manager, self.move_group_name)

        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander(self.move_group_name)

        grasp_reachability_analyzer = GraspReachabilityAnalyzer(group, self.grasp_approach_tran_frame)
        grasp_reachability_analyzer.planner_id = self.reachability_planner_id

        self.robot_interface = robot_interface.RobotInterface(trajectory_action_client=self.trajectory_action_client,
                                                               display_trajectory_publisher=display_trajectory_publisher,
                                                               hand_manager=hand_manager,
                                                               group=group,
                                                               grasp_reachability_analyzer=grasp_reachability_analyzer)

        self.pre_planning_pipeline = execution_stages.PrePlanningPipeline(self.robot_interface)
        self.execution_pipeline = execution_stages.GraspExecutionPipeline(self.robot_interface)

        self.last_grasp_time = 0

        if not manual_mode:
            self._grasp_execution = actionlib.SimpleActionServer("grasp_execution_action",
                                                                 graspit_msgs.msg.GraspExecutionAction,
                                                                 execute_cb=self._grasp_execution_cb,
                                                                 auto_start=False)
            self._grasp_execution.start()

            self._after_grasping = actionlib.SimpleActionServer("after_grasping_action",
                                                                graspit_msgs.msg.AfterGraspingAction,
                                                                execute_cb=self._after_grasping_cb,
                                                                auto_start=False)
            self._after_grasping.start()

            self._toggle_gripper = actionlib.SimpleActionServer("toggle_gripper_action",
                                                                graspit_msgs.msg.ToggleGripperAction,
                                                                execute_cb=self._toggle_gripper_cb,
                                                                auto_start=False)
            self._toggle_gripper.start()

            self._manual_movement = actionlib.SimpleActionServer("manual_action",
                                                                 graspit_msgs.msg.ManualAction,
                                                                 execute_cb=self._manual_movement_cb,
                                                                 auto_start=False)
            self._manual_movement.start()

        rospy.loginfo(self.__class__.__name__ + " is initialized")


    def _grasp_execution_cb(self, grasp_goal): # NEW 6/15/16 ******

        rospy.loginfo("GraspExecutor::process_grasp_msg::" + str(grasp_goal))

        status = graspit_msgs.msg.GraspStatus.SUCCESS
        status_msg = "grasp_succeeded"
        success = True

        #Pre Planning Moves, normally home arm and open hand
        if self.use_robot_hw and success:
            success, status_msg = self.pre_planning_pipeline.run(grasp_goal.grasp, None, self._grasp_execution)

        if success:
            #Generate Pick Plan
            success, pick_plan = self.robot_interface.generate_pick_plan(grasp_goal.grasp)
            if not success:
                grasp_status_msg = "MoveIt Failed to plan pick"
                status = graspit_msgs.msg.GraspStatus.ROBOTERROR
                rospy.logerr(grasp_status_msg)

        #Execute Plan on actual robot
        if self.use_robot_hw and success:
            success, status_msg = self.execution_pipeline.run(grasp_goal.grasp, pick_plan, self._grasp_execution)

        # if success:
        #     self.robot_interface.move_object()


        #need to return [] for empty response.
        print success
        print status_msg
        _result = graspit_msgs.msg.GraspExecutionResult()
        _result.success = success
        self._grasp_execution.set_succeeded(_result)
        return []


    def _after_grasping_cb(self, goal):
        action_type = goal.action

        print("ACTION REQUESTED: " + action_type)

        if action_type == "moveobject":
            completed = self.robot_interface.move_object()
        else:
            completed = self.robot_interface.open_hand_and_go_home()

        _result = graspit_msgs.msg.AfterGraspingResult()
        _result.success = completed
        self._after_grasping.set_succeeded(_result)
        return []



    def _toggle_gripper_cb(self, goal):
        _result = graspit_msgs.msg.ToggleGripperResult()
        fingers = self.robot_interface.hand_manager.hand_manager.get_mico_joints()
        if fingers[0] < 3000:
            self.robot_interface.hand_manager.close_hand()
        else:
            self.robot_interface.hand_manager.open_hand()
        _result.success = True
        self._toggle_gripper.set_succeeded(_result)
        return []

    def _manual_movement_cb(self, goal):
        """ axis: x = 0, y = 1, z = 2, roll = 3, pitch = 4, yaw = 5; direction: positive = 1, negative = -1 """
        _result = graspit_msgs.msg.ManualResult()
        success = True
        axis = goal.manualinfo.axis
        direction = goal.manualinfo.direction

        if axis < 3:
            distance = 0
            while (success == True) and (distance < 0.5):
                if self._manual_movement.is_preempt_requested():
                    self._manual_movement.set_preempted()
                    
                    return
                success = self.robot_interface.cartesian_translation2(axis, direction, server=self._manual_movement)
                distance += 0.05

        else:
            while success == True:
                if self._manual_movement.is_preempt_requested():
                    self._manual_movement.set_preempted()
                    return
                success = self.robot_interface.manual_move(axis, direction, server=self._manual_movement)

        if success:
            _result.success = success
            self._manual_movement.set_succeeded(_result)
        return []



if __name__ == '__main__':

    try:
        ge = GraspExecutionNode()
        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            loop.sleep()

    except rospy.ROSInterruptException:
        rospy.signal_shutdown()
