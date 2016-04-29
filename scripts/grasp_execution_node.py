#!/usr/bin/env python

import sys
from time import time

import rospy
import moveit_msgs.msg
import control_msgs.msg
import actionlib
import moveit_commander
import importlib

import graspit_msgs.msg
from common_helpers.grasp_reachability_analyzer import GraspReachabilityAnalyzer

import common_helpers.GraspManager

import numpy


class GraspExecutor():
    """@brief - Generates a persistent converter between grasps published by a graspit commander and
    the trajectory planning module to actually lift objects off the table

    @member grasp_listener - subscriber to the graspit grasp channel. Expects graspit_msgs Grasp types
    @member name_listener - subscribes to an object name channel. Expects a string which corresponds
                            to an entry in the filename dictionary
    @member global_data - Planning environment
    @member target_object_name - the current grasp target
    """

    def __init__(self, use_robot_hw=True, move_group_name='StaubliArm', grasp_tran_frame_name='approach_tran'):

        self.grasp_listener = rospy.Subscriber("/graspit/grasps", graspit_msgs.msg.Grasp, self.process_grasp_msg)

        self.graspit_status_publisher = rospy.Publisher("/graspit/status", graspit_msgs.msg.GraspStatus)

        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                            moveit_msgs.msg.DisplayTrajectory)

        self.trajectory_action_client = actionlib.SimpleActionClient(rospy.get_param('trajectory_action_name'), control_msgs.msg.FollowJointTrajectoryAction)
        self.hand_manager = common_helpers.GraspManager.GraspManager(importlib.import_module(rospy.get_param('hand_manager')), move_group_name)

        # if move_group_name == 'StaubliArm':
        #     self.trajectory_action_client = actionlib.SimpleActionClient('/setFollowTrajectory',  control_msgs.msg.FollowJointTrajectoryAction)
        #     self.hand_manager = common_helpers.GraspManager.GraspManager(barrett_manager, move_group_name)
        # else:
        #     self.trajectory_action_client = actionlib.SimpleActionClient('/mico_arm_driver/controller/follow_joint_trajectory',  control_msgs.msg.FollowJointTrajectoryAction)
        #     self.hand_manager = common_helpers.GraspManager.GraspManager(jaco_manager, move_group_name)

        self.last_grasp_time = 0
        #self.table_cube = [geometry_msgs.msg.Point(-0.7, 0, -0.02), geometry_msgs.msg.Point(0.2, 1, 1)]
        self.robot_running = use_robot_hw

        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(move_group_name)

        #self.group.set_planner_id(move_group_name + 'SBLkConfigDefault2]')
        self.group.set_planner_id(move_group_name + 'PRMkConfigDefault')
        self.group.planner_id = move_group_name + rospy.get_param('grasp_executer/planner_config_name', 'SBLkConfigDefault2')

        self.grasp_reachability_analyzer = GraspReachabilityAnalyzer(self.group, grasp_tran_frame_name)
        self.grasp_reachability_analyzer.planner_id = move_group_name + rospy.get_param('grasp_analyzer/planner_config_name', 'PRMkConfigDefault')
        # self.grasp_reachability_analyzer.planner_id = move_group_name + 'PRMkConfigDefault'
        self.preshape_ratio = 0.0

        if bool(rospy.get_param('reload_model_rec', 0)):
            self.reload_model_list([])
        rospy.loginfo(self.__class__.__name__ + " is initialized")


    def run_trajectory(self, trajectory_msg):
        """:type : control_msgs.msg.FollowJointTrajectoryResult"""
        if not self.trajectory_action_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr('Failed to find trajectory server')
            return False, 'Failed to find trajectory server', []

        trajectory_action_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        trajectory_action_goal.trajectory = trajectory_msg
        self.trajectory_action_client.send_goal(trajectory_action_goal)

        if not self.trajectory_action_client.wait_for_result():
            rospy.logerr("Failed to execute trajectory")
            return False, "Failed to execute trajectory", []

        trajectory_result = self.trajectory_action_client.get_result()
        #trajectory_result = control_msgs.msg.FollowJointTrajectoryResult(trajectory_result)
        #control_msgs.msg.FollowJointTrajectoryResult(trajectory_result)

        """:type : control_msgs.msg.FollowJointTrajectoryResult"""

        success = (trajectory_result.SUCCESSFUL == trajectory_result.error_code)

        return success, None, trajectory_result

    def run_pickup_trajectory(self, pickup_result, pickup_phase):
        """

        :type pickup_phase: int
        :type pickup_result: moveit_msgs.msg.PickupResult
        """

        robot_trajectory_msg = pickup_result.trajectory_stages[pickup_phase]
        trajectory_msg = robot_trajectory_msg.joint_trajectory

        success, error_msg, trajectory_result = self.run_trajectory(trajectory_msg)
        return success, error_msg, trajectory_result

    def display_trajectory(self, trajectory_msg):
        """
        :type trajectory_msg: moveit_msgs.msg.trajectory_msg
        """
        display_msg = moveit_msgs.msg.DisplayTrajectory()
        display_msg.trajectory = trajectory_msg
        display_msg.model_id = self.group.get_name()
        self.display_trajectory_publisher.publish(display_msg)

    def fix_stage_0(self, pickup_result):
        grasp_status_msg = ''
        success = False

        if (pickup_result.trajectory_stages[0].joint_trajectory.joint_names !=
                pickup_result.trajectory_stages[2].joint_trajectory.joint_names):
            grasp_status_msg = ("joint names and order of stages 0 and 2 must agree! Aborting."
                                " Stage 0: %s, Stage 1: %s"%(
                                pickup_result.trajectory_stages[0].joint_trajectory.joint_names,
                                pickup_result.trajectory_stages[2].joint_trajectory.joint_names))
            rospy.logerror(grasp_status_msg)

        else:
            if (pickup_result.trajectory_stages[0].joint_trajectory.points[0].positions !=
                    pickup_result.trajectory_stages[2].joint_trajectory.points[-1].positions):
                stage2_start = pickup_result.trajectory_stages[2].joint_trajectory.points[0]
                pickup_result.trajectory_stages[0].joint_trajectory.points.append(stage2_start)
                grasp_status_msg = 'Stage 0 and Stage 2 were misaligned. Fixed missing trajectory point'

            else:
                grasp_status_msg = 'Stage 0 and Stage 2 were already aligned. No fix needed.'
            rospy.loginfo(grasp_status_msg)
            success = True

        return success, grasp_status_msg, pickup_result

    def process_grasp_msg(self, grasp_msg):
        """@brief - Attempt to grasp the object and lift it

        First moves the arm to pregrasp the object, then attempts to grasp and lift it.
        The grasp and lifting phase is currently completely open loop

        """
        if 1 > 0:
        #try:
            #Reject multiple grasp messages sent too fast. They are likely a mistake
            #FIXME - We should probably just set the queue size to 1.
            if (time() - self.last_grasp_time) < 3:
                return [], []
            self.last_grasp_time = time()

            rospy.loginfo("GraspExecutor::process_grasp_msg::" + str(grasp_msg))

            grasp_status = graspit_msgs.msg.GraspStatus.SUCCESS
            grasp_status_msg = "grasp_succeeded"
            success = 1

            # Send the robot to its home position if it is actually running
            #and not currently there

            # if self.robot_running:

                # home_joint_values = rospy.get_param('home_joint_values', [0, 0, 0, 0, 0, 0])
                # current_joint_values = self.group.get_current_joint_values()
                # rospy.loginfo("Current Joint Values: " + str(current_joint_values))
                # rospy.loginfo("home_joint_values: " + str(home_joint_values))
                # if not numpy.allclose(home_joint_values, current_joint_values, 0.2):

                #     print 'go home'
                #     print home_joint_values
                #     print current_joint_values
                #     self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 20))
                #     self.group.set_start_state_to_current_state()
                #     self.group.set_named_target("home")
                #     plan = self.group.plan()
                #     success = self.group.execute(plan)
                #     if not success:
                #         print "Couldn't move arm home"
                # else:
                #     print("already home")


            #This sometimes fails near the end of the trajectory because the last few
            #steps of the trajectory are slow because our blending parameters
            #are probably wrong. It is basically in the home position, so we
            #don't check for success in reaching the home position because meaningful
            #failures are rare.
            success = True

            #Open the hand - Leaves spread angle unchanged
            if success and self.robot_running:
                success, grasp_status_msg, positions = self.hand_manager.open_hand()
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

            if success:
                #Preshape the hand to the grasps' spread angle

                if self.robot_running:

                    rospy.logerr('GraspExecutor::process_grasp_msg::pre-grasp')
                    #Pregrasp the object
                    if not success:
                        grasp_status_msg = "Failed to preshape hand"

            if success:
                #Move to the goal location and grasp object
                for i in xrange(10):
                    success, result = self.grasp_reachability_analyzer.query_moveit_for_reachability(grasp_msg)
                    if success:
                        break
                    else:
                        print("Planning attempt %i"%(i))

                if not success:
                    grasp_status_msg = "MoveIt Failed to plan pick"

            #if success:
            #    success, grasp_status_msg, result = self.fix_stage_0(result)

            #Now run stuff on the robot
            if self.robot_running:
                rospy.loginfo("GraspExecutor::process_grasp_msg::Running grasp")
                #Move the hand to the pre grasp position
                if success and self.robot_running:
                    success, grasp_status_msg, trajectory = self.run_pickup_trajectory(result, 0)
                    rospy.loginfo("run pickup phase 0 success:%i"%success)

                # #Preshape the hand before approach
                # if success:
                #     success, grasp_status_msg = self.hand_manager.move_hand_trajectory(result.trajectory_stages[1].joint_trajectory)
                #     rospy.loginfo("run pickup phase 1 success:%i"%success)

                #do approach
                if success:

                    success, grasp_status_msg, trajectory = self.run_pickup_trajectory(result, 2)
                    rospy.loginfo("run pickup phase 2 success:%i"%success)

                if success:
                    #Close the hand completely until the motors stall or they hit
                    #the final grasp DOFS
                    rospy.loginfo('in phase 3')
                    #import IPython
                    #IPython.embed()
                    success, grasp_status_msg = self.hand_manager.move_hand_trajectory(result.trajectory_stages[3].joint_trajectory)
                    rospy.loginfo("run pickup phase 3 success:%i"%success)

                #close fingers
                if success:
                    #Now close the hand completely until the motors stall.
                    grasp_joint_state = self.hand_manager.joint_trajectory_to_joint_state(result.trajectory_stages[3].joint_trajectory, 0)
                    rospy.loginfo('in close hand')
                    #import IPython
                    #success, grasp_status_msg, joint_angles = self.hand_manager.close_grasp(grasp_joint_state)
                    success, reason, position = self.hand_manager.close_hand()
                    rospy.loginfo("closing hand completely success:%i" % success)


                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

                #Now wait for user input on whether or not to lift the object
                if success:
                    selection = 1
                    #selection = int(raw_input('Lift up (1) or not (0): '))
                    if selection == 1:
                        rospy.loginfo('GraspExecutor::process_grasp_msg::lift up the object')
                        #Lift the object using the staubli's straight line path planner
                        success, grasp_status_msg, trajectory_result = self.run_pickup_trajectory(result, 4)
                        rospy.loginfo("run pickup phase 4 success:%i" % success)
                        if success:
                            rospy.logerr('GraspExecutor::process_grasp_msg::lifted up the object')
                        else:
                            grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
                            grasp_status_msg = "Couldn't lift object"


            #Maybe decide if it has been successfully lifted here...
            if success:
                rospy.logwarn(grasp_status_msg)
            else:
                rospy.logfatal(grasp_status_msg)
            #Tell graspit whether the grasp succeeded
            self.graspit_status_publisher.publish(grasp_status, grasp_status_msg, -1)

            rospy.loginfo('GraspExecutor::process_grasp_msg::' + str(grasp_status_msg))
            return grasp_status, grasp_status_msg

        #except Exception as e:
        #    #print any exceptions and drop in to the debugger.
        #    import traceback
        #    print traceback.format_exc()
        #    pdb.set_trace()


if __name__ == '__main__':
    try:

        rospy.init_node('graspit_message_robot_server')

        use_robot_hw = rospy.get_param('use_robot_hw', True)
        move_group_name = rospy.get_param('/arm_name', 'StaubliArm')
        grasp_approach_tran_frame = rospy.get_param('/approach_tran_frame','/approach_tran')
        rospy.loginfo("use_robot_hw value %d \n" % use_robot_hw)

        ge = GraspExecutor(use_robot_hw=use_robot_hw, move_group_name=move_group_name, grasp_tran_frame_name=grasp_approach_tran_frame)


        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            loop.sleep()
    except rospy.ROSInterruptException:
        pass

