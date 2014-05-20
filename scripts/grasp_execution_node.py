#!/usr/bin/env python

import sys
from time import time
import pdb
from numpy import array

import rospy
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.msg
import control_msgs.msg
import actionlib
import moveit_commander

import graspit_msgs.msg
from grasp_execution_helpers import barrett_manager
from common_helpers.grasp_reachability_analyzer import GraspReachabilityAnalyzer


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

        self.trajectory_action_client = actionlib.SimpleActionClient('setFollowTrajectory',
                                                                     control_msgs.msg.FollowJointTrajectoryAction)

        self.last_grasp_time = 0
        self.table_cube = [geometry_msgs.msg.Point(-0.7, 0, -0.02), geometry_msgs.msg.Point(0.2, 1, 1)]
        self.robot_running = use_robot_hw

        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(move_group_name)

        self.grasp_reachability_analyzer = GraspReachabilityAnalyzer(self.group, grasp_tran_frame_name)

        self.preshape_ratio = .5

        if bool(rospy.get_param('reload_model_rec', 0)):
            self.reload_model_list([])
        rospy.loginfo(self.__class__.__name__ + " is initialized")

    def run_trajectory(self, trajectory_msg):
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
        trajectory_result = control_msgs.msg.FollowJointTrajectoryResult(trajectory_result)
        control_msgs.msg.FollowJointTrajectoryResult(trajectory_result)
        """:type : control_msgs.msg.FollowJointTrajectoryResult"""
        success = trajectory_result.SUCCESSFUL == trajectory_result.error_code
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

    def process_grasp_msg(self, grasp_msg):
        """@brief - Attempt to grasp the object and lift it

        First moves the arm to pregrasp the object, then attempts to grasp and lift it.
        The grasp and lifting phase is currently completely open loop
        
        """
        try:
            #Reject multiple grasp messages sent too fast. They are likely a mistake
            #FIXME - We should probably just set the queue size to 1.
            if (time() - self.last_grasp_time) < 30:
                return [], []
            self.last_grasp_time = time()
            
            rospy.loginfo("GraspExecutor::process_grasp_msg::" + str(grasp_msg))
            grasp_status = graspit_msgs.msg.GraspStatus.SUCCESS
            grasp_status_msg = "grasp_succeeded"
            success = 1
            
            # Send the robot to its home position if it is actually running
            #and not currently there

            if self.robot_running:
                rospy.loginfo('GraspExecutor::process_grasp_msg::go home')
                self.group.set_named_target("home")
                plan = self.group.plan()
                success = self.group.execute(plan)
                if not success:
                    rospy.logerr("GraspExecutor::process_grasp_msg::Couldn't move arm home")

                #This sometimes fails near the end of the trajectory because the last few
                #steps of the trajectory are slow because our blending parameters
                #are probably wrong. It is basically in the home position, so we
                #don't check for success in reaching the home position because meaningful
                #failures are rare.


            #Open the hand - Leaves spread angle unchanged
            if success and self.robot_running:
                success, grasp_status_msg, positions = barrett_manager.open_barrett()
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

            if success:
                #Preshape the hand to the grasps' spread angle

                if self.robot_running:
                    success = barrett_manager.move_hand([0,0,0, grasp_msg.pre_grasp_dof[0]])

                    rospy.logerr('GraspExecutor::process_grasp_msg::pre-grasp')
                    #Pregrasp the object
                    if not success:
                        grasp_status_msg = "Failed to preshape hand"

            if success:
                #Move to the goal location and grasp object
                success, result = self.grasp_reachability_analyzer.query_moveit_for_reachability(grasp_msg)
                if not success:
                    grasp_status_msg = "MoveIt Failed to plan pick"

            #Failures shouldn't happen if grasp analysis is being used, so that's wierd.
            if not success:
                pdb.set_trace()

            #Now run stuff on the robot
            if self.robot_running:
                #Move the hand to the pre grasp position
                if success and self.robot_running:
                    success, grasp_status_msg, trajectory = self.run_pickup_trajectory(result, 0)


                #Preshape the hand before approach
                if success:
                    preapproach_shape = self.preshape_ratio *array(grasp_msg.final_grasp_dof)
                    preapproach_shape[0] = grasp_msg.final_grasp_dof[0]
                    success, grasp_status_msg, joint_angles = barrett_manager.move_hand(preapproach_shape)

                #do approach
                if success:
                    success, grasp_status_msg, trajectory = self.run_pickup_trajectory(result, 1)
                if success:
                    #Close the hand completely until the motors stall or they hit
                    #the final grasp DOFS
                    success, grasp_status_msg, joint_angles = barrett_manager.move_hand([grasp_msg.final_grasp_dof[1],grasp_msg.final_grasp_dof[2], grasp_msg.final_grasp_dof[3], grasp_msg.final_grasp_dof[0]])

                #close fingers
                if success:
                    #Now close the hand completely until the motors stall.
                    success, grasp_status_msg, joint_angles = barrett_manager.close_barrett()

                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

                #Now wait for user input on whether or not to lift the object
                if success:
                    selection = int(raw_input('Lift up (1) or not (0): '))
                    if selection == 1:
                        rospy.loginfo('GraspExecutor::process_grasp_msg::lift up the object')
                        #Lift the object using the staubli's straight line path planner
                        success, grasp_status_msg, trajectory_result = self.run_pickup_trajectory(result, 2)
                        if not success:
                            grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
                            grasp_status_msg = "Couldn't lift object"
                        else:
                            rospy.logerr('GraspExecutor::process_grasp_msg::not lift up the object')

            #Maybe decide if it has been successfully lifted here...
            if success:
                rospy.logwarn(grasp_status_msg)
            else:
                rospy.logfatal(grasp_status_msg)
            #Tell graspit whether the grasp succeeded
            self.graspit_status_publisher.publish(grasp_status, grasp_status_msg, -1)
            
            rospy.loginfo('GraspExecutor::process_grasp_msg::' + str(grasp_status_msg))

            return grasp_status, grasp_status_msg

        except Exception as e:
            #print any exceptions and drop in to the debugger.
            import traceback
            print traceback.format_exc()
            pdb.set_trace()


if __name__ == '__main__':
    try:        
        rospy.init_node('graspit_message_robot_server')

        use_robot_hw = rospy.get_param('use_robot_hw', False)
        print use_robot_hw
        rospy.loginfo("use_robot_hw value %d \n" % use_robot_hw)

        ge = GraspExecutor(use_robot_hw=use_robot_hw)

        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            loop.sleep()
    except rospy.ROSInterruptException:
        pass

