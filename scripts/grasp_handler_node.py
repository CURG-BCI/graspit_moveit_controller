#!/usr/bin/env python

import rospy
import graspit_msgs.msg
import sys
import moveit_commander
import actionlib


class GraspHandlerNode(object):

    def __init__(self):

        rospy.init_node('cruimanager')
        analyze_grasp_topic = "analyze_grasp_action"
        arm_move_group_name = rospy.get_param('/arm_name', 'arm')
        gripper_move_group_name = rospy.get_param("/gripper_name", "gripper")
        grasp_approach_tran_frame = rospy.get_param('approach_tran_frame', '/approach_tran')
        planner_id = arm_move_group_name + rospy.get_param('grasp_analyzer/planner_config_name', '[PRMkConfigDefault]')
        allowed_planning_time = rospy.get_param('~allowed_planning_time')

        moveit_commander.roscpp_initialize(sys.argv)

        group = moveit_commander.MoveGroupCommander(arm_move_group_name)

        self.grasp_reachability_analyzer = GraspReachabilityAnalyzer(group, grasp_approach_tran_frame, planner_id, allowed_planning_time)

        self._analyze_grasp_as = actionlib.SimpleActionServer(analyze_grasp_topic,
                                                              graspit_msgs.msg.CheckGraspReachabilityAction,
                                                              execute_cb=self.analyze_grasp_reachability_cb,
                                                              auto_start=False)
        self._analyze_grasp_as.start()

        rospy.loginfo(self.__class__.__name__ + " is inited")

    def _analyze_grasp_reachability_cb(self, goal):
        # type: (graspit_msgs.msg.CheckGraspReachabilityGoal) -> graspit_msgs.msg.CheckGraspReachabilityResult
        """
        @return: Whether the grasp is expected to succeed
        """
        # Convert graspit grasp to moveit grasp
        success, result = self.grasp_reachability_analyzer.query_moveit_for_reachability(goal.grasp)
        _result = graspit_msgs.msg.CheckGraspReachabilityResult()
        _result.isPossible = success
        _result.grasp_id = goal.grasp.grasp_id
        rospy.loginfo(self.__class__.__name__ + " finished analyze grasp request: " + str(_result))
        self._analyze_grasp_as.set_succeeded(_result)
        return _result

    def _execute_grasp_cb(self, goal):
        rospy.loginfo("GraspExecutor::process_grasp_msg::" + str(grasp_goal))

        status = graspit_msgs.msg.GraspStatus.SUCCESS
        status_msg = "grasp_succeeded"
        success = True

        # Pre Planning Moves, normally home arm and open hand
        if self.use_robot_hw and success:
            success, status_msg = self.pre_planning_pipeline.run(grasp_goal.grasp, None, self._grasp_execution)

        # IPython.embed()

        if success:
            # Generate Pick Plan
            success, pick_plan = self.robot_interface.generate_pick_plan(grasp_goal.grasp)
            if not success:
                grasp_status_msg = "MoveIt Failed to plan pick"
                status = graspit_msgs.msg.GraspStatus.ROBOTERROR
                rospy.logerr(grasp_status_msg)

        # Execute Plan on actual robot
        if self.use_robot_hw and success:
            success, status_msg = self.execution_pipeline.run(grasp_goal.grasp, pick_plan, self._grasp_execution)

        if success:
            rospy.loginfo("Successfully executed grasp")
            success = self.robot_interface.move_object()
        else:
            rospy.logerr("Picking up object failed")

        print success
        print status_msg
        _result = graspit_msgs.msg.GraspExecutionResult()
        _result.success = success
        self._grasp_execution.set_succeeded(_result)

        # need to return [] for empty response.
        return []

    def _run_recognition_cb(self, goal):
        pass

    # FUTURE
    def select_grasp_cb(self):
        pass

    def select_object_cb(self):
        pass

    def next_object_cb(self):
        pass

    def next_grasp_cb(self):
        pass

    def plan_new_grasps_cb(self):
        pass

    def back_from_select_object_cb(self):
        pass

    def stop_execution_cb(self):
        pass

    def continue_execution_cb(self):
        pass


def main():
    try:
        grasp_analyzer_node = GraspAnalyzerNode()
        loop = rospy.Rate(10)

        while not rospy.is_shutdown():
            loop.sleep()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown()


if __name__ == '__main__':
    main()
