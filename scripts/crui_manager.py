#!/usr/bin/env python

import rospy
import graspit_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg
import block_recognition.msg
import typing

import sys
import moveit_commander
import actionlib
import graspit_moveit_controller
import world_manager
import tf
import tf_conversions.posemath as pm


class CRUIManager(object):

    def __init__(self):

        rospy.init_node('cruimanager')
        moveit_commander.roscpp_initialize(sys.argv)

        # Pull all params off param server
        self.analyze_grasp_topic = rospy.get_param("analyze_grasp_topic")
        self.execute_grasp_topic = rospy.get_param("execute_grasp_topic")
        self.run_recognition_topic = rospy.get_param("run_recognition_topic")
        self.grasp_approach_tran_frame = rospy.get_param("grasp_approach_tran_frame")
        self.world_frame = rospy.get_param("world_frame")
        self.arm_move_group_name = rospy.get_param("arm_move_group_name")
        self.gripper_move_group_name = rospy.get_param("gripper_move_group_name")

        self.analyzer_planner_id = rospy.get_param("analyzer_planner_id")
        self.executor_planner_id = rospy.get_param("executor_planner_id")
        self.allowed_analyzing_time = rospy.get_param("allowed_analyzing_time")
        self.allowed_execution_time = rospy.get_param("allowed_execution_time")

        self.grasping_controller = graspit_moveit_controller.MoveitPickPlaceInterface(
            arm_name=self.arm_move_group_name,
            gripper_name=self.gripper_move_group_name,
            grasp_approach_tran_frame=self.grasp_approach_tran_frame,
            analyzer_planner_id=self.analyzer_planner_id,
            execution_planner_id=self.executor_planner_id,
            allowed_analyzing_time=self.allowed_analyzing_time,
            allowed_execution_time=self.allowed_execution_time
        )

        self.scene = moveit_commander.PlanningSceneInterface()
        self.block_recognition_client = block_recognition.BlockRecognitionClient()
        self.world_manager_client = world_manager.world_manager_client.WorldManagerClient()
        self.tf_listener = tf.TransformListener()

        # Start grasp analyzer action server
        self._analyze_grasp_as = actionlib.SimpleActionServer(self.analyze_grasp_topic,
                                                              graspit_msgs.msg.CheckGraspReachabilityAction,
                                                              execute_cb=self._analyze_grasp_reachability_cb,
                                                              auto_start=False)
        self._analyze_grasp_as.start()

        # Start grasp execution action server
        self._execute_grasp_as = actionlib.SimpleActionServer(self.execute_grasp_topic,
                                                              graspit_msgs.msg.GraspExecutionAction,
                                                              execute_cb=self._execute_grasp_cb,
                                                              auto_start=False)
        self._execute_grasp_as.start()

        # Start object recognition action server
        self._run_recognition_as = actionlib.SimpleActionServer(self.run_recognition_topic,
                                                                graspit_msgs.msg.RunObjectRecognitionAction,
                                                                execute_cb=self._run_recognition_cb,
                                                                auto_start=False)
        self._run_recognition_as.start()

        rospy.loginfo(self.__class__.__name__ + " is inited")

    def _graspit_grasp_to_moveit_grasp(self, graspit_grasp):
        # type: (graspit_msgs.msg.Grasp) -> moveit_msgs.msg.Grasp

        pre_grasp_approach_direction = geometry_msgs.msg.Vector3Stamped()
        pre_grasp_approach_direction.header.frame_id = rospy.get_param("pre_grasp_approach_direction_frame_id")
        pre_grasp_approach_direction.vector.x = rospy.get_param("pre_grasp_approach_direction_x")
        pre_grasp_approach_direction.vector.y = rospy.get_param("pre_grasp_approach_direction_y")
        pre_grasp_approach_direction.vector.z = rospy.get_param("pre_grasp_approach_direction_z")

        post_grasp_retreat_direction = geometry_msgs.msg.Vector3Stamped()
        post_grasp_retreat_direction.header.frame_id = rospy.get_param("post_grasp_retreat_direction_frame_id")
        post_grasp_retreat_direction.vector.x = rospy.get_param("post_grasp_retreat_direction_x")
        post_grasp_retreat_direction.vector.y = rospy.get_param("post_grasp_retreat_direction_y")
        post_grasp_retreat_direction.vector.z = rospy.get_param("post_grasp_retreat_direction_z")

        moveit_grasp_msg = graspit_moveit_controller.graspit_grasp_to_moveit_grasp(
            graspit_grasp_msg=graspit_grasp,
            listener=self.tf_listener,
            grasp_tran_frame_name=self.grasp_approach_tran_frame,
            end_effector_link=self.grasping_controller.get_end_effector_link(),

            pre_grasp_goal_point_effort=rospy.get_param("pre_grasp_goal_point_effort"),
            pre_grasp_goal_point_positions=rospy.get_param("pre_grasp_goal_point_positions"),
            pre_grasp_goal_point_time_from_start_secs=rospy.get_param("pre_grasp_goal_point_time_from_start_secs"),
            pre_grasp_joint_names=rospy.get_param("pre_grasp_joint_names"),

            grasp_goal_point_effort=rospy.get_param("grasp_goal_point_effort"),
            grasp_goal_point_positions=rospy.get_param("grasp_goal_point_positions"),
            grasp_goal_point_time_from_start_secs=rospy.get_param("grasp_goal_point_time_from_start_secs"),

            grasp_posture_joint_names=rospy.get_param("grasp_posture_joint_names"),

            pre_grasp_approach_min_distance=rospy.get_param("pre_grasp_approach_min_distance"),
            pre_grasp_approach_desired_distance=rospy.get_param("pre_grasp_approach_desired_distance"),
            pre_grasp_approach_direction=pre_grasp_approach_direction,

            post_grasp_retreat_min_distance=rospy.get_param("post_grasp_retreat_min_distance"),
            post_grasp_retreat_desired_distance=rospy.get_param("post_grasp_retreat_desired_distance"),
            post_grasp_retreat_direction=post_grasp_retreat_direction,

            max_contact_force=rospy.get_param("max_contact_force")
        )

        return moveit_grasp_msg

    def _analyze_grasp_reachability_cb(self, goal):
        # type: (graspit_msgs.msg.CheckGraspReachabilityGoal) -> graspit_msgs.msg.CheckGraspReachabilityResult
        """
        @return: Whether the grasp is expected to succeed
        """
        # Convert graspit grasp to moveit grasp
        rospy.loginfo("Analyzing grasp for object: {}".format(goal.grasp.object_name))

        block_names = self.scene.get_attached_objects().keys()
        self.grasping_controller.detach_all_blocks(block_names)

        moveit_grasp_msg = self._graspit_grasp_to_moveit_grasp(goal.grasp)
        success, pick_result = self.grasping_controller.analyze_moveit_grasp(goal.grasp.object_name, moveit_grasp_msg)

        result = graspit_msgs.msg.CheckGraspReachabilityResult()
        result.isPossible = success
        result.grasp_id = goal.grasp.grasp_id

        rospy.loginfo("Able to execute grasp with grasp id {} after analysis: {}".format(goal.grasp.grasp_id, success))
        self._analyze_grasp_as.set_succeeded(result)

        return []

    def _execute_grasp_cb(self, goal):
        # type: (graspit_msgs.msg.GraspExecutionGoal) -> graspit_msgs.msg.GraspExecutionResult
        rospy.loginfo("Executing grasp goal")
        result = graspit_msgs.msg.GraspExecutionResult()
        result.success = False

        block_names = self.scene.get_attached_objects().keys()
        self.grasping_controller.detach_all_blocks(block_names)

        # Acquire block position for place
        objects = self.scene.get_object_poses([goal.grasp.object_name])
        if not goal.grasp.object_name in objects:
            rospy.logerr("Object {} not in planning scene. Execute grasp failed".format(goal.grasp.object_name))
            self._execute_grasp_as.set_aborted(result)
            return []

        block_pose_stamped = geometry_msgs.msg.PoseStamped()
        block_pose_stamped.pose = objects[goal.grasp.object_name]
        block_pose_stamped.header.frame_id = self.grasping_controller.get_planning_frame()

        rospy.loginfo("Object {} in planning scene. Pose: {}".format(goal.grasp.object_name, block_pose_stamped.pose))

        # Shift block pose to place location in param server
        block_pose_stamped.pose.position.x = rospy.get_param("final_block_position_x")
        block_pose_stamped.pose.position.y = rospy.get_param("final_block_position_y")
        block_pose_stamped.pose.position.z = rospy.get_param("final_block_position_z")

        rospy.loginfo("Placing block as position ({}, {}, {})"
                      .format(block_pose_stamped.pose.position.x,
                              block_pose_stamped.pose.position.y,
                              block_pose_stamped.pose.position.z))

        # Convert graspit grasp to moveit grasp
        moveit_grasp_msg = self._graspit_grasp_to_moveit_grasp(goal.grasp)

        # Execute pick on block
        success, pick_result = self.grasping_controller.execute_moveit_grasp(goal.grasp.object_name, moveit_grasp_msg)
        # type: pick_result -> moveit_msgs.msg.PickupResult

        if not success:
            rospy.logerr("Failed to execute pick. Reason: {}".format(pick_result))
            self._execute_grasp_as.set_aborted(result)
            return []
        else:
            rospy.loginfo("Successfully executed pick")

        # Execute place on block
        success, place_result = self.grasping_controller.place(goal.grasp.object_name, pick_result, block_pose_stamped)

        if not success:
            rospy.logerr("Failed to execute place. Reason: {}".format(place_result))
            self._execute_grasp_as.set_aborted(result)
            return []
        else:
            rospy.loginfo("Successfully executed place")

        # Home arm and open hand
        success = self.grasping_controller.home_arm()
        if not success:
            rospy.logerr("Failed to home arm")
            self._execute_grasp_as.set_aborted(result)
            return []
        else:
            rospy.loginfo("Successfully homed arm")

        success = self.grasping_controller.open_hand()
        if not success:
            rospy.logerr("Failed to open hand")
            self._execute_grasp_as.set_aborted(result)
            return []
        else:
            rospy.loginfo("Successfully opened hand")

        result.success = True
        self._execute_grasp_as.set_succeeded(result)

        return []

    def _run_recognition_cb(self, goal):
        rospy.loginfo("Running recognition")
        result = graspit_msgs.msg.RunObjectRecognitionResult()
        # type: result -> graspit_msgs.msg.RunObjectRecognitionResult

        self.world_manager_client.clear_objects()

        detected_blocks = self.block_recognition_client.find_blocks()
        # type: detected_blocks -> typing.List[block_recognition.msg.DetectedBlock]

        if len(detected_blocks) == 0:
            rospy.loginfo("Detected no blocks. No work done. ")
            self._run_recognition_as.set_succeeded(result)
            return []

        rospy.loginfo("Detected {} blocks".format(len(detected_blocks)))

        for detected_block in detected_blocks:
            # Add all blocks to the scene
            self.world_manager_client.add_box(detected_block.unique_block_name,
                                             detected_block.pose_stamped,
                                             detected_block.edge_length,
                                             detected_block.edge_length,
                                             detected_block.edge_length)

            self.tf_listener.waitForTransform(self.world_frame, detected_block.unique_block_name, rospy.Time(0), rospy.Duration(10))
            detected_block_world_pose = pm.toMsg(pm.fromTf(self.tf_listener.lookupTransform(self.world_frame, detected_block.unique_block_name, rospy.Time(0))))

            # Add blocks to graspit result
            object_info = graspit_msgs.msg.ObjectInfo(
                detected_block.unique_block_name,
                detected_block.mesh_filename,
                detected_block_world_pose
            )
            result.object_info.append(object_info)

        rospy.loginfo("Finished recognition")
        self._run_recognition_as.set_succeeded(result)
        return []


def main():
    try:
        crui_manager = CRUIManager()
        loop = rospy.Rate(10)

        while not rospy.is_shutdown():
            loop.sleep()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown(reason="Interrupted")


if __name__ == '__main__':
    main()
