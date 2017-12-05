import rospy

import graspit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg

import moveit_commander
import moveit_python

import tf_conversions.posemath as pm
import tf
import actionlib
import copy


# Tools for grasping
class GraspingClient(object):

    def __init__(self,
                 arm_topic,                  # type: str
                 gripper_topic,              # type: str
                 grasp_approach_tran_frame,  # type: str
                 analyzer_planner_id,        # type: str
                 execution_planner_id,       # type: str
                 allowed_analyzing_time,     # type: int
                 allowed_execution_time      # type: int
                 ):
        """
        :param arm_topic: Topic for arm controller
        :param gripper_topic: Topic for gripper controller
        :param grasp_approach_tran_frame: Transform for approach_tran
        :param analyzer_planner_id: Planner type for planning command
        :param execution_planner_id: Planner type for executing command
        :param allowed_analyzing_time: Seconds for analyzing grasp
        :param allowed_execution_time: Seconds for executing grasp
        """
        self.arm_move_group = moveit_commander.MoveGroupCommander(arm_topic)
        self.gripper_move_group = moveit_commander.MoveGroupCommander(gripper_topic)

        self.pick_place_analyzer = moveit_python.PickPlaceInterface(arm_topic, gripper_topic, plan_only=True)
        self.pick_place_executor = moveit_python.PickPlaceInterface(arm_topic, gripper_topic, plan_only=False)

        self.analyzer_planner_id = analyzer_planner_id
        self.execution_planner_id = execution_planner_id
        self.grasp_approach_tran_frame = grasp_approach_tran_frame
        self.tf_listener = tf.TransformListener()

        self.allowed_analyzing_time = allowed_analyzing_time
        self.allowed_execution_time = allowed_execution_time

        self.tf_broadcaster = tf.TransformBroadcaster()

    # PRIVATE


    # PUBLIC
    def analyze_graspit_grasp(self, graspit_grasp_msg):
        # type: (graspit_msgs.msg.Grasp) -> (bool, moveit_msgs.msg.PickupResult)

        moveit_grasp_msg = message_utils.graspit_grasp_to_moveit_grasp(graspit_grasp_msg,
                                                                       self.arm_move_group,
                                                                       self.tf_listener,
                                                                       self.grasp_approach_tran_frame)

        # type: moveit_grasp_msg -> moveit_msgs.msg.Grasp

        pick_result = self.pick_place_analyzer.pickup(name=graspit_grasp_msg.object_name,
                                                      grasps=[moveit_grasp_msg, ],
                                                      planner_id=self.analyzer_planner_id,
                                                      planning_time=self.allowed_analyzing_time)

        # type: pick_result -> moveit_msgs.msg.PickupResult
        success = pick_result.error_code.val == pick_result.error_code.SUCCESS
        if success:
            rospy.loginfo("Grasp is reachable. Result: {}".format(pick_result))
        else:
            rospy.loginfo("Grasp is not reachable. Result: {}".format(pick_result))

        return success, pick_result

    def execute_graspit_grasp(self, graspit_grasp_msg):
        # type: (graspit_msgs.msg.Grasp) -> (bool, moveit_msgs.msg.PickupResult)

        moveit_grasp_msg = message_utils.graspit_grasp_to_moveit_grasp(graspit_grasp_msg,
                                                                       self.arm_move_group,
                                                                       self.tf_listener,
                                                                       self.grasp_approach_tran_frame)

        # type: moveit_grasp_msg -> moveit_msgs.msg.Grasp

        pick_result = self.pick_place_analyzer.pickup(name=graspit_grasp_msg.object_name,
                                                      grasps=[moveit_grasp_msg, ],
                                                      planner_id=self.execution_planner_id,
                                                      planning_time=self.allowed_execution_time)

        # type: pick_result -> moveit_msgs.msg.PickupResult
        success = pick_result.error_code.val == pick_result.error_code.SUCCESS
        if success:
            rospy.loginfo("Grasp executed correctly. Result: {}".format(pick_result))
        else:
            rospy.loginfo("Grasp did not execute. Result: {}".format(pick_result))

        return success, pick_result

    def place(self, object_name, pick_result, pose_stamped):
        # type: (str, moveit_msgs.msg.PickupResult, geometry_msgs.msg.PoseStamped) -> bool
        places = list()
        l = moveit_msgs.msg.PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))

        success, place_result = self.pick_place_executor.place_with_retry(object_name, places)
        return success

    def pub_tfs(self, graspit_grasp_msg, moveit_grasp_msg):
        # type: (graspit_msgs.msg.Grasp, moveit_msgs.msg.Grasp) -> None
        tf_pose = pm.toTf(pm.fromMsg(moveit_grasp_msg.grasp_pose.pose))
        self.tf_broadcaster.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), "/moveit_end_effector_frame",
                                          graspit_grasp_msg.object_name)

        tf_pose = pm.toTf(pm.fromMsg(graspit_grasp_msg.final_grasp_pose))
        self.tf_broadcaster.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), "/grasp_approach_tran",
                                          graspit_grasp_msg.object_name)

