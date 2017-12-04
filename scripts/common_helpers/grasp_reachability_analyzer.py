
from common_helpers import message_utils
import rospy

import moveit_msgs.msg
import actionlib
import moveit_commander
import tf_conversions.posemath as pm
from visualization_msgs.msg import Marker
import numpy as np
import tf
from copy import deepcopy


class GraspReachabilityAnalyzer():

    def __init__(self, move_group, grasp_approach_tran_frame, planner_id, allowed_planning_time):
        """
        :type move_group: moveit_commander.MoveGroupCommander
        """
        self.pick_plan_client = actionlib.SimpleActionClient('/pickup', moveit_msgs.msg.PickupAction)
        self.move_group = move_group
        self.planner_id = planner_id
        self.grasp_approach_tran_frame = grasp_approach_tran_frame
        self.listener = tf.TransformListener()
        self.allowed_planning_time = allowed_planning_time
        self.planner_timeout = allowed_planning_time + 1 #Extra second for functional moveit overhead
        self.tf_broadcaster = tf.TransformBroadcaster()

    def send_goal(self, pickup_goal):
        rospy.loginfo("pickup_goal: " + str(pickup_goal))

        received_result = False
        try:
            self.pick_plan_client.wait_for_server(rospy.Duration(3))
            pickup_goal.planner_id = self.planner_id
            self.pick_plan_client.send_goal(pickup_goal)

            received_result = self.pick_plan_client.wait_for_result(rospy.Duration(self.planner_timeout))
        except Exception as e:
            rospy.logerr("failed to reach pick action server with err: %s" % e.message)

        if received_result:
            result = self.pick_plan_client.get_result()
            rospy.loginfo("result: " + str(result))
            success = result.error_code.val == result.error_code.SUCCESS
        else:
            result = None
            success = False

        rospy.loginfo("success of pick_plan_client:" + str(success))

        return success, result

    def pub_tfs(self, graspit_grasp_msg, moveit_grasp_msg):
        tf_pose = pm.toTf(pm.fromMsg(moveit_grasp_msg.grasp_pose.pose))
        self.tf_broadcaster.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), "/moveit_end_effector_frame",
                                          graspit_grasp_msg.object_name)

        tf_pose = pm.toTf(pm.fromMsg(graspit_grasp_msg.final_grasp_pose))
        self.tf_broadcaster.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), "/grasp_approach_tran",
                                          graspit_grasp_msg.object_name)

    def execute_pickup_plan(self, mgc_arm, mgc_gripper, pick_plan):

        # pick_plan.trajectory_descriptions
        # ['plan', 'pre_grasp', 'approach', 'grasp', 'retreat']

        for i in range(5):
            print(i)
            print(pick_plan.trajectory_stages[i])
            print()

            if i % 2 == 0:
                success = mgc_arm.execute(pick_plan.trajectory_stages[i])
            else:
                success = mgc_gripper.execute(pick_plan.trajectory_stages[i])

        return success

    def query_moveit_for_reachability(self, graspit_grasp_msg):
        """
        :type graspit_grasp_msg: graspit_msgs.msg.Grasp
        """

        # self.move_group.set_planning_time(rospy.get_param('~allowed_planning_time'))
        self.move_group.set_planning_time(self.allowed_planning_time)

        moveit_grasp_msg = message_utils.graspit_grasp_to_moveit_grasp(graspit_grasp_msg,                                                                       
                                                                       self.move_group,
                                                                       self.listener,
                                                                       self.grasp_approach_tran_frame)

        self.pub_tfs(graspit_grasp_msg, moveit_grasp_msg)

        rospy.loginfo("moveit_grasp_msg: " + str(moveit_grasp_msg))
        rospy.loginfo("Planning for %f seconds with planner %s" % (self.allowed_planning_time, self.planner_id))

        pickup_goal = message_utils.build_pickup_goal(moveit_grasp_msg=moveit_grasp_msg,
                                                      object_name=graspit_grasp_msg.object_name,
                                                      allowed_planning_time=self.allowed_planning_time,
                                                      planner_id=self.planner_id,
                                                      planning_group=self.move_group,
                                                      plan_only=True)

        # Adjust pickup_goal for errors


        import IPython
        IPython.embed()

        assert(False)

        return self.send_goal(pickup_goal)
