
from common_helpers import message_utils
import rospy

import moveit_msgs.msg
import actionlib
import moveit_commander
import geometry_msgs.msg
import ipdb
import tf
import tf_conversions.posemath as pm

class GraspReachabilityAnalyzer():

    def __init__(self, move_group, grasp_approach_tran_frame):
        """
        :type move_group: moveit_commander.MoveGroupCommander
        """
        self.pick_plan_client = actionlib.SimpleActionClient('/pickup', moveit_msgs.msg.PickupAction)
        self.move_group = move_group
        #self.move_group.set_workspace([-1.25, -.5, -.4, .25, .5, 1.6])
        self.planner_id = 'SBLkConfigDefault'
        self.grasp_approach_tran_frame = grasp_approach_tran_frame
        self.grasp_dict = {}
        self.listener = tf.TransformListener()
        self.has_msg = False

    def query_moveit_for_reachability(self, graspit_grasp_msg):
        """
        :type graspit_grasp_msg: graspit_msgs.msg.Grasp
        """
        #ipdb.set_trace()
        #return self.pose_reachability_checker(graspit_grasp_msg.final_grasp_pose, graspit_grasp_msg.object_name)

        moveit_grasp_msg = message_utils.graspit_grasp_to_moveit_grasp(graspit_grasp_msg,                                                                       
                                                                       self.move_group,
                                                                       self.listener,
                                                                       self.grasp_approach_tran_frame)
        self.grasp_dict[moveit_grasp_msg.id] = moveit_grasp_msg
        rospy.loginfo("moveit_grasp_msg: " + str(moveit_grasp_msg))

        if rospy.get_param('/debug', 0) != 0:
            tf_msg = pm.toTf(pm.fromMsg(moveit_grasp_msg.grasp_pose.pose))
            bc = tf.TransformBroadcaster()
            bc.sendTransform(tf_msg[0], tf_msg[1], rospy.Time.now(), 'moveit_msg_grasp_pose', moveit_grasp_msg.grasp_pose.header.frame_id)
            tf_msg = pm.toTf(pm.fromMsg(graspit_grasp_msg.pre_grasp_pose))
            bc.sendTransform(tf_msg[0], tf_msg[1], rospy.Time.now(), 'graspit_msg_pre_grasp_pose', graspit_grasp_msg.object_name)
            #moveit_grasp_msg.grasp_pose = self.move_group.get_current_pose()
        else:
            pass
            #moveit_grasp_msg.grasp_pose.pose = graspit_grasp_msg.pre_grasp_pose

        pickup_goal = message_utils.build_pickup_goal(moveit_grasp_msg=moveit_grasp_msg,
                                                      object_name=graspit_grasp_msg.object_name,
                                                      planning_group=self.move_group)
        # if not self.has_msg:
        #     self.has_msg = True
        #     rospy.loginfo("Entering IPython")
        #     import IPython
        #     IPython.embed()

        pick_attempt_succeeded = False
        #try:
        #    pick_attempt_succeeded = self.move_group.pick(graspit_grasp_msg.object_name, [moveit_grasp_msg])
        #
        #except Exception as e:
        #    rospy.logerr("pickup attempt failed: %s"%(e.message))
        #rospy.loginfo("pickup attempt result: %s"%(str(pick_attempt_succeeded)))

        rospy.loginfo("pickup_goal: " + str(pickup_goal))


        received_result = False
        try:
            self.pick_plan_client.wait_for_server(rospy.Duration(3))
            pickup_goal.planner_id = self.planner_id
            self.pick_plan_client.send_goal(pickup_goal)

            received_result = self.pick_plan_client.wait_for_result(rospy.Duration(rospy.get_param('~allowed_planning_time', 20)))
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

    def pose_reachability_checker(self, palm_pose, object_name):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose = palm_pose
        pose.header.frame_id = object_name
        self.move_group.set_pose_target(pose)
        plan = self.move_group.plan()

        if not plan.joint_trajectory.joint_names:
            return False, plan #this means the list is empty and therefore there is no path
        else:
            return True, plan

