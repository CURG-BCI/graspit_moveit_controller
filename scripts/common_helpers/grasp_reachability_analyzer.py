
from common_helpers import message_utils
import rospy

import moveit_msgs.msg
import actionlib
import moveit_commander
import tf_conversions.posemath as pm
from visualization_msgs.msg import Marker
import numpy as np
import tf


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
        self.markerPub = rospy.Publisher('analyzed_grasp_location', Marker, queue_size=10)

    def display_grasp_marker(self, grasp):
        """
        :type grasp: graspit_msgs.msg.Grasp
        """
        grasp_marker = Marker()
        grasp_marker.pose = grasp.final_grasp_pose
        grasp_marker.type = 0
        grasp_marker.action = 0
        grasp_marker.id = grasp.grasp_id
        grasp_marker.header.frame_id = grasp.object_name
        grasp_marker.header.stamp = rospy.get_rostime()

        pose_frame = pm.fromMsg(grasp_marker.pose)
        pose_mat = pm.toMatrix(pose_frame)

        # rotate to keep moveit world consistent with graspit world
        rot = np.identity(4)
        rot[0][0] = -1
        rot[2][2] = -1

        pmat_new = np.dot(pose_mat, rot)
        pmat_new_frame = pm.fromMatrix(pmat_new)
        pmat_msg = pm.toMsg(pmat_new_frame)

        pmat_msg.position.z += 0.05

        # grasp_marker.pose = pmat_msg

        grasp_marker.scale.x = 0.05
        grasp_marker.scale.y = 0.05
        grasp_marker.scale.z = 0.05

        grasp_marker.color.r = 0.0
        grasp_marker.color.g = 1.0
        grasp_marker.color.b = 0.0
        grasp_marker.color.a = 1.0

        grasp_marker.lifetime = rospy.Duration(0)

        self.markerPub.publish(grasp_marker)


    def send_goal(self, pickup_goal):
        rospy.loginfo("pickup_goal: " + str(pickup_goal))

        received_result = False
        try:
            self.pick_plan_client.wait_for_server(rospy.Duration(3))
            pickup_goal.planner_id = self.planner_id
            self.pick_plan_client.send_goal(pickup_goal)

            # received_result = self.pick_plan_client.wait_for_result(rospy.Duration(rospy.get_param('~allowed_planning_time', 7)))
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


    def query_moveit_for_reachability(self, graspit_grasp_msg):
        """
        :type graspit_grasp_msg: graspit_msgs.msg.Grasp
        """
        # self.move_group.set_planning_time(rospy.get_param('~allowed_planning_time'))
        self.move_group.set_planning_time(self.allowed_planning_time)

        self.display_grasp_marker(graspit_grasp_msg)

        moveit_grasp_msg = message_utils.graspit_grasp_to_moveit_grasp(graspit_grasp_msg,                                                                       
                                                                       self.move_group,
                                                                       self.listener,
                                                                       self.grasp_approach_tran_frame)
        rospy.loginfo("moveit_grasp_msg: " + str(moveit_grasp_msg))
        rospy.loginfo("Planning for %f seconds with planner %s" % (self.allowed_planning_time, self.planner_id))

        pickup_goal = message_utils.build_pickup_goal(moveit_grasp_msg=moveit_grasp_msg,
                                                      object_name=graspit_grasp_msg.object_name,
                                                      allowed_planning_time=self.allowed_planning_time,
                                                      planner_id=self.planner_id,
                                                      planning_group=self.move_group)

        import IPython
        IPython.embed()

        return self.send_goal(pickup_goal)
