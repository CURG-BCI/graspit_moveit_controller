#!/usr/bin/env python
PKG = 'test_roslaunch'

import sys
import unittest
import time

#additional imports for the service node
import roslib; roslib.load_manifest('mico_moveit')
import rospy
from moveit_trajectory_planner.srv import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg


class TestTrajectoryPlanner(unittest.TestCase):
	def test_add_object(self):
		rospy.wait_for_service('moveit_trajectory_planner/add_box')

		moveit_commander.roscpp_initialize(sys.argv)
		robot = moveit_commander.RobotCommander()

		boxPose = geometry_msgs.msg.PoseStamped()
		boxPose.header.frame_id = robot.get_planning_frame()
		boxPose.pose.position.x = -0.2
		boxPose.pose.position.y = 0
		boxPose.pose.position.z = 0.6
		boxXDimension = 0.2
		boxYDimension = 1
		boxZDimension = 0.2

		add_box = rospy.ServiceProxy('moveit_trajectory_planner/add_box', BoxInfo)
		add_box(boxPose, boxXDimension, boxYDimension, boxZDimension, "box1")

		self.assertTrue(True)

	def test_add_mesh(self):
		rospy.wait_for_service('moveit_trajectory_planner/add_autoscaled_mesh')

		moveit_commander.roscpp_initialize(sys.argv)
		robot = moveit_commander.RobotCommander()

		meshPose = geometry_msgs.msg.PoseStamped()
		meshPose.header.frame_id = robot.get_planning_frame()
		meshPose.pose.position.x = -1.0
		meshPose.pose.position.y = 0
		meshPose.pose.position.z = 0.6

		add_autoscaled_mesh = rospy.ServiceProxy('moveit_trajectory_planner/add_autoscaled_mesh', MeshInfo)

		add_autoscaled_mesh(meshPose, roslib.packages.get_pkg_dir('object_models') + "/models/cgdb/model_database/clorox.ply", "mesh1")

		self.assertTrue(True)

	def test_remove_object(self):
		rospy.wait_for_service('moveit_trajectory_planner/add_box')
		rospy.wait_for_service('moveit_trajectory_planner/remove_object')

		moveit_commander.roscpp_initialize(sys.argv)
		robot = moveit_commander.RobotCommander()

		boxPose = geometry_msgs.msg.PoseStamped()
		boxPose.header.frame_id = robot.get_planning_frame()
		boxPose.pose.position.x = -0.2
		boxPose.pose.position.y = 0
		boxPose.pose.position.z = 0.6
		boxXDimension = 0.2
		boxYDimension = 1
		boxZDimension = 0.2

		add_box = rospy.ServiceProxy('moveit_trajectory_planner/add_box', BoxInfo)
		add_box(boxPose, boxXDimension, boxYDimension, boxZDimension, "box1")

		remove_object = rospy.ServiceProxy('moveit_trajectory_planner/remove_object', ObjectName)
		remove_object("box1")
		self.assertTrue(True)


	def test_movable_location(self):
		rospy.wait_for_service('moveit_trajectory_planner/check_reachability')
		
		robot = moveit_commander.RobotCommander()

		pose_target = geometry_msgs.msg.PoseStamped()
		pose_target.header.frame_id = robot.get_planning_frame()
		pose_target.pose.position.x = -0.395823069744
		pose_target.pose.position.y = 0.147705986571
		pose_target.pose.position.z = 0.450246479896
		pose_target.pose.orientation.x = 0.530314113123
		pose_target.pose.orientation.y = 0.24583699541
		pose_target.pose.orientation.z = 0.294578916148
		pose_target.pose.orientation.w = 0.756012152859
		
		check_reachability = rospy.ServiceProxy('moveit_trajectory_planner/check_reachability', LocationInfo)
		resp1 = check_reachability(pose_target)

		self.assertTrue(resp1.isPossible)


	def test_unmovable_location(self):
		rospy.wait_for_service('moveit_trajectory_planner/add_box')
		rospy.wait_for_service('moveit_trajectory_planner/check_reachability')
		
		robot = moveit_commander.RobotCommander()

		pose_target = geometry_msgs.msg.PoseStamped()
		pose_target.header.frame_id = robot.get_planning_frame()
		pose_target.pose.position.x = -0.395823069744
		pose_target.pose.position.y = 0.147705986571
		pose_target.pose.position.z = 0.450246479896
		pose_target.pose.orientation.x = 0.530314113123
		pose_target.pose.orientation.y = 0.24583699541
		pose_target.pose.orientation.z = 0.294578916148
		pose_target.pose.orientation.w = 0.756012152859
		

		boxPose = geometry_msgs.msg.PoseStamped()
		boxPose.header.frame_id = robot.get_planning_frame()
		boxPose.pose.position.x = pose_target.pose.position.x
		boxPose.pose.position.y = pose_target.pose.position.y
		boxPose.pose.position.z = pose_target.pose.position.z
		boxXDimension = 0.2
		boxYDimension = 0.2
		boxZDimension = 0.2

		add_box = rospy.ServiceProxy('moveit_trajectory_planner/add_box', BoxInfo)
		add_box(boxPose, boxXDimension, boxYDimension, boxZDimension, "box1")


		check_reachability = rospy.ServiceProxy('moveit_trajectory_planner/check_reachability', LocationInfo)
		resp1 = check_reachability(pose_target)

		self.assertFalse(resp1.isPossible)

if __name__ == '__main__':
	import rostest

	time.sleep(10) #wait until moveit is loaded
	rostest.rosrun(PKG, 'test_trajectory_planner', TestTrajectoryPlanner) 