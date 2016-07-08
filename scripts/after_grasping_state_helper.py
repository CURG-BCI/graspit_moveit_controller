#!/usr/bin/env python
# coding: utf-8
import actionlib
import graspit_msgs.srv
import graspit_msgs.msg
import rospy
from grasp_execution_node import GraspExecutionNode


class robotmovement():

	def __init__(self):

		self.ge = GraspExecutionNode("after_grasping_state_helper", manual_mode=True)
		self.after_grasping = actionlib.SimpleActionServer("after_grasping_action",
                                                        graspit_msgs.msg.AfterGraspingAction,
                                                        execute_cb=self._after_grasping_cb,
                                                        auto_start=False)


	def _after_grasping_cb(self, goal):
		action_type = goal.action

		print("ACTION REQUESTED: " + action_type)

		if action_type == "moveobject":
			completed = self.move_object()
		else:
			completed = self.go_home()

		_result = graspit_msgs.msg.AfterGraspingResult()
		_result.success = completed
		self._after_grasping.set_succeeded(_result)
		return []

	def go_home(self):
		self.ge.robot_interface.hand_manager.open_hand()
		self.ge.robot_interface.home_arm()
		return True

	def move_object(self):
		self.ge.robot_interface.group.set_start_state_to_current_state()
		self.ge.robot_interface.group.set_named_target("dropoff")
		plan = self.ge.robot_interface.group.plan()
		self.ge.robot_interface.group.execute(plan)
		self.ge.robot_interface.hand_manager.open_hand()
		self.ge.robot_interface.home_arm()
		return True


if __name__ == '__main__':

	instance = robotmovement()
	instance.after_grasping.start()


# http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html