import rospy
import numpy as np


class ExecutionStage:

    def __init__(self, robot_interface):

        self.robot_interface = robot_interface
        self._success = True
        self._status_msg = "Success"

    def run(self, grasp_msg, pick_plan):
        pass

    def is_sucessful(self):
        return self._success

    def get_status_msg(self):
        return self._status_msg



class PreshapeSpreadAngle(ExecutionStage):

    def run(self, grasp_msg, pick_plan):
        self._success = self.robot_interface.hand_manager.hand_manager.move_hand([0, 0, 0, grasp_msg.pre_grasp_dof[0]])


class MoveToPreGraspPosition(ExecutionStage):

    def run(self, grasp_msg, pick_plan):
        self._success, self._status_msg, trajectory = self.robot_interface.run_pickup_trajectory(pick_plan, pickup_phase=0)


class PreshapeHand(ExecutionStage):

    def run(self, grasp_msg, pick_plan):
        self._success, self._status_msg = self.robot_interface.hand_manager.move_hand_trajectory(pick_plan.trajectory_stages[1].joint_trajectory)


class Approach(ExecutionStage):

    def run(self, grasp_msg, pick_plan):
        self._success, self._status_msg, trajectory = self.robot_interface.run_pickup_trajectory(pick_plan, pickup_phase=2)


class CloseHand(ExecutionStage):

    def run(self, grasp_msg, pick_plan):
        self._success, self._status_msg = self.robot_interface.hand_manager.move_hand_trajectory(pick_plan.trajectory_stages[3].joint_trajectory)

        if self._success:
            grasp_joint_state = self.robot_interface.hand_manager.joint_trajectory_to_joint_state(pick_plan.trajectory_stages[3].joint_trajectory, 0)
            self._success, self._status_msg, joint_angles = self.robot_interface.hand_manager.close_grasp(grasp_joint_state)

class OpenHand(ExecutionStage):

    def run(self, grasp_msg, pick_plan):
        self._success = self.robot_interface.hand_manager.open_hand()


class Lift(ExecutionStage):

    def run(self, grasp_msg, pick_plan):
        self._success, self._status_msg, trajectory_result = self.robot_interface.run_pickup_trajectory(pick_plan, pickup_phase=4)


class GraspExecutionPipeline():

    def __init__(self, robot_interface):

        self.stages = []

        #Shape Competion Pipeline
        self.stages.append(MoveToPreGraspPosition(robot_interface))
        self.stages.append(PreshapeHand(robot_interface))
        self.stages.append(Approach(robot_interface))
        self.stages.append(CloseHand(robot_interface))
        self.stages.append(Lift(robot_interface))

        #Visio-Tactile Pipeline:
        # self.stages.append(MoveToPreGraspPosition(robot_interface))
        # self.stages.append(PreshapeHand(robot_interface))
        # self.stages.append(Approach(robot_interface))
        # self.stages.append(CloseHand(robot_interface))


    def run(self, grasp_msg, pick_plan):
        status_msg = "Success"
        success = True

        for stage in self.stages:
            rospy.loginfo("Starting Execution Stage: " + stage.__class__.__name__)
            stage.run(grasp_msg, pick_plan)

            status_msg = stage.get_status_msg()
            success = stage.is_sucessful()

            if success:
                rospy.loginfo("Successfully Finished Execution Stage: " + stage.__class__.__name__)
            else:
                rospy.logerr("Execution Failed on stage: " +
                             stage.__class__.__name__ +
                             " with status: " +
                             str(status_msg))

                break

        return success, status_msg
