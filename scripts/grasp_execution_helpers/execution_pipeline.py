import rospy
import execution_stages

class GraspExecutionPipeline():

    def __init__(self, robot_interface, stage_names):

        self.stages = [getattr(execution_stages, s)(robot_interface) for s in stage_names]

        #Shape Competion Pipeline
        #self.stages.append(MoveToPreGraspPosition(robot_interface))
        #self.stages.append(PreshapeHand(robot_interface))
        #self.stages.append(Approach(robot_interface))
        #self.stages.append(CloseHand(robot_interface))
        #self.stages.append(Lift(robot_interface))

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
