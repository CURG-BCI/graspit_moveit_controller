import rospy
import execution_stages


class GraspExecutionPipeline:

    def __init__(self, robot_interface, stage_names):

        self.stages = [getattr(execution_stages, s)(robot_interface) for s in stage_names]

    def run(self, grasp_msg, pick_plan, execution_as):
        status_msg = "Success"
        success = True

        for stage in self.stages:
            if execution_as.is_preempt_requested():
                rospy.loginfo("Preempted")
                execution_as.set_preempted()
                execution_as.set_succeeded(False)
                return False, "Preempt requested"
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

                return success, status_msg

        return success, status_msg
