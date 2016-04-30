import rospy
import control_msgs.msg
import moveit_msgs.msg


class RobotInterface():

    def __init__(self,
                 trajectory_action_client,
                 display_trajectory_publisher,
                 hand_manager,
                 group,
                 grasp_reachability_analyzer):

        self.trajectory_action_client = trajectory_action_client
        self.display_trajectory_publisher = display_trajectory_publisher
        self.hand_manager = hand_manager
        self.group = group
        self.grasp_reachability_analyzer = grasp_reachability_analyzer

    def run_trajectory(self, trajectory_msg):
        """
        :type trajectory_msg: control_msgs.msg.FollowJointTrajectoryResult
        """

        if not self.trajectory_action_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr('Failed to find trajectory server')
            return False, 'Failed to find trajectory server', []

        trajectory_action_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        trajectory_action_goal.trajectory = trajectory_msg
        self.trajectory_action_client.send_goal(trajectory_action_goal)

        if not self.trajectory_action_client.wait_for_result():
            rospy.logerr("Failed to execute trajectory")
            return False, "Failed to execute trajectory", []

        trajectory_result = self.trajectory_action_client.get_result()

        success = (trajectory_result.SUCCESSFUL == trajectory_result.error_code)
        if not success:
            return False, "Failed to follow trajectory", trajectory_result

        return success, None, trajectory_result

    def run_pickup_trajectory(self, pickup_result, pickup_phase):
        """
        :type pickup_phase: int
        :type pickup_result: moveit_msgs.msg.PickupResult
        """

        robot_trajectory_msg = pickup_result.trajectory_stages[pickup_phase]
        trajectory_msg = robot_trajectory_msg.joint_trajectory

        success, error_msg, trajectory_result = self.run_trajectory(trajectory_msg)
        return success, error_msg, trajectory_result

    def display_trajectory(self, trajectory_msg):
        """
        :type trajectory_msg: moveit_msgs.msg.trajectory_msg
        """
        display_msg = moveit_msgs.msg.DisplayTrajectory()
        display_msg.trajectory = trajectory_msg
        display_msg.model_id = self.group.get_name()
        self.display_trajectory_publisher.publish(display_msg)

    def generate_pick_plan(self, grasp_msg):
        """
        :type grasp_msg: graspit_msgs.msg.Grasp
        """
        success = False
        pick_plan_result = None

        for i in xrange(10):
            success, pick_plan_result = self.grasp_reachability_analyzer.query_moveit_for_reachability(grasp_msg)
            if success:
                if (pick_plan_result.trajectory_stages[0].joint_trajectory.points[0].positions !=
                        pick_plan_result.trajectory_stages[2].joint_trajectory.points[-1].positions):

                    stage2_start = pick_plan_result.trajectory_stages[2].joint_trajectory.points[0]
                    pick_plan_result.trajectory_stages[0].joint_trajectory.points.append(stage2_start)
                    rospy.loginfo('Stage 0 and Stage 2 were misaligned. added missing trajectory point')

                break
            else:
                rospy.loginfo("Planning attempt %i Failed" % i)

        return success, pick_plan_result

    def home_arm(self):

        self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 20))
        self.group.set_start_state_to_current_state()
        self.group.set_named_target("home")

        plan = self.group.plan()

        success = self.group.execute(plan)

        return success
