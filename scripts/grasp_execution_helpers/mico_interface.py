import rospy
import control_msgs.msg
import moveit_msgs.msg
import jaco_msgs.srv
import jaco_msgs.msg
import numpy as np
import math
import tf
import IPython
import copy

class RobotInterface:

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
        self.rotate_pub = rospy.Publisher('/mico_arm_driver/joint_angles/arm_joint_angles/goal', jaco_msgs.msg.ArmJointAnglesActionGoal, queue_size=10)

    def waypoints_translation(self, fraction=1.0, x_distance=0.0, y_distance=0.0, z_distance=0.0, server=None):

        def append_waypoints(distance, direction, waypoints, pose, fraction, step=0.05):
            steps = abs(int(distance/step))
            for i in range(steps):
                setattr(pose.position, direction, getattr(pose.position, direction) + (distance/steps) * fraction)
                # pose.position[direction] += (distance/steps) * fraction
                waypoints.append(copy.deepcopy(pose))

        def compute_waypoints(fraction, x_distance, y_distance, z_distance):
            pose = self.group.get_current_pose().pose
            waypoints = [copy.deepcopy(pose)]

            append_waypoints(x_distance, "x", waypoints, pose, fraction)
            append_waypoints(y_distance, "y", waypoints, pose, fraction)
            append_waypoints(z_distance, "z", waypoints, pose, fraction)

            return waypoints

        self.group.clear_pose_targets()

        waypoints = compute_waypoints(fraction=fraction, x_distance=x_distance, y_distance=y_distance, z_distance=z_distance)

        for waypoint in waypoints:
            print waypoint

        (plan, planned_fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # eef_step
            10,  # joint_jump
            avoid_collisions=False)

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        # print waypoints
        for i, point in enumerate(plan.joint_trajectory.points):
            if i == 0:
                point.velocities = tuple([0] * len(point.positions))
            end_time = point.time_from_start.secs + point.time_from_start.nsecs / 1000000000.0
            point.velocities = tuple(
                list((np.array(point.positions) - np.array(prev_point.positions)) / (end_time - start_time)))
            start_time = end_time
            prev_point = point

        if server and server.is_preempt_requested():
            server.set_preempted()
            return False

        success = True
        if planned_fraction * fraction == 1:
            rospy.loginfo("Can execute entire waypoint translation")
        else:
            rospy.loginfo("Only " + str(float(int(planned_fraction * fraction * 1000))/10) + "% of the waypoint can be executed (" + str(z_distance * planned_fraction * fraction) + " meters of " + str(z_distance) + " meters)")
        self.group.execute(plan)

        return success, planned_fraction

    def rotate_base(self, rotation):
        self.group.clear_pose_targets()
        group_variable_values = self.group.get_current_joint_values()
        # goal = jaco_msgs.msg.ArmJointAnglesActionGoal()
        # goal.goal.angles.joint1 = group_variable_values[0] + rotation#-1.8603088994580952
        # goal.goal.angles.joint2 = group_variable_values[1]#-1.6535711019001942
        # goal.goal.angles.joint3 = group_variable_values[2]#0.1944244748554111
        # goal.goal.angles.joint4 = group_variable_values[3]#-1.101937495822704
        # goal.goal.angles.joint5 = group_variable_values[4]#1.6731360148484626
        # goal.goal.angles.joint6 = group_variable_values[5]#-2.8774134650407386
        # goal.goal_id.id = '420'
        # rospy.loginfo("Rotating base joint by " + str(rotation) + " to " + str(goal.goal.angles.joint1))
        # self.rotate_pub.publish(goal)

        rospy.loginfo("Current rotation values: " + str(group_variable_values))
        group_variable_values[0] += rotation
        rospy.loginfo("New rotation values: " + str(group_variable_values))
        # self.group.set_joint_value_target(group_variable_values)

        # This executes command???
        plan = self.group.plan()
        # success = self.group.go(joints=group_variable_values, wait=True)
        success = self.group.execute(plan, wait=True)
        rospy.loginfo("Result of rotation: " + str(success))

        return True

    def move_object(self, server=None):
        #FOR HARDCODING A PATH THAT RAISES THE END EFFECTOR, ROTATES BASE JOINT, and LOWERS END EFFECTOR:

        rospy.loginfo("Waypoint translation up by 0.10")
        # IPython.embed()
        # self.open_hand_and_go_home()
        # return True
        # success, fraction = self.waypoints_translation(z_distance=0.10, server=server)

        rospy.loginfo("Waypoint translation right by 0.30 and down by 0.05")
        pose = self.group.get_current_pose().pose
        x_distance = -0.201656975107 - pose.position.x
        z_distance = 0.275514443391 - pose.position.z
        x_distance = -0.36
        rospy.loginfo("Moving from %f to %f" % (pose.position.x, x_distance - pose.position.x))
        success, fraction = self.waypoints_translation(z_distance=0.05, server=server)
        success, fraction = self.waypoints_translation(x_distance=x_distance, z_distance=-0.1, server=server)
        rospy.loginfo("Final position: " + str(self.group.get_current_pose().pose))

        # rospy.loginfo("Waypoint translation down by 0.10")
        # success, fraction = self.waypoints_translation( fraction=fraction, server=server)

        self.open_hand_and_go_home()

        return True

    def run_pickup_trajectory(self, pickup_result, pickup_phase):
        """
        :type pickup_phase: int
        :type pickup_result: moveit_msgs.msg.PickupResult
        """

        robot_trajectory_msg = pickup_result.trajectory_stages[pickup_phase]
        trajectory_msg = robot_trajectory_msg.joint_trajectory

        success, error_msg, trajectory_result = self.run_trajectory(trajectory_msg)
        return success, error_msg, trajectory_result

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

    def generate_pick_plan(self, grasp_msg):
        """
        :type grasp_msg: graspit_msgs.msg.Grasp
        """
        success = False
        pick_plan_result = None

        for i in xrange(100):
            success, pick_plan_result = self.grasp_reachability_analyzer.query_moveit_for_reachability(grasp_msg)
            # IPython.embed()
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

    def put_object_down(self, server=None):
        success = True
        self.cartesian_translation("z", "negative", server)
        self.hand_manager.open_hand()
        self.cartesian_translation("z", "positive", server)
        self.cartesian_translation("z", "positive", server)
        self.home_arm(True)
        return success

    def cartesian_translation(self, axis, direction, server=None):
        # waypoints = []

        # start with the current pose
        pose = self.group.get_current_pose()
        print ("got current pose! \n")
        print pose
        if direction == "positive":
            change = 0.05
        else:
            change = -0.05

        if axis == "x":
            pose.pose.position.x += change
        elif axis == "y":
            pose.pose.position.y += change
        else:
            pose.pose.position.z += change

        # DO NOT DELETE!! LOOKS STUPID BUT IS NEEDED TO RESET AND FIX UNIDENTIFIED BUG!!!
        self.home_arm(execute=False)
        if server and server.is_preempt_requested():
            server.set_preempted()
            return False

        self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 20))
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose)
        plan = self.group.plan()
        if server and server.is_preempt_requested():
            server.set_preempted()
            return False
        success = self.group.execute(plan)
        return success

    def home_arm(self, execute=True):

        def get_plan():
            success = True
            self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 10))
            self.group.set_start_state_to_current_state()
            self.group.set_named_target("home")
            plan = self.group.plan()

            if len(plan.joint_trajectory.points) == 0:
                rospy.logerr("Failed to plan path home.")
                success = False

            return success, plan

        def is_already_at_goal(plan):
            current_joint_values = self.group.get_current_joint_values()
            final_joint_values = plan.joint_trajectory.points[-1].positions

            rospy.loginfo("current_joint_values: " + str(current_joint_values))
            rospy.loginfo("final_joint_values: " + str(final_joint_values))

            already_home = True
            threshold = 0.05
            for i, (current, final) in enumerate(zip(current_joint_values, final_joint_values)):
                difference = abs(current-final) % math.pi
                rospy.loginfo("joint_value " + str(i) + " difference: " + str(difference))
                if difference > threshold:
                    already_home = False
                    break

            return already_home

        def is_near_goal(plan):
            current_joint_values = self.group.get_current_joint_values()
            final_joint_values = plan.joint_trajectory.points[-1].positions

            rospy.loginfo("current_joint_values: " + str(current_joint_values))
            rospy.loginfo("final_joint_values: " + str(final_joint_values))

            near_home = True
            threshold = 0.5
            for i, (current, final) in enumerate(zip(current_joint_values, final_joint_values)):
                if i > 4:
                    break
                difference = abs(current-final) % (math.pi)
                if i == 3:
                    difference2 = abs(difference - (math.pi))
                    if difference2 < difference:
                        difference = difference2
                rospy.loginfo("joint_value " + str(i) + " difference: " + str(difference))
                if difference > threshold:
                    near_home = False
                    break

            return near_home

        success, plan = get_plan()
        if not execute:
            return success

        if not success:
            #Force it
            gohome = rospy.ServiceProxy('/mico_arm_driver/in/home_arm', jaco_msgs.srv.HomeArm)
            gohome()
            return True
        else:
            if is_already_at_goal(plan):
                rospy.loginfo("Arm is already home, no need to home it.")
            elif is_near_goal(plan):
                rospy.loginfo("MICO Arm is close to home, no need to plan path.")
                gohome = rospy.ServiceProxy('/mico_arm_driver/in/home_arm', jaco_msgs.srv.HomeArm)
                gohome()
            else:
                rospy.loginfo("Arm is not home, so homing it")
                success = self.group.execute(plan)
                rospy.loginfo("Arm homed with success: " + str(success))

        if not success:
            gohome = rospy.ServiceProxy('/mico_arm_driver/in/home_arm', jaco_msgs.srv.HomeArm)
            gohome()
            return True

        return success


    def open_hand_and_go_home(self):
        self.hand_manager.open_hand()
        self.home_arm()
        return True

    def manual_move(self, axis, direction, distance=0.05, server=None):
        """ axis: x = 0, y = 1, z = 2, roll = 3, pitch = 4, yaw = 5; direction: positive = 1, negative = -1 """

        value = distance * direction
        if axis > 2:
            value = ((math.pi)/16) * direction
        # self.group.clear_pose_targets()
        # self.home_arm(execute=False)
        # if server and server.is_preempt_requested():
        #     server.set_preempted()
        #     return False
        self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 5))
        self.group.set_start_state_to_current_state()
        self.group.set_pose_reference_frame("/root")
        self.group.shift_pose_target(axis, value)
        plan = self.group.plan()
        if server and server.is_preempt_requested():
            server.set_preempted()
            return False
        success = self.group.execute(plan)
        return success