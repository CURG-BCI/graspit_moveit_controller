import rospy
import control_msgs.msg
import moveit_msgs.msg
import jaco_msgs.srv
import numpy as np
import math
import tf
import IPython


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
            return success
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

    def open_hand_and_go_home(self):
        self.hand_manager.open_hand()
        self.home_arm()
        return True

    def move_object(self, server=None):
        #FOR HARDCODING A PATH THAT RAISES THE END EFFECTOR, ROTATES BASE JOINT, and LOWERS END EFFECTOR:

        print("Before anything")

        #This moves end effector up
        self.group.clear_pose_targets()
        self.waypoints_translation(-0.4, server=server)

        print("First waypoints translation")

        #This executes command???
        # plan = self.group.plan()
        # self.group.execute(plan)

        print("Executed")

        #Rotation
        self.group.clear_pose_targets()
        group_variable_values = self.group.get_current_joint_values()
        group_variable_values[0] += 0.5
        self.group.set_joint_value_target(group_variable_values)

        print("Base rotation")

        #This executes command???
        plan = self.group.plan()
        self.group.go()
        # self.group.execute(plan)

        print("Executed")

        #This moves end efector down
        self.group.clear_pose_targets()
        self.waypoints_translation(0.4, server=server)

        print("Second waypoints translation")

        #This executes command???
        # plan = self.group.plan()
        # self.group.execute(plan)

        print("Executed")

        self.open_hand_and_go_home()

        print("Open hand and go home")

        #This executes command???
        # plan = self.group.plan()
        # self.group.execute(plan)

        print("Executed")

        return True

        #FOR PLANNING A PATH FROM CURRENT POSITION TO SET END GOAL:

        # self.group.set_start_state_to_current_state()
        # pose = self.group.get_current_pose()
        # pose.pose.position.x = -0.0271767654181
        # pose.pose.position.y = -0.23098809043
        # pose.pose.position.z = 0.199946972128
        # pose.pose.orientation.x = 0.226220924638
        # pose.pose.orientation.y = -0.666850427277
        # pose.pose.orientation.z = -0.665674832609
        # pose.pose.orientation.w = 0.247005299795
        #
        # self.group.set_planner_id("PRMkConfigDefault")
        # self.group.set_pose_target(pose)
        # plan = self.group.plan()
        # if server and server.is_preempt_requested():
        #     server.set_preempted()
        #     return False
        # success = self.group.execute(plan)
        # # IPython.embed()
        # print(success)
        #
        # self.open_hand_and_go_home()
        # return True

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

    def put_object_down(self, server=None):
        success = True
        self.cartesian_translation("z","negative", server)
        self.hand_manager.open_hand()
        self.cartesian_translation("z", "positive", server)
        self.cartesian_translation("z", "positive", server)
        self.home_arm(True)
        return success

    def cartesian_translation2(self, axis, direction, server=None):
        success = True
        change = 0.05 * float(direction)
        changes = [0, 0, 0]
        for i in range(len(changes)):
            if axis == i:
                changes[i] = change

        if server and server.is_preempt_requested():
            server.set_preempted()
            return False

        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))
        print pose

        # first orient gripper and move forward (+z)
        wpose = copy.deepcopy(pose.pose)

        nsteps = 3
        for i in range(nsteps):
            wpose.position.x += changes[0]/nsteps
            wpose.position.y += changes[1]/nsteps
            wpose.position.z += changes[2]/nsteps
            waypoints.append(copy.deepcopy(wpose))

        self.group.set_planning_time(60)
        (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.000001)           # joint_jump

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        print waypoints
        print fraction
        for i, point in enumerate(plan.joint_trajectory.points):
            if i == 0:
                point.velocities = tuple([0]*len(point.positions))
            end_time = point.time_from_start.secs + point.time_from_start.nsecs / 1000000000.0
            point.velocities =tuple(list( (np.array(point.positions) - np.array(prev_point.positions))/(end_time-start_time)))
            start_time = end_time
            prev_point = point

        if server and server.is_preempt_requested():
            server.set_preempted()
            return False
        self.group.execute(plan)
        # self.group.go()
        if fraction != 1:
            success = False

        return success

    def up_turn_and_down(self, server=None):
        success_up = self.new_dropoff_up()
        if success_up:
            print("lifted up")
            success_turn = self.new_dropoff_turn()
            if success_turn:
                print("turned")
                self.new_dropoff_down()

    def new_dropoff_up(self, server=None):
        success = True
        pose = self.group.get_current_pose()
        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))
        # print pose

        z0 = pose.pose.position.z

        wpose = copy.deepcopy(pose.pose)

        # print quaternions
        for i in range(5):
            wpose.position.z += 0.016
            wpose.position.y -= 0.02
            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # eef_step
            10,  # joint_jump
            avoid_collisions=False)

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        # print waypoints
        print fraction
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

        if fraction != 1:
            success = False
        else:
            self.group.execute(plan)

        return success

    def new_dropoff_turn(self, server=None):
        success = self._new_dropoff_turn(longway=False, server=server)
        if not success:
            print("Need to try longway")
            success = self._new_dropoff_turn(longway=True, server=server)

    def _new_dropoff_turn(self, longway=False, server=None):
        success = True

        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))
        # print pose

        x0 = pose.pose.position.x
        y0 = pose.pose.position.y
        z0 = pose.pose.position.z

        x1 = 0.352748511959
        y1 = -0.21911669887
        z1 = 0.10981918533
        q1 = [0.495242102741, -0.504267995644, -0.486711373379, 0.513382009098]

        quaternion0 = pose.pose.orientation
        q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]

        wpose = copy.deepcopy(pose.pose)

        midpoint = tf.transformations.quaternion_slerp(q0, q1, 0.5)
        print midpoint

        if longway:
            midpoint = tf.transformations.quaternion_multiply([0, 0, 1, 0], midpoint)

        nsteps = 10
        quaternions = [tf.transformations.quaternion_slerp(q0, midpoint, i * (1.0 / float(nsteps / 2))) for i in
                       range(nsteps / 2)]
        quaternions += [tf.transformations.quaternion_slerp(midpoint, q1, i * (1.0 / float(nsteps / 2))) for i in
                        range(nsteps / 2)]
        # print quaternions
        for i in range(nsteps):
            s = i * (1.0 / float(nsteps))
            wpose.position.x = (1 - s) * x0 + s * x1
            wpose.position.y = (1 - s) * y0 + s * y1

            orientation = quaternions[i]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]

            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # eef_step
            10,  # joint_jump
            avoid_collisions=False)

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        # print waypoints
        print fraction
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

        if fraction != 1:
            success = False
        else:
            self.group.execute(plan)

        return success

    def new_dropoff_down(self, server=None):
        success = True
        pose = self.group.get_current_pose()
        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))
        # print pose

        z0 = pose.pose.position.z

        wpose = copy.deepcopy(pose.pose)

        # print quaternions
        for i in range(8):
            wpose.position.z -= 0.02
            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # eef_step
            10,  # joint_jump
            avoid_collisions=False)

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        # print waypoints
        print fraction
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

        if fraction != 1:
            success = False
        else:
            success_executed = self.group.execute(plan)

        return success_executed

    def waypoints_translation(self, z_distance, server=None):
        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))
        # print pose
        wpose = copy.deepcopy(pose.pose)
        nsteps = abs(int(z_distance/0.05))

        for i in range(nsteps):
            wpose.position.z += (z_distance/nsteps)

            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # eef_step
            10,  # joint_jump
            avoid_collisions=False)

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        # print waypoints
        print fraction
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
        # if fraction != 1:
        #     success = False
        # else:
        self.group.execute(plan)

        return success

    def dropoff2(self, server=None):
        success = self._dropoff2(longway=False, server=server)
        if not success:
            print("Need to try longway")
            success = self._dropoff2(longway=True, server=server)

        return success

    def _dropoff2(self, longway=False, server=None):
        """x: 0.218732042971
        y: -0.36935908641
        z: 0.313392472227  orientation: x: 0.509653072671  y: 0.488239530881  z: 0.495386669354  w: 0.506426651972"""

        success = True

        frame = self.group.get_pose_reference_frame()

        self.group.set_planner_id("RRTConnectkConfigDefault")  # ("RRTConnectkConfigDefault") ("RRTkConfigDefault") ("KPIECEkConfigDefault")
        # BKPIECEkConfigDefault   LBKPIECEkConfigDefault   PRMkConfigDefault   PRMstarkConfigDefault   RRTConnectkConfigPercise
        # RRTstarkConfigDefault   SBLkConfigDefault2   SBLkConfigDefault   TRRTkConfigDefault

        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))
        # print pose

        x0 = pose.pose.position.x
        y0 = pose.pose.position.y
        z0 = pose.pose.position.z

        # x1 = 0.0843734169772
        # y1 = -0.297149433291
        # z1 = 0.285390054635
        # q1 = [0.277958852453, -0.665224278085, -0.620185033681, 0.309169953566]
        x1 = 0.352748511959
        y1 = -0.21911669887
        z1 = 0.10981918533
        q1 = [0.495242102741, -0.504267995644, -0.486711373379, 0.513382009098]


        quaternion0 = pose.pose.orientation
        q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]

        height = 0.22
        if z0 > z1:
            height = 0.01
            a = -2*((height*(height+z0-z1))**0.5) - 2*height - z0 + z1
            b = 2*(((height*(height + z0 - z1))**0.5) + height)
        else:
            a = -2*((height*(height - z0 + z1))**0.5) - 2*height + z0 - z1
            b = 2*(((height*(height - z0 + z1))**0.5) + height - z0 + z1)

        wpose = copy.deepcopy(pose.pose)

        midpoint = tf.transformations.quaternion_slerp(q0, q1, 0.5)
        print midpoint

        if longway:
            midpoint = tf.transformations.quaternion_multiply([0,0,1,0], midpoint)

        nsteps = 10
        quaternions = [tf.transformations.quaternion_slerp(q0, midpoint, i*(1.0/float(nsteps/2))) for i in range(nsteps/2)]
        quaternions += [tf.transformations.quaternion_slerp(midpoint, q1, i*(1.0/float(nsteps/2))) for i in range(nsteps/2)]
        # print quaternions
        for i in range(nsteps):
            s = i*(1.0/float(nsteps))
            wpose.position.x = (1-s)*x0 + s*x1
            wpose.position.y = (1-s)*y0 + s*y1
            wpose.position.z = a*(s**2) + b*s + z0

            orientation = quaternions[i]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]

            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.1,        # eef_step
                             10,          # joint_jump
                             avoid_collisions = False)           

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        # print waypoints
        print fraction
        for i, point in enumerate(plan.joint_trajectory.points):
            if i == 0:
                point.velocities = tuple([0]*len(point.positions))
            end_time = point.time_from_start.secs + point.time_from_start.nsecs / 1000000000.0
            point.velocities =tuple(list( (np.array(point.positions) - np.array(prev_point.positions))/(end_time-start_time)))
            start_time = end_time
            prev_point = point

        if server and server.is_preempt_requested():
            server.set_preempted()
            return False

        if fraction != 1:
            success = False
        else:
            self.group.execute(plan)

        print(success)

        return success

    def dropoff3(self, server=None):
        success = self._dropoff3(rotate=False, longway=False, server=server)
        if not success:
            success = self._dropoff3(rotate=True, longway=False, server=server)
        if not success:
            success = self._dropoff3(rotate=True, longway=True, server=server)
        return success

    def _dropoff3(self, rotate=False, longway=False, server=None):
        success = True
        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))

        hand_position = self.group.get_current_pose(end_effector_link="mico_end_effector").pose.position

        x0 = pose.pose.position.x
        y0 = pose.pose.position.y
        z0 = pose.pose.position.z

        diff = [hand_position.x-x0, hand_position.y-y0, hand_position.z-z0]

        x1 = 0.207778792972 - diff[0]
        y1 = -0.297149433291 - diff[1]
        z1 = 0.275390054635 - diff[2]
        if rotate:
            x1 = 0.0843734169772
            y1 = -0.297149433291

        quaternion0 = pose.pose.orientation
        q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]
        rpy = tf.transformations.euler_from_quaternion(q0)
        q1 = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],-2)

        midpoint = tf.transformations.quaternion_slerp(q0, q1, 0.5)
        if longway:
            midpoint = tf.transformations.quaternion_multiply([0, 0, 1, 0], midpoint)

        height = 0.05
        if z0 > z1:
            height = 0.01
            a = -2 * ((height * (height + z0 - z1)) ** 0.5) - 2 * height - z0 + z1
            b = 2 * (((height * (height + z0 - z1)) ** 0.5) + height)
        else:
            a = -2 * ((height * (height - z0 + z1)) ** 0.5) - 2 * height + z0 - z1
            b = 2 * (((height * (height - z0 + z1)) ** 0.5) + height - z0 + z1)

        wpose = copy.deepcopy(pose.pose)

        nsteps = 10
        quaternions = [tf.transformations.quaternion_slerp(q0, midpoint, i * (1.0 / float(nsteps / 2))) for i in range(nsteps / 2)]
        quaternions += [tf.transformations.quaternion_slerp(midpoint, q1, i * (1.0 / float(nsteps / 2))) for i in range(nsteps / 2)]
        for i in range(nsteps):
            s = i * (1.0 / float(nsteps))
            wpose.position.x = (1 - s) * x0 + s * x1
            wpose.position.y = (1 - s) * y0 + s * y1
            wpose.position.z = a * (s ** 2) + b * s + z0
            orientation = q0
            if rotate:
                orientation = quaternions[i]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]

            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # eef_step
            10,  # joint_jump
            avoid_collisions=False)

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        print waypoints
        print fraction
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

        if fraction != 1:
            success = False
        else:
            self.group.execute(plan)

        return success

    def example_place(self, choice):
        a = (0.2453, -0.2417)
        b = (0.1605, -0.4507)
        c = (-0.1366, -0.4620)
        d = (-0.3281, -0.2705)
        places = [a, b, c, d]
        location = places[choice]
        success = self.place_object(location[0], location[1])
        if success:
            self.open_hand_and_go_home()

    def place_object(self, x_goal, y_goal, server=None):
        # self.manual_move(2, 1)
        pose = self.group.get_current_pose().pose
        hand_position = self.group.get_current_pose(end_effector_link="mico_end_effector").pose.position
        q0 = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        yaw = tf.transformations.euler_from_quaternion(q0)[2]
        length = abs((hand_position.x - pose.position.x) / math.sin(yaw))
        success = False
        for i in range(7):
            angle = i*(math.pi/4)
            success = self._place_object(angle, length, x_goal, y_goal, longway=False, server=server)
            if not success:
                success = self._place_object(angle, length, x_goal, y_goal, longway=True, server=server)
            if success:
                self.manual_move(2, -14/5)
                # self.manual_move(2, -9/5)
                break
        return success

    def _place_object(self, rot, length, x_goal, y_goal, longway=False, server=None):
        self.home_arm(execute=False)
        success = True
        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))

        x0 = pose.pose.position.x
        y0 = pose.pose.position.y
        z0 = pose.pose.position.z

        quaternion0 = pose.pose.orientation
        q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]
        rpy = tf.transformations.euler_from_quaternion(q0)
        print rpy[2]
        print("\n")
        yaw_goal = rpy[2]+rot
        if yaw_goal > math.pi:
            yaw_goal -= 2*math.pi
        print length
        print yaw_goal
        print("\n\n")

        q1 = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],yaw_goal)

        midpoint = tf.transformations.quaternion_slerp(q0, q1, 0.5)
        if longway:
            midpoint = tf.transformations.quaternion_multiply([0, 0, 1, 0], midpoint)

        x1 = x_goal - (length * math.sin(yaw_goal))
        y1 = y_goal + (length * math.cos(yaw_goal))
        z1 = z0 + 0.05

        height = 0.03

        a = -2 * ((height * (height - z0 + z1)) ** 0.5) - 2 * height + z0 - z1
        b = 2 * (((height * (height - z0 + z1)) ** 0.5) + height - z0 + z1)

        wpose = copy.deepcopy(pose.pose)

        nsteps = 10
        quaternions = [tf.transformations.quaternion_slerp(q0, midpoint, i * (1.0 / float(nsteps / 2))) for i in range(nsteps / 2)]
        quaternions += [tf.transformations.quaternion_slerp(midpoint, q1, i * (1.0 / float(nsteps / 2))) for i in range(nsteps / 2)]
        for i in range(nsteps):
            s = i * (1.0 / float(nsteps))
            wpose.position.x = (1 - s) * x0 + s * x1
            wpose.position.y = (1 - s) * y0 + s * y1
            wpose.position.z = a * (s ** 2) + b * s + z0

            wpose.orientation.x = quaternions[i][0]
            wpose.orientation.y = quaternions[i][1]
            wpose.orientation.z = quaternions[i][2]
            wpose.orientation.w = quaternions[i][3]

            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # eef_step
            10,  # joint_jump
            avoid_collisions=True)

        prev_point = plan.joint_trajectory.points[0]
        start_time = 0
        print fraction
        print("\n\n\n\n")
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

        if fraction != 1:
            success = False
        else:
            self.group.execute(plan)

        return success

    def lazy_length(self):
        pose = self.group.get_current_pose()
        quaternion0 = pose.pose.orientation
        q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]
        yaw = tf.transformations.euler_from_quaternion(q0)[2]
        hand_position = self.group.get_current_pose(end_effector_link="mico_end_effector").pose.position
        hand_position1 = (hand_position.x, hand_position.y)
        wrist_position = (pose.pose.position.x, pose.pose.position.y)
        hypothesis_length = abs((hand_position1[0] - wrist_position[0]) / math.sin(yaw))
        return hypothesis_length

    def lazy(self, length):
        pose = self.group.get_current_pose()

        quaternion0 = pose.pose.orientation
        q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]
        yaw = tf.transformations.euler_from_quaternion(q0)[2]

        hand_position = self.group.get_current_pose(end_effector_link="mico_end_effector").pose.position
        wrist_position = (pose.pose.position.x, pose.pose.position.y)
        hand_position1 = (hand_position.x, hand_position.y)

        hypothesis = ((-1*length * math.sin(yaw)) + wrist_position[0], (length * math.cos(yaw)) + wrist_position[1])
        compare_x = hypothesis[0] - hand_position1[0]
        compare_y = hypothesis[1] - hand_position1[1]
        print yaw
        print wrist_position
        print("\n")
        print hand_position1
        print hypothesis
        print("\n")
        print "x comparison: " + str(compare_x) + "  y comparison: " + str(compare_y)
        return yaw, wrist_position, hand_position1, hypothesis, compare_x, compare_y

    def go_to_dropoff(self, angle1, length1, x_goal1, y_goal1):
        def get_plan(angle, length, x_goal, y_goal):

            self.home_arm(execute=False)
            print("Planned path home")

            success = True
            self.group.set_planner_id("KPIECEkConfigDefault")  # ("RRTConnectkConfigDefault") ("RRTkConfigDefault") ("KPIECEkConfigDefault")
            # BKPIECEkConfigDefault   LBKPIECEkConfigDefault   PRMkConfigDefault   PRMstarkConfigDefault   RRTConnectkConfigPercise
            # RRTstarkConfigDefault   SBLkConfigDefault2   SBLkConfigDefault   TRRTkConfigDefault

            self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 10))
            start_pose = self.group.get_current_pose()
            end_effector = self.group.get_end_effector_link()
            self.group.set_start_state_to_current_state()

            z1 = start_pose.pose.position.z + 0.05
            quaternion0 = start_pose.pose.orientation
            q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]
            rpy = tf.transformations.euler_from_quaternion(q0)
            print rpy[2]
            print("\n")
            yaw_goal = rpy[2] + angle
            if yaw_goal > math.pi:
                yaw_goal -= 2 * math.pi
            print length
            print yaw_goal
            print("\n\n")

            x1 = x_goal + (length * math.sin(yaw_goal))
            y1 = y_goal - (length * math.cos(yaw_goal))

            q1 = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], yaw_goal)
            # target = [x1, y1, z1, q1[0], q1[1], q1[2], q1[3]]
            # self.group.set_pose_target(target)
            self.group.set_position_target([x1,y1,z1])
            # self.group.set_goal_position_tolerance(.001)

            # constraints = self.constrain_path(start_pose, end_effector)
            # print("\n \n GOT CONSTRAINTS \n \n")
            # self.group.set_path_constraints(constraints)
            # print("\n \n SET CONSTRAINTS \n \n")
            print("\n \n ABOUT TO FIND PLAN \n \n")

            plan = self.group.plan()
            print("\n \n PLANNED \n \n")

            if len(plan.joint_trajectory.points) == 0:
                rospy.logerr("Failed to plan path to dropoff.")
                success = False

            return success, plan

        success, plan = get_plan(angle1, length1, x_goal1, y_goal1)
        if not success:
            return success
        else:
            # success = self.group.execute(plan)
            # return success
            # trajectory_message = plan.joint_trajectory
            # success, error_msg, trajectory_result = self.run_trajectory(trajectory_message)
            error_msg = None
            trajectory_result = None
            return success

    def constrain_path(self, start_pose, end_effector):
        # current_orientation = self.group.get_current_pose().pose.orientation
        # start_pose = self.group.get_current_pose()
        current_orientation = start_pose.pose.orientation
        constraint_msg = moveit_msgs.msg.OrientationConstraint()
        constraint_msg.orientation = current_orientation
        print ("Current orientation: %s" % current_orientation)
        constraint_msg.link_name = end_effector
        print ("End effector: %s" % constraint_msg.link_name)
        constraint_msg.absolute_x_axis_tolerance = 3/4 * math.pi  # (math.pi) / 3
        constraint_msg.absolute_y_axis_tolerance = 3/4 * math.pi  # (math.pi) / 3
        constraint_msg.absolute_z_axis_tolerance = 0.1  # 200000000
        constraint_msg.weight = 0.5
        tf_frame = self.group.get_pose_reference_frame()
        print ("tf_frame: %s " % tf_frame)
        constraint_msg.header = start_pose.header
        all_constraints = self.group.get_path_constraints()
        all_constraints.orientation_constraints = []
        all_constraints.orientation_constraints.append(constraint_msg)
        return all_constraints

    def example_place2(self, choice):
        a = (0.2453, -0.2417)
        b = (0.1605, -0.4507)
        c = (-0.1366, -0.4620)
        d = (-0.3281, -0.2705)
        places = [a, b, c, d]
        location = places[choice]
        success = self.place_object2(location[0], location[1])
        if success:
            self.open_hand_and_go_home()

    def place_object2(self, x_goal, y_goal, server=None):
        # self.manual_move(2, 1)
        pose = self.group.get_current_pose().pose
        hand_position = self.group.get_current_pose(end_effector_link="mico_end_effector").pose.position
        q0 = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        yaw = tf.transformations.euler_from_quaternion(q0)[2]
        length = abs((hand_position.x - pose.position.x) / math.sin(yaw))
        success = False
        for i in range(4):
            angle = i * (math.pi / 2)
            success = self.go_to_dropoff(angle, length, x_goal, y_goal)
            if success:
                # self.manual_move(2, -14 / 5)
                # self.manual_move(2, -9/5)
                break
        return success

    #rosrun moveit_trajectory_planner manual_grasp_execution_node.py joint_states:=/mico_arm_driver/out/joint_state


