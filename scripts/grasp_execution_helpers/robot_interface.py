import rospy
import control_msgs.msg
import moveit_msgs.msg
import jaco_msgs.srv
import geometry_msgs.msg
import copy

import numpy as np
import math
import tf

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

    def home_arm(self, execute=True):

        def get_plan():
            success = True
            self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 20))
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
                difference = abs(current-final) % (math.pi)
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
        #print plan
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
                current_joint_values = self.group.get_current_joint_values()
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

    def go_to_dropoff(self):
        def get_plan():

            self.home_arm(execute=False)
            print("Planned path home")


            success = True
            #self.group.set_planner_id("KPIECEkConfigDefault")#("RRTkConfigDefault")#("RRTConnectkConfigDefault")#("KPIECEkConfigDefault")
            # BKPIECEkConfigDefault   LBKPIECEkConfigDefault   PRMkConfigDefault   PRMstarkConfigDefault   RRTConnectkConfigPercise
            # RRTstarkConfigDefault   SBLkConfigDefault2   SBLkConfigDefault   TRRTkConfigDefault
            self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 30))
            start_pose = self.group.get_current_pose()
            end_effector = self.group.get_end_effector_link()

            self.group.set_start_state_to_current_state()
            # q = start_pose.pose.orientation
            # target = [0.045, -0.383, 0.306, q.x, q.y, q.z, q.w]
            # self.group.set_pose_target(target, end_effector)
            self.group.set_named_target("dropoff")
            # self.group.set_goal_orientation_tolerance(0.75)#200000)

            # constraints = self.constrain_path(start_pose, end_effector)
            # print("\n \n GOT CONSTRAINTS \n \n")
            # self.group.set_path_constraints(constraints)
            # print("\n \n SET CONSTRAINTS \n \n")
            # print("\n \n ABOUT TO FIND PLAN \n \n")
            plan = self.group.plan()
            print("\n \n PLANNED \n \n")

            if len(plan.joint_trajectory.points) == 0:
                rospy.logerr("Failed to plan path to dropoff.")
                success = False

            return success, plan

        success, plan = get_plan()
        if not success:
            return success, None, None
        else:
            success = self.group.execute(plan)
            #return success
            trajectory_message = plan.joint_trajectory
            # success, error_msg, trajectory_result = self.run_trajectory(trajectory_message)
            error_msg = None
            trajectory_result = None
            return success, error_msg, trajectory_result


    def open_hand_and_go_home(self):
        self.hand_manager.open_hand()
        self.home_arm()
        return True

    def move_object(self):
        success = self.dropoff2()
        if success:
            self.hand_manager.open_hand()
            self.home_arm()
        return True

    def constrain_path(self, start_pose, end_effector):
        # current_orientation = self.group.get_current_pose().pose.orientation
        # start_pose = self.group.get_current_pose()
        current_orientation = start_pose.pose.orientation
        constraint_msg = moveit_msgs.msg.OrientationConstraint()
        constraint_msg.orientation = current_orientation
        print ("Current orientation: %s" %current_orientation)
        constraint_msg.link_name = end_effector
        print ("End effector: %s" %constraint_msg.link_name)
        constraint_msg.absolute_x_axis_tolerance = 0.75#(math.pi) / 3
        constraint_msg.absolute_y_axis_tolerance = 0.75#(math.pi) / 3 
        constraint_msg.absolute_z_axis_tolerance = 2*(math.pi) # 200000000
        constraint_msg.weight = 0.5
        tf_frame = self.group.get_pose_reference_frame()
        print ("tf_frame: %s " %tf_frame)
        constraint_msg.header = start_pose.header
        all_constraints = self.group.get_path_constraints()
        all_constraints.orientation_constraints = []
        all_constraints.orientation_constraints.append(constraint_msg) 
        return all_constraints

    def manual_move(self, axis, direction, server=None):
        """ axis: x = 0, y = 1, z = 2, roll = 3, pitch = 4, yaw = 5; direction: positive = 1, negative = -1 """

        value = 0.05 * direction
        if axis > 2:
            value = ((math.pi)/16) * direction
        self.group.clear_pose_targets()
        self.home_arm(execute=False)
        if server and server.is_preempt_requested():
            server.set_preempted()
            return False
        self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 10))
        self.group.set_start_state_to_current_state()
        self.group.set_pose_reference_frame("/root")
        self.group.shift_pose_target(axis, value)
        plan = self.group.plan()
        if server and server.is_preempt_requested():
            server.set_preempted()
            return False
        success = self.group.execute(plan)
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

        #first orient gripper and move forward (+z)
        wpose = copy.deepcopy(pose.pose)

        nsteps = 3
        for i in range(nsteps):
            wpose.position.x += changes[0]/nsteps
            wpose.position.y += changes[1]/nsteps
            wpose.position.z += changes[2]/nsteps
            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             10)           # joint_jump

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

    def dropoff2(self, server=None):
        success = self._dropoff2(longway=False, server=server)
        if not success:
            success = self._dropoff2(longway=True, server=server)

        return success

    def _dropoff2(self, longway=False, server=None):
        """x: 0.218732042971
        y: -0.36935908641
        z: 0.313392472227  orientation: x: 0.509653072671  y: 0.488239530881  z: 0.495386669354  w: 0.506426651972"""

        success = True

        frame = self.group.get_pose_reference_frame()
        print frame


        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))
        print pose

        x0 = pose.pose.position.x
        y0 = pose.pose.position.y
        z0 = pose.pose.position.z

        x1 = 0.00381255224334
        y1 = -0.340024369206
        z1 = 0.307940997544
        q1 = [-0.507977656598, -0.567790967257, 0.47915214215, 0.435873080806]

        # x1 = 0.0801740514278
        # y1 = 0.0801740514278
        # z1 = 0.307537764781
        # q1 = [-0.507977656598, -0.500647312925, -0.511124327039, 0.509742274093]
        quaternion0 = pose.pose.orientation
        q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]

        height = 0.08
        if z0 > z1:
            height = 0.01
            a = -2*((height*(height+z0-z1))**0.5) - 2*height - z0 + z1
            b = 2*(((height*(height + z0 - z1))**0.5) + height)
        else:
            a = -2*((height*(height - z0 + z1))**0.5) - 2*height + z0 - z1
            b = 2*(((height*(height - z0 + z1))**0.5) + height - z0 + z1)

        #first orient gripper and move forward (+z)
        wpose = copy.deepcopy(pose.pose)

        midpoint = tf.transformations.quaternion_slerp(q0, q1, 0.5)
        print midpoint

        if longway:
            midpoint = tf.transformations.quaternion_multiply([0,0,1,0], midpoint)

        nsteps = 10
        quaternions = [tf.transformations.quaternion_slerp(q0, midpoint, i*(1.0/float(nsteps/2))) for i in range(nsteps/2)]
        quaternions += [tf.transformations.quaternion_slerp(midpoint, q1, i*(1.0/float(nsteps/2))) for i in range(nsteps/2)]
        print quaternions
        for i in range(nsteps):
            s = i*(1.0/float(nsteps))
            wpose.position.x = (1-s)*x0 + s*x1
            wpose.position.y = (1-s)*y0 + s*y1
            wpose.position.z = a*(s**2) + b*s + z0

            # orientation = tf.transformations.quaternion_slerp(q0, q1, s)
            # orientation = tf.transformations.quaternion_inverse(quaternion)
            orientation = quaternions[i]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]
            rpy = tf.transformations.euler_from_quaternion(orientation)
            print rpy


            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.1,        # eef_step
                             10,          # joint_jump
                             avoid_collisions = False)           

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

        if fraction != 1:
            success = False
        else:
            self.group.execute(plan)

        return success




    def dropoff2_stable(self, height, server=None):
        """x: 0.218732042971
        y: -0.36935908641
        z: 0.313392472227  orientation: x: 0.509653072671  y: 0.488239530881  z: 0.495386669354  w: 0.506426651972"""

        success = True

        import copy
        waypoints = []
        pose = self.group.get_current_pose()
        waypoints.append(copy.deepcopy(pose.pose))
        print pose

        x0 = pose.pose.position.x
        y0 = pose.pose.position.y
        z0 = pose.pose.position.z

        x1 = 0.2187
        y1 = -0.3694
        z1 = 0.3134
        q1 = [0.473526062974, -0.526639748739, -0.517949982628, 0.479741032463]
        quaternion0 = pose.pose.orientation
        q0 = [quaternion0.x, quaternion0.y, quaternion0.z, quaternion0.w]

        a = -2*((height*(height - z0 + z1))**0.5) - 2*height + z0 - z1
        b = 2*(((height*(height - z0 + z1))**0.5) + height - z0 + z1)

        #first orient gripper and move forward (+z)
        wpose = copy.deepcopy(pose.pose)

        midpoint = tf.transformations.quaternion_slerp(q0, q1, 0.5)
        print midpoint
        # midpoint_inv = tf.transformations.quaternion_inverse(midpoint)
        # midpoint[0] = -midpoint[0]
        # midpoint[1] = -midpoint[1]
        # midpoint[2] = -midpoint[2]
        # midpoint[3] = -midpoint[3]
        print midpoint



        nsteps = 50
        quaternions = [tf.transformations.quaternion_slerp(q0, midpoint, i*(1.0/float(nsteps/2))) for i in range(nsteps/2)]
        # quaternions += [tf.transformations.quaternion_slerp(midpoint, q1, i*(1.0/float(nsteps/2))) for i in range(nsteps/2)]
        print quaternions
        for i in range(nsteps):
            s = i*(1.0/float(nsteps))
            wpose.position.x = (1-s)*x0 + s*x1
            wpose.position.y = (1-s)*y0 + s*y1
            wpose.position.z = a*(s**2) + b*s + z0

            # orientation = tf.transformations.quaternion_slerp(q0, q1, s)
            # orientation = tf.transformations.quaternion_inverse(quaternion)
            orientation = quaternions[i]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]
            rpy = tf.transformations.euler_from_quaternion(orientation)
            print rpy


            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.1,        # eef_step
                             10)           # joint_jump

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

        # self.group.execute(plan)

        if fraction != 1:
            success = False

        return success








