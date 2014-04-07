#!/usr/bin/env python
import roslib
import rospy
import time
import graspit_msgs.msg
import geometry_msgs.msg
import tf, tf_conversions, tf.transformations
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size, zeros
from numpy import hstack, vstack, mat, array, arange, fabs
import tf_conversions.posemath as pm
from time import sleep 
#import trajectory_planner as tp
import pdb
from std_msgs.msg import String, Empty
from time import time
import WorldManager

import moveit_commander


import trajectory_planner as tp
import sys
import moveit_msgs.msg
import trajectory_msgs

#import grasp_analyzer

Hao = False

class GraspExecutor():
    """@brief - Generates a persistent converter between grasps published by a graspit commander and
    the trajectory planning module to actually lift objects off the table

    @member grasp_listener - subscriber to the graspit grasp channel. Expects graspit_msgs Grasp types
    @member name_listener - subscribes to an object name channel. Expects a string which corresponds
                            to an entry in the filename dictionary 
    @member global_data - Planning environment
    @member target_object_name - the current grasp target
    """

    def __init__(self, init_planner = True, moveGroupName='StaubliArm', grasp_tran_frame_name = 'wam/bhand/approach_tran'):
        self.grasp_listener = rospy.Subscriber("/graspit/grasps", graspit_msgs.msg.Grasp, self.process_grasp_msg)

        self.graspit_status_publisher = rospy.Publisher("/graspit/status", graspit_msgs.msg.GraspStatus)

        self.last_grasp_time = 0
        self.table_cube=[geometry_msgs.msg.Point(-0.7,0,-0.02), geometry_msgs.msg.Point(0.2,1,1)]
        self.robot_running = init_planner

        #start up WorldManager
        self.wm = WorldManager.WorldManager()

        moveit_commander.roscpp_initialize(sys.argv)
        moveit_commander.MoveGroupCommander
        self.group = moveit_commander.MoveGroupCommander(moveGroupName)

        self.grasp_tran_frame_name = grasp_tran_frame_name


        #self.grasp_analyzer = grasp_analyzer.GraspAnalyzer(self.global_data)
        if bool(rospy.get_param('reload_model_rec',0)):
            self.reload_model_list([])



    def barrett_positions_from_graspit_positions(self, positions):
        names = ['/finger_1/prox_link','/finger_1/med_link','/finger_1/dist_link',
                 '/finger_2/prox_link','/finger_2/med_link','/finger_2/dist_link',
                 '/finger_3/med_link','/finger_3/dist_link']
        prefix_str = 'wam/bhand/'
        joint_names = [prefix_str + name for name in names]
        joint_positions = zeros([len(names),1])
        joint_positions[0] = positions[0]
        joint_positions[1] = positions[1]
        joint_positions[2] = positions[1]/3.0
        joint_positions[3] = positions[0]
        joint_positions[4] = positions[2]
        joint_positions[5] = positions[2]/3.0
        joint_positions[7] = positions[3]
        joint_positions[8] = positions[3]/3.0

        return joint_names, joint_positions
        

    def process_grasp_msg(self, grasp_msg):
        """@brief - Attempt to grasp the object and lift it

        First moves the arm to pregrasp the object, then attempts to grasp and lift it.
        The grasp and lifting phase is currently completely open loop
        
        """
        try:
            #Reject multiple grasp messages sent too fast. They are likely a mistake
            #FIXME - We should probably just set the queue size to 1.
            if (time() - self.last_grasp_time) < 30:
                return [], []
            self.last_grasp_time = time()
            
            print grasp_msg
            grasp_status = graspit_msgs.msg.GraspStatus.SUCCESS
            grasp_status_msg = "grasp_succeeded"
            success = 1
            
            # Send the robot to its home position if it is actually running
            #and not currently there

            if self.robot_running:
                print 'go home'
                self.group.set_named_target("home")
                plan = self.group.plan()
                success = self.group.execute(plan)
                if not success:
                    print "Couldn't move arm home"

                #This sometimes fails near the end of the trajectory because the last few
                #steps of the trajectory are slow because our blending parameters
                #are probably wrong. It is basically in the home position, so we
                #don't check for success in reaching the home position because meaningful
                #failures are rare.


            #Open the hand - Leaves spread angle unchanged
            if success and self.robot_running:
                success, grasp_status_msg, positions = tp.open_barrett()
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

                
            if success:
                #Pregrasp the object
                    
                #Convert the grasp message to a transform
                grasp_tran = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
                grasp_tran[0:3,3] /=1000 #mm to meters

                pregrasp_tran = pm.toMatrix(pm.fromMsg(grasp_msg.pre_grasp_pose))
                pregrasp_tran[0:3,3] /=1000 #mm to meters

                pregrasp_dist = linalg.norm(pregrasp_tran[0:3,3] - grasp_tran[0:3,3])

                #Move the hand to the pregrasp spread angle
                if self.robot_running:
                    tp.MoveHandSrv(1, [0,0,0, grasp_msg.pre_grasp_dof[0]])
                    print 'pre-grasp'



                grasp = moveit_msgs.msg.Grasp()
                grasp.allowed_touch_objects = False
                grasp.grasp_pose.pose = grasp_msg.final_grasp_pose
                goal_point = trajectory_msgs.msg.JointTrajectoryPoint()

                joint_names, goal_point.positions = self.barrett_positions_from_graspit_positions(graspit_msgs.msg.Grasp.pre_grasp_dof)
                grasp.pregrasp_posture.points.append(goal_point)
                joint_names, goal_point.positions = self.barrett_positions_from_graspit_positions(graspit_msgs.msg.Grasp.final_grasp_dof)
                grasp.grasp_posture.points.append(goal_point)

                grasp.pre_grasp_approach.direction.vector = geometry_msgs.msg.Vector3(0,0,1)
                grasp.pre_grasp_approach.direction.header.frame_id = self.grasp_tran_frame_name
                grasp.pre_grasp_approach.desired_distance = pregrasp_dist
                grasp.pre_grasp_approach.min_distance = pregrasp_dist

                grasp.post_grasp_retreat.min_distance = .05
                grasp.post_grasp_retreat.desired_distance = .05
                grasp.post_grasp_retreat.direction.header.frame_id = '/world'
                grasp.post_grasp_retreat.direction.vector = geometry_msgs.msg.Vector3(0,0,1)

                success = self.group.pick(grasp_msg.object_name, grasp)


            #Failures shouldn't happen if grasp analysis is being used, so that's wierd.
            if not success:
                pdb.set_trace()
            else:
                success, grasp_status_msg, joint_angles = tp.move_hand([grasp_msg.pre_grasp_dof[1],grasp_msg.pre_grasp_dof[2], grasp_msg.pre_grasp_dof[3], grasp_msg.pre_grasp_dof[0]])


            if success:
                #Close the hand completely until the motors stall or they hit
                #the final grasp DOFS
                success, grasp_status_msg, joint_angles = tp.move_hand([grasp_msg.final_grasp_dof[1],grasp_msg.final_grasp_dof[2], grasp_msg.final_grasp_dof[3], grasp_msg.final_grasp_dof[0]])


            if success:
                #Now close the hand completely until the motors stall.
                success, grasp_status_msg, joint_angles = tp.close_barrett()
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

            #Now wait for user input on whether or not to lift the object
            if success:
                selection = int(raw_input('Lift up (1) or not (0): '))
                if selection == 1:
                    print 'lift up the object'
                    #Lift the object using the staubli's straight line path planner
                    success = tp.lift_arm(.05, True)
                    if not success:
                        grasp_status = graspit_msgs.msg.GraspStatus.UNREACHABLE
                        grasp_status_msg = "Couldn't lift object"
                else:
                    print 'not lift up the object'


            #Maybe decide if it has been successfully lifted here...
            if success:
                rospy.logwarn(grasp_status_msg)
            else:
                rospy.logfatal(grasp_status_msg)
            #Tell graspit whether the grasp succeeded
            self.graspit_status_publisher.publish(grasp_status, grasp_status_msg, -1)
            
            print grasp_status_msg

            return grasp_status, grasp_status_msg

        except Exception as e:
            #print any exceptions and drop in to the debugger.
            import traceback
            print traceback.format_exc()
            pdb.set_trace()



    def point_within_table_cube(self, test_point):
        """
        @brief - Test whether a point is within a cube defined by its
        lower left and upper right corners. The corners are stored in the table_cube
        member. 

        FIXME - This implementation is likely slow and dumb, but it works for me. 

        @param test_point - The point to test
        """
        [min_corner_point , max_corner_point ] = self.table_cube 
        keys = ['x', 'y', 'z']
        for k in keys:
            t = getattr(test_point, k)
            min_test = getattr(min_corner_point, k)
            max_test = getattr(max_corner_point, k)
            if t < min_test or t > max_test:
                print 'Failed to be inside table in key %s - min - %f max - %f value %f'%(k, min_test, max_test, t)
                return False
        return True




if __name__ == '__main__':
    try:        
        rospy.init_node('graspit_message_robot_server')
        init_planner = rospy.get_param('init_planner', True)
        print "init planner value %d \n"%(init_planner)
        ge = GraspExecutor(init_planner = init_planner)
        loop = rospy.Rate(10)

        
        while not rospy.is_shutdown():
            ge.wm.model_manager.rebroadcast_object_tfs()
            loop.sleep()
    except rospy.ROSInterruptException: pass

