#!/usr/bin/env python

import pdb
from time import time
import sys

import rospy

import graspit_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg

import moveit_commander

import barrett_manager
import staubli_manager

import convert_graspit_msg
import WorldManager
import ReachabilityChecker


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
        self.reachability_checker = ReachabilityChecker.ReachabilityChecker()
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
                success, grasp_status_msg, positions = barrett_manager.open_barrett()
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

                
            if success:
                #Preshape the hand to the grasps' spread angle

                if self.robot_running:
                    success = barrett_manager.MoveHandSrv(1, [0,0,0, grasp_msg.pre_grasp_dof[0]])
                    print 'pre-grasp'
                    #Pregrasp the object
                    if not success:
                        grasp_status_msg = "Failed to preshape hand"

            if success:
                #Move to the goal location and grasp object
                moveit_grasp_msg = convert_graspit_msg.graspit_grasp_to_moveit_grasp(grasp_msg)
                result = self.group.pick(grasp_msg.object_name, moveit_grasp_msg)
                result = moveit_msgs.msg.MoveItErrorCodes(result)
                success = result.val == result.SUCCESS
                if not success:
                    grasp_status_msg = "MoveIt Failed to plan pick"

            #Failures shouldn't happen if grasp analysis is being used, so that's wierd.
            if not success:
                pdb.set_trace()

            if success and self.robot_running:
                #Close the hand completely until the motors stall or they hit
                #the final grasp DOFS
                success, grasp_status_msg, joint_angles = barrett_manager.move_hand([grasp_msg.final_grasp_dof[1],grasp_msg.final_grasp_dof[2], grasp_msg.final_grasp_dof[3], grasp_msg.final_grasp_dof[0]])

            if success and self.robot_running:
                #Now close the hand completely until the motors stall.
                success, grasp_status_msg, joint_angles = barrett_manager.close_barrett()
                if not success:
                    grasp_status = graspit_msgs.msg.GraspStatus.ROBOTERROR

            #Now wait for user input on whether or not to lift the object
            if success and self.robot_running:
                selection = int(raw_input('Lift up (1) or not (0): '))
                if selection == 1:
                    print 'lift up the object'
                    #Lift the object using the staubli's straight line path planner
                    success = staubli_manager.lift_arm(.05, True)
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

