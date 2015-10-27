#!/usr/bin/env python
from grasp_execution_node import GraspExecutor
import rospy


if __name__ == '__main__':

    rospy.init_node('graspit_message_robot_server')

    use_robot_hw = rospy.get_param('use_robot_hw', False)
    move_group_name = rospy.get_param('/arm_name', 'StaubliArm')
    grasp_approach_tran_frame = rospy.get_param('/approach_tran_frame','/approach_tran')
    print use_robot_hw
    rospy.loginfo("use_robot_hw value %d \n" % use_robot_hw)

    ge = GraspExecutor(use_robot_hw=use_robot_hw, move_group_name=move_group_name, grasp_tran_frame_name=grasp_approach_tran_frame)

    import IPython
    IPython.embed()
    assert False