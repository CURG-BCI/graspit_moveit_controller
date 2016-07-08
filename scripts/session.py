# coding: utf-8
import rospy
import grasp_analyzer_node


rospy.init_node('grasp_analyzer_node')

move_group_name = rospy.get_param('/arm_name', 'mico')
approach_frame = rospy.get_param('approach_tran_frame', '/approach_tran')
grasp_analyzer_node = grasp_analyzer_node.GraspAnalyzerNode(move_group_name=move_group_name, grasp_approach_tran_frame=approach_frame)
import IPython
IPython.embed()
