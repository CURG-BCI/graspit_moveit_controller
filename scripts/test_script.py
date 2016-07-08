from grasp_execution_helpers import jaco_manager
from common_helpers import grasp_joint_message_utils
from sensor_msgs.msg import JointState
import rospy
import ipdb

rospy.init_node('test_hand')
util = grasp_joint_message_utils.GraspJointMessageUtils()
current_state = rospy.wait_for_message('/joint_states', JointState)
joint_msg = util.get_hand_preshape(current_state, .5)
joint_names, joint_angles = util.joint_state_message_to_joint_angles(joint_msg)
