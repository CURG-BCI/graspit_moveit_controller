import rospy
import math
from sensor_msgs.msg import JointState

class GraspJointMessageUtils:
    def __init__(self):
        barrett_default_hand_closing_subspace = {'finger1/dist_joint': 1/3, 'finger1/med_joint': 1, 'finger1/prox_joint': 0,
                                                 'finger2/dist_joint': 1/3, 'finger2/med_joint': 1, 'finger2/prox_joint': 0,
                                                 'finger3/dist_joint': 1/3, 'finger3/med_joint': 1}
        jaco_default_hand_closing_subspace = {'mico_joint_finger_1': 1, 'mico_joint_finger_1_distal': 0,
                                              'mico_joint_finger_2': 1, 'mico_joint_finger_2_distal': 0}
        self.hand_closing_subspace = rospy.get_param('/hand_closing_subspace', jaco_default_hand_closing_subspace)
        barrett_default_hand_closed_angle = 2
        jaco_default_hand_closed_angle = math.pi/180*60 ## ???
        self.hand_closed_angle = rospy.get_param('/hand_closed_angle', jaco_default_hand_closed_angle)
        barrett_default_hand_preshape_subspace = {'finger1/dist_joint': 0, 'finger1/med_joint': 0, 'finger1/prox_joint': 1,
                                                  'finger2/dist_joint': 0, 'finger2/med_joint': 0, 'finger2/prox_joint': 1,
                                                  'finger3/dist_joint': 0, 'finger3/med_joint': 0}
        jaco_default_hand_preshape_subspace = {'mico_joint_finger_1': 0, 'mico_joint_finger_1_distal': 0,
                                              'mico_joint_finger_2': 0, 'mico_joint_finger_2_distal': 0}
        self.hand_preshape_subspace = rospy.get_param('/hand_preshape_subspace', jaco_default_hand_preshape_subspace)
        barrett_default_hand_joint_order = ['finger1/med_joint', 'finger2/med_joint', 'finger3/med_joint',
                                            'finger1/prox_joint']
        jaco_default_hand_joint_order = ['mico_joint_finger_1', 'mico_joint_finger_2']
        self.hand_joint_order = rospy.get_param('/hand_joint_order', jaco_default_hand_joint_order)

    def get_hand_preshape(self, joint_state_msg, percent_closed):
        output_msg = JointState()
        for name, pos in zip(joint_state_msg.name, joint_state_msg.position):
            if name in self.hand_closing_subspace.keys():
                joint_value = self.hand_preshape_subspace[name] * pos + \
                    self.hand_closing_subspace[name] * percent_closed * self.hand_closed_angle
                output_msg.name.append(name)
                output_msg.position.append(joint_value)
        return output_msg

    def joint_state_message_to_joint_angles(self, joint_state_message):
        output_angles = []
        joint_names = []
        for name, pos in zip(joint_state_message.name, joint_state_message.position):
            if name in self.hand_joint_order:
                joint_names.append(name)
                output_angles.append(pos)
        return joint_names, output_angles