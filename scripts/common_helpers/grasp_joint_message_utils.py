import rospy
import math
from sensor_msgs.msg import JointState

class GraspJointMessageUtils:
    def __init__(self):
       
        self.hand_closing_subspace = rospy.get_param('hand_closing_subspace',{})
        
        self.hand_closed_angle = rospy.get_param('hand_closed_angle', math.pi)       
        self.hand_preshape_subspace = rospy.get_param('hand_preshape_subspace',{})
        self.hand_joint_order = rospy.get_param('hand_joint_order', [])
        self.joint_mimic = rospy.get_param('joint_mimic',{})
        self.graspit_joint_order = rospy.get_param('graspit_joint_order',self.hand_joint_order)


    def get_hand_preshape(self, joint_state_msg, percent_closed):
        output_msg = JointState()
        for name, pos in zip(joint_state_msg.name, joint_state_msg.position):
            if name in self.hand_closing_subspace.keys():
                joint_value = (self.hand_preshape_subspace[name] * pos +
                               self.hand_closing_subspace[name] * percent_closed * self.hand_closed_angle)
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


    def joint_angles_to_joint_state_message(self, joint_angles):
        joint_value_dict = dict(zip(self.hand_joint_order, joint_angles))
        output_msg = JointState()
        output_msg.name = self.joint_mimic.keys()
        output_msg.position = []
        for key in output_msg.name:
            mimicked_joint = self.joint_mimic[key]
            output_msg.position.append(mimicked_joint['ratio'] * joint_value_dict[mimicked_joint['source']])
        return output_msg

    def graspit_joint_angles_to_joint_state_message(self, graspit_joint_angles):
        graspit_joint_angles_dict = {zip(self.graspit_joint_order,graspit_joint_angles)}
        joint_angles = [graspit_joint_angles[joint] for joint in self.hand_joint_order]
        return self.joint_angles_to_joint_state_message(joint_angles)
        
                                        
