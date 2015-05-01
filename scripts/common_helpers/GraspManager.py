import rospy
import math
from sensor_msgs.msg import JointState


class GraspManager(object):
    def __init__(self, hand_manager, move_group_name="StaubliArm"):
            barrett_default_hand_closing_subspace = {'finger1/dist_joint': 1/3, 'finger1/med_joint': 1, 'finger1/prox_joint': 0,
                                                     'finger2/dist_joint': 1/3, 'finger2/med_joint': 1, 'finger2/prox_joint': 0,
                                                     'finger3/dist_joint': 1/3, 'finger3/med_joint': 1}
            jaco_default_hand_closing_subspace = {'mico_joint_finger_1': 1, 'mico_joint_finger_1_distal': 0,
                                                  'mico_joint_finger_2': 1, 'mico_joint_finger_2_distal': 0}

            barrett_default_hand_closed_angle = 2
            jaco_default_hand_closed_angle = math.pi/180*60 ## ???

            barrett_default_hand_preshape_subspace = {'finger1/dist_joint': 0, 'finger1/med_joint': 0, 'finger1/prox_joint': 1,
                                                      'finger2/dist_joint': 0, 'finger2/med_joint': 0, 'finger2/prox_joint': 1,
                                                      'finger3/dist_joint': 0, 'finger3/med_joint': 0}
            jaco_default_hand_preshape_subspace = {'mico_joint_finger_1': 0, 'mico_joint_finger_1_distal': 0,
                                                  'mico_joint_finger_2': 0, 'mico_joint_finger_2_distal': 0}

            barrett_default_hand_joint_order = ['finger1/med_joint', 'finger2/med_joint', 'finger3/med_joint',
                                                'finger1/prox_joint']
            jaco_default_hand_joint_order = ['mico_joint_finger_1', 'mico_joint_finger_2']

            default_hand_closing_subspace = jaco_default_hand_closing_subspace
            default_hand_closed_angle = jaco_default_hand_closed_angle
            default_hand_preshape_subspace = jaco_default_hand_preshape_subspace
            default_hand_joint_order = jaco_default_hand_joint_order

            if move_group_name == "StaubliArm":
                rospy.loginfo("Grasp Mananger Using Staubli Arm")
                default_hand_closing_subspace = barrett_default_hand_closing_subspace
                default_hand_closed_angle = barrett_default_hand_closed_angle
                default_hand_preshape_subspace = barrett_default_hand_preshape_subspace
                default_hand_joint_order = barrett_default_hand_joint_order
            else:
                rospy.loginfo("Grasp Manager using Jaco Arm")

            self.hand_closing_subspace = self.get_sanitized_dict('/hand_closing_subspace', default_hand_closing_subspace)
            self.hand_closed_angle = rospy.get_param('/hand_closed_angle', default_hand_closed_angle)
            self.hand_preshape_subspace = self.get_sanitized_dict('/hand_preshape_subspace', default_hand_preshape_subspace)
            self.hand_joint_order = rospy.get_param('/hand_joint_order', default_hand_joint_order)

            self.hand_manager = hand_manager

    def get_sanitized_dict(self, param_name, default):
        return self.sanitize_dict(rospy.get_param(param_name, default))

    def sanitize_dict(self, d):
        for k in d.keys():
            val = d.pop(k)
            k = k.replace('slash','/')
            if isinstance(val, basestring):
                d[k] = val.replace('slash','/')
            else:
                d[k] = val
        print d
        return d

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


    def move_hand_msg(self, joint_state_msg):
        """
        :type joint_state_msg: sensor_msgs.msg.JointState
        """
        joint_names, joint_angles = self.joint_state_message_to_joint_angles(joint_state_msg)
        ordered_joint_angles = [joint_angles[joint_names.index(joint_name)] for joint_name in self.hand_joint_order]
        return self.hand_manager.move_hand(ordered_joint_angles)

    def move_hand_percentage(self, joint_state_msg, percent_closed):
        """
        :type joint_state_msg: sensor_msgs.msg.JointState
        """
        percentage_joint_state = self.get_hand_preshape(joint_state_msg, percent_closed)
        return self.move_hand_msg(percentage_joint_state)

    def open_grasp(self, joint_state_msg):
        return self.move_hand_percentage(joint_state_msg, 0)

    def close_grasp(self, joint_state_msg):
        return self.move_hand_percentage(joint_state_msg, 1)

    def open_hand(self):
        return self.hand_manager.open_hand()

    def close_hand(self):
        return self.hand_manager.close_hand()

    def joint_trajectory_to_joint_state(self, joint_trajectory_msg, index):
        joint_states_msg = JointState()
        joint_states_msg.name = joint_trajectory_msg.joint_names
        joint_states_msg.position = joint_trajectory_msg.points[index].positions
        joint_states_msg.velocity = joint_trajectory_msg.points[index].velocities
        joint_states_msg.effort = joint_trajectory_msg.points[index].effort
        return joint_states_msg

    def move_hand_trajectory(self, joint_trajectory_msg):

        for i in xrange(len(joint_trajectory_msg.points)):
            joint_state_msg = self.joint_trajectory_to_joint_state(joint_trajectory_msg, i)
            if not self.move_hand_msg(joint_state_msg):
                return False, "Failed to grasp"

        return True, "Grasp Succeeded"

