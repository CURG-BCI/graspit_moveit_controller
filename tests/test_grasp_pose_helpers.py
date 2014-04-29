import moveit_msgs.msg
import numpy
import tf
import tf_conversions
import numpy.linalg
import numpy as np
from copy import deepcopy
import rospy

def get_pose_as_tran(moveit_grasp_msg):
    """
    type: moveit_grasp_msg: moveit_msgs.msg.Grasp
    """
    assert isinstance(moveit_grasp_msg, moveit_msgs.msg.Grasp)
    pose_msg = moveit_grasp_msg.grasp_pose.pose
    return tf_conversions.toMatrix(tf_conversions.fromMsg(pose_msg))

def set_tran_as_pose(moveit_grasp_msg, tran):
    pose_msg = tf_conversions.toMsg(tf_conversions.fromMatrix(tran))
    moveit_grasp_msg_copy = deepcopy(moveit_grasp_msg)
    moveit_grasp_msg_copy.grasp_pose.pose = pose_msg
    return moveit_grasp_msg_copy

def invert_grasp_pose(moveit_grasp_msg):
    grasp_tran = get_pose_as_tran(moveit_grasp_msg)
    grasp_tran = numpy.linalg.inv(grasp_tran)
    return set_tran_as_pose(moveit_grasp_msg, grasp_tran)

def invert_grasp_rotation(moveit_grasp_msg):
    grasp_tran = get_pose_as_tran(moveit_grasp_msg)
    grasp_tran_inv = numpy.linalg.inv(grasp_tran)
    grasp_tran[:3,:3] = grasp_tran_inv[:3,:3]
    return set_tran_as_pose(moveit_grasp_msg, grasp_tran)

def invert_grasp_translation(moveit_grasp_msg):
    grasp_tran = get_pose_as_tran(moveit_grasp_msg)
    grasp_tran_inv = numpy.linalg.inv(grasp_tran)
    grasp_tran[:3,3] = grasp_tran_inv[:3,3]
    return set_tran_as_pose(moveit_grasp_msg, grasp_tran)


def invert_grasp_moveit(moveit_grasp_msg):
    graspit_tran = get_original_grasp_tran(moveit_grasp_msg)
    print graspit_tran
    graspit_tran_inv = numpy.linalg.inv(graspit_tran)
    print '7'
    return get_graspit_pose_to_moveit_pose(moveit_grasp_msg, graspit_tran_inv)

def invert_grasp_moveit_translation(moveit_grasp_msg):
    graspit_tran = get_original_grasp_tran(moveit_grasp_msg)
    graspit_tran_inv = numpy.linalg.inv(graspit_tran)
    graspit_tran[:3,3] = graspit_tran_inv[:3,3]
    return get_graspit_pose_to_moveit_pose(moveit_grasp_msg, graspit_tran)

def invert_grasp_moveit_rotation(moveit_grasp_msg):
    graspit_tran = get_original_grasp_tran(moveit_grasp_msg)
    graspit_tran_inv = numpy.linalg.inv(graspit_tran)
    graspit_tran[:3,:3] = graspit_tran_inv[:3,:3]
    return get_graspit_pose_to_moveit_pose(moveit_grasp_msg, graspit_tran)



def get_original_grasp_tran(moveit_grasp_msg):
    listener = tf.listener.TransformListener()
    print '1'
    listener.waitForTransform("approach_tran", 'staubli_rx60l_link7', rospy.Time(), timeout=rospy.Duration(1.0))
    at_to_ee_tran, at_to_ee_rot = listener.lookupTransform("approach_tran", 'staubli_rx60l_link7',rospy.Time())
    print '2'
    moveit_grasp_msg_final_grasp_tran_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(moveit_grasp_msg.grasp_pose.pose))
    approach_tran_to_end_effector_tran_matrix = tf.TransformerROS().fromTranslationRotation(at_to_ee_tran, at_to_ee_rot)
    original_ee_pose_matrix = np.dot( moveit_grasp_msg_final_grasp_tran_matrix, numpy.linalg.inv(approach_tran_to_end_effector_tran_matrix))
    return original_ee_pose_matrix

def get_graspit_pose_to_moveit_pose(moveit_grasp_msg, graspit_tran):

    listener = tf.listener.TransformListener()

    listener.waitForTransform("approach_tran", 'staubli_rx60l_link7', rospy.Time(), timeout=rospy.Duration(1.0))
    print '3'
    at_to_ee_tran, at_to_ee_rot = listener.lookupTransform("approach_tran", 'staubli_rx60l_link7',rospy.Time())
    print '4'

    approach_tran_to_end_effector_tran_matrix = tf.TransformerROS().fromTranslationRotation(at_to_ee_tran, at_to_ee_rot)
    actual_ee_pose_matrix = np.dot( graspit_tran, approach_tran_to_end_effector_tran_matrix)
    actual_ee_pose = tf_conversions.toMsg(tf_conversions.fromMatrix(actual_ee_pose_matrix))
    rospy.loginfo("actual_ee_pose: " + str(actual_ee_pose))
    moveit_grasp_msg_copy = deepcopy(moveit_grasp_msg)
    moveit_grasp_msg_copy.grasp_pose.pose = actual_ee_pose
    return moveit_grasp_msg_copy
