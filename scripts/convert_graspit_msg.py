__author__ = 'jweisz'
import graspit_msgs.msg
import tf_conversions.posemath as pm
import moveit_msgs.msg
from numpy import zeros
from numpy import linalg
import geometry_msgs.msg

import trajectory_msgs.msg


def barrett_positions_from_graspit_positions(positions):
    """
    :param positions: Positions of the barrett hand in graspit's convention
    :type positions: numpy.array
    :returns a pair containing the joint names array and the joint positions
    :rtype: (list[string],numpy.array)
    """
    names = ['/finger_1/prox_link', '/finger_1/med_link', '/finger_1/dist_link',
             '/finger_2/prox_link', '/finger_2/med_link', '/finger_2/dist_link',
             '/finger_3/med_link', '/finger_3/dist_link']
    prefix_str = 'wam/bhand/'
    joint_names = [prefix_str + name for name in names]
    joint_positions = zeros([len(names), 1])
    joint_positions[0] = positions[0]
    joint_positions[1] = positions[1]
    joint_positions[2] = positions[1]/3.0
    joint_positions[3] = positions[0]
    joint_positions[4] = positions[2]
    joint_positions[5] = positions[2]/3.0
    joint_positions[7] = positions[3]
    joint_positions[8] = positions[3]/3.0

    return joint_names, joint_positions


def graspit_grasp_to_moveit_grasp(grasp_msg, grasp_tran_frame_name='wam/bhand/approach_tran'):
    """
    :param grasp_msg: A graspit grasp message
    :type grasp_msg: graspit_msgs.msg.Grasp
    :returns a moveit message built from the graspit grasp
    :rtype: moveit_msgs.msg.Grasp
    """
    #Convert the grasp message to a transform
    grasp_tran = pm.toMatrix(pm.fromMsg(grasp_msg.final_grasp_pose))
    grasp_tran[0:3, 3] /= 1000      # mm to meters

    pregrasp_tran = pm.toMatrix(pm.fromMsg(grasp_msg.pre_grasp_pose))
    pregrasp_tran[0:3, 3] /= 1000     # mm to meters

    pregrasp_dist = linalg.norm(pregrasp_tran[0:3, 3] - grasp_tran[0:3, 3])

    grasp = moveit_msgs.msg.Grasp()
    grasp.allowed_touch_objects = False
    grasp.grasp_pose.pose = grasp_msg.final_grasp_pose
    goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
    spread_pregrasp_dof = [0, 0, 0, grasp_msg.pre_grasp_dof[3]]

    joint_names, goal_point.positions = barrett_positions_from_graspit_positions(spread_pregrasp_dof)
    grasp.pre_grasp_posture.points.append(goal_point)
    joint_names, goal_point.positions = barrett_positions_from_graspit_positions(grasp_msg.pre_grasp_dof)
    grasp.grasp_posture.points.append(goal_point)

    grasp.pre_grasp_approach.direction.vector = geometry_msgs.msg.Vector3(0, 0, 1)
    grasp.pre_grasp_approach.direction.header.frame_id = grasp_tran_frame_name
    grasp.pre_grasp_approach.desired_distance = pregrasp_dist
    grasp.pre_grasp_approach.min_distance = pregrasp_dist

    grasp.post_grasp_retreat.min_distance = .05
    grasp.post_grasp_retreat.desired_distance = .05
    grasp.post_grasp_retreat.direction.header.frame_id = '/world'
    grasp.post_grasp_retreat.direction.vector = geometry_msgs.msg.Vector3(0, 0, 1)
    return grasp