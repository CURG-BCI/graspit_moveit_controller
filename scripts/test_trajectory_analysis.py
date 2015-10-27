import rospy
import rosbag
import os
import moveit_msgs.msg
import actionlib
import sensor_msgs.msg
import math

def is_bag_file(file_name):
    suffix = os.path.splitext(file_name)[1]
    return suffix == ".bag"

def load_bag_file(file_name):
    bag = rosbag.Bag(file_name)
    msgs = bag.read_messages()
    return msgs

def get_requests_from_bagfile(file_name, target_topics=['/pickup/goal']):
    msgs = load_bag_file(file_name)

    requests = [msg[1] for msg in msgs if msg[0] in target_topics]
    return requests



def load_cached_requests(req_file="/home/armuser/ros/bci_project_ws/2015-01-22-16-41-49.bag"):
    requests = []
    if is_bag_file(req_file):
        requests = get_requests_from_bagfile(req_file)
    return requests


def init_rosnode():
    try:
        rospy.init_node('test')
    except:
        pass


def send_planning_goal(goal):
    init_rosnode()
    pick_plan_client = actionlib.SimpleActionClient('/pickup', moveit_msgs.msg.PickupAction)
    try:
        pick_plan_client.wait_for_server(timeout = rospy.Duration(5))
    except:
        "Failed to find server"

    pick_plan_client.send_goal(goal)
    return pick_plan_client

def get_current_joint_pose():
    """

    :return: the current joint positions
    :rtype: sensors_msgs.msg.JointState
    """
    init_rosnode()
    joint_topic = "/joint_states"
    joint_msg = rospy.wait_for_message(joint_topic, sensor_msgs.msg.JointState)
    return joint_msg

def generate_joint_constraints(limits = {}):
    current_joints = get_current_joint_pose()
    constraints = []
    assert isinstance(current_joints, sensor_msgs.msg.JointState)
    for name, value in zip(current_joints.name, current_joints.position):
        if "finger" in name:
            continue

        joint_constraint = moveit_msgs.msg.JointConstraint()
        joint_constraint.position = value
        joint_constraint.joint_name = name
        temp_limit_constant = -math.pi #FIXME set and use limits
        joint_constraint.tolerance_below = temp_limit_constant
        joint_constraint.tolerance_above = temp_limit_constant
        constraints.append(joint_constraint)
    return constraints

def set_path_constraint(goal):
    """

    :param goal:
    :type goal: moveit_msgs.msg.PickupGoal
    :return:
    """
    jp = get_current_joint_pose()
    tc = moveit_msgs.msg.TrajectoryConstraints
    constraint = moveit_msgs.msg.Constraints()

    constraint.joint_constraints = generate_joint_constraints()
    goal.path_constraints = constraint
    return goal

requests = load_cached_requests()
g = set_path_constraint(requests[3].goal)
