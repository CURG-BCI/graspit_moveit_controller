__author__ = 'jvarley'
import roslib

roslib.load_manifest('jaco_msgs')
import rospy
from numpy import array

import actionlib

import math

import jaco_msgs.msg

import sys

import ipdb


def move_hand(angles, blocking=True):
    client = actionlib.SimpleActionClient('/jaco_arm_driver/fingers/finger_positions',
                                          jaco_msgs.msg.SetFingersPositionAction)

    angles[0] = angles[0]*180/math.pi*6400/60
    angles[1] = angles[1]*180/math.pi*6400/60
    angles[2] = angles[2]*180/math.pi*6400/60

    goal = jaco_msgs.msg.SetFingersPositionGoal()

    if len(sys.argv) < 4:
        goal.fingers.finger1 = angles[0]
        goal.fingers.finger2 = angles[1]
        goal.fingers.finger3 = angles[2]

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
        goal.fingers.finger1 = angles[0]
        goal.fingers.finger2 = angles[1]
        goal.fingers.finger3 = angles[2]

    if not client.wait_for_server():
        success = False
        reason = 'failed to connect to action server'
        position = None
        return success, reason, position
    rospy.loginfo("Connected to Finger server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()
        success = False
        reason = 'Program interrupted from keyboard'
        position = None
        return success, reason, position
    result = client.get_result()
    position = [result.fingers.finger1, result.fingers.finger2, result.fingers.finger3]
    success = True
    reason = None
    return success, reason, position


def close_hand():
    fingers = [math.pi/180*60, math.pi/180*60, 0]
    success, reason, position = move_hand(fingers)
    return success, reason, position


def open_hand():
    fingers = [0, 0, 0]
    success, reason, position = move_hand(fingers)
    return success, reason, position

def move_hand_percentage(percentage):
    """@brief - set joint angles of Mico relative to current position
       @param percentage - target relative positions.
    """
    jnts=get_mico_joints()
    move_hand(array([jnts[0] * percentage, jnts[1] * percentage, 0]))


def get_mico_joints():
    """@brief - Get the current position of the hand.

       Returns the current joint position of the hand as a list.
    """
    msg = rospy.wait_for_message("/jaco_arm_driver/out/finger_position", jaco_msgs.msg.FingerPosition)
    return [msg.finger1, msg.finger2, msg.finger3]

