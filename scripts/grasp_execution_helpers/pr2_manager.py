__author__ = 'jvarley'
import roslib

roslib.load_manifest('pr2_controllers_msgs')
import rospy

import actionlib

from pr2_controllers_msgs.msg import Pr2GripperCommand, Pr2GripperCommandAction, Pr2GripperCommandGoal, JointControllerState


def move_hand(positions, blocking=True):
    client = actionlib.SimpleActionClient(rospy.get_param('gripper_controller_name', 'l_gripper_controller') + '/gripper_action', Pr2GripperCommandAction)

    if len(positions) != 1:
        return False, "Wrong joint number", positions

    if not client.wait_for_server():
        success = False
        reason = 'failed to connect to action server'
        return success, reason, positions
    rospy.loginfo("Connected to Finger server")

    client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=positions[0], max_effort=-1)))

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()
        success = False
        reason = 'Program interrupted from keyboard'
        return success, reason, positions
    result = client.get_result()
    positions = [result.position]
    success = True
    reason = None
    return success, reason, positions


def close_hand():
    fingers = [0]
    success, reason, position = move_hand(fingers)
    return success, reason, position


def open_hand():
    fingers = [0.087] # slightly less than actual max of 0.0875
    success, reason, position = move_hand(fingers)
    return success, reason, position

def get_gripper_width():
    msg = rospy.wait_for_message(rospy.get_param('gripper_controller_name', 'l_gripper_controller') + '/state', JointControllerState)
    return [msg.process_value]

