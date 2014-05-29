import rospy
from numpy import array

import pr_msgs.msg
import pr_msgs.srv


def move_hand(angles, blocking=True):
    """:brief: joint angles of BarrettHand
       :param angles: target joint angles.
       :param blocking: if true, block until the hand stops moving

       :Returns whether movement succeeded, why it failed if necessary, and the final position for the joint motions.
    """
    success = MoveHandSrv(1, angles)
    if not success:
        return success, "MovehandSrv Failed", []
    if blocking:
        success, reason, position = WaitForHand(0)
    else:
        reason = "nonblocking"
        position = []
    return success, reason, position

def move_hand_velocity(velocities, blocking = True):
    """@brief - set joint velocities of BarrettHand
       @param velocities - target joint velocities.
       @param blocking - if true, block until the hand stops moving

       Returns whether movement succeeded, why it failed if necessary, and the final position for the joint motions.
    """
    success = MoveHandSrv(2, velocities)
    if not success:
        return success, "MovehandSrv in velocitymode failed", []
    if blocking:
        success, reason, position = WaitForHand(0)
    else:
        reason = "nonblocking"
        position = []
    return success, reason, position

def move_hand_percentage(percentage):
    """@brief - set joint angles of BarrettHand relative to current position
       @param percentage - target relative positions.
    """
    jnts = get_barrett_joints()
    move_hand(array([jnts[0] * percentage, jnts[1] * percentage, jnts[2] * percentage, jnts[3]]))
    
def get_barrett_joints():
    """@brief - Get the current position of the hand.

       Returns the current joint position of the hand as a list.
    """
    msg = rospy.wait_for_message("/bhd/handstate", pr_msgs.msg.BHState, 5)
    return msg.positions
                           
def stop_barrett():
    """@brief - Set desired hand position to current hand position, stopping the hand

       Returns true of motion is successfully stopped.
    """
    try:
        msg = rospy.wait_for_message("/bhd/handstate", pr_msgs.msg.BHState, 5)
        MoveHandSrv(1,msg.positions)
        return 1, msg.positions
    except:
        return 0, []

def relax_barrett():
    """@brief - Stop applying torque to the hand.
    """
    return RelaxHandSrv()

def open_barrett(open_aperture = 0):
    """@brief - open the fingers without changing the spread angle
    """
    success = 0
    reason = "Exception in open barrett"
    positions = []
    try:
        msg = rospy.wait_for_message("/bhd/handstate", pr_msgs.msg.BHState, 5)
        success,reason,positions =  move_hand([open_aperture,open_aperture,open_aperture,msg.positions[3]])
        
    except:
        pass
    return success,reason, positions

def close_barrett():
    """@brief - move fingers completely closed in position mode.
    """
    return open_barrett(2)


def GetHandPose( whichmanip ):
    try:
        msg = rospy.wait_for_message(topic = "/bhd/handstate", topic_type
                = pr_msgs.msg.BHState, timeout = 0.5 )
    except rospy.ROSException as ex:
        rospy.logwarn( 'Cannot get BHState: %s' % ex )
        return 0, ex, array( [0, 0, 0, 0] )

    return 1, '', array( msg.positions )


import time
from numpy import floor

def WaitForHand( whichmanip, timeout=5.0 ):
    '''
    :brief: Get hand pose and return as soon as it stops moving (or after timeout)
    :returns: Whether the hand had reached its final position before it finished moving or it errored out.

    '''
    waittime = 0.25
    numiter = int(floor(timeout / waittime))
    success, reason, positions = GetHandPose( whichmanip )
    for i in range( numiter ):
      lastpos = positions
      time.sleep( waittime )
      success, reason, positions = GetHandPose( whichmanip )
      rospy.loginfo( '  hand pose: %s' % ( array( positions ) ) )
      if sum( abs( lastpos - positions ) ) < 0.01:
          rospy.loginfo( 'break' )
          break
    return success, reason, positions

def MoveHandSrv(movetype, movevalues):
    try:
        owd_movehand = rospy.ServiceProxy('/bhd/MoveHand', pr_msgs.srv.MoveHand)
        res = owd_movehand(movetype, movevalues)
        #print res
        return 1
    except rospy.ServiceException as e:
        rospy.logerr("Service call to MoveHand failed: %s"%e.message)
        return 0


def RelaxHandSrv():
    try:
        owd_relaxhand = rospy.ServiceProxy('/bhd/RelaxHand', pr_msgs.srv.RelaxHand)
        res = owd_relaxhand()
        return 1
    except rospy.ServiceException as e:
        rospy.logerr("Service call to MoveHand failed: %s"%e)
        return 0
