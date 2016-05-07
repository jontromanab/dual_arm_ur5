#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_elbow_joint',
               'r_wrist_1_joint', 'r_wrist_2_joint', 'r_wrist_3_joint']
Q1 = [1.57,-1.57,0,-1.57,0,0]
Q2 = [-1.57,-1.57,0,-1.57,0,0]
Q3 = [-1.57,-0.1745,-2.79,-1.57,0,0]

client = None

def move():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(6.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for right_arm server..."
        client.wait_for_server()
        print "Connected to right_arm server"
        move()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()

