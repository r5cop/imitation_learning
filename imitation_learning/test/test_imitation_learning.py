#! /usr/bin/env python

import rospy
import actionlib
import imitation_learning_msgs.msg
import sys
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


def yes_or_no(question):
    reply = str(raw_input(question+' (y/n): ')).lower().strip()
    if reply[0] == 'y':
        return True
    if reply[0] == 'n':
        return False
    else:
        return yes_or_no("Uhhhh... please enter ")

rospy.init_node('record_for_x_seconds')
record_client = actionlib.SimpleActionClient('imitation_learning', imitation_learning_msgs.msg.LearnAction)
play_client = actionlib.SimpleActionClient("body/joint_trajectory_action", FollowJointTrajectoryAction)

# Waits until the action server has started up and started
# listening for goals.
rospy.loginfo("Waiting for actionlib servers to come online ..")
record_client.wait_for_server()
play_client.wait_for_server()

# Creates a goal to send to the action server.
goal = imitation_learning_msgs.msg.LearnGoal()

if not yes_or_no("Start recording?"):
    sys.exit(0)

# Sends the goal to the action server.
record_client.send_goal(goal)

while not yes_or_no("Stop recording?"):
    pass

rospy.loginfo("Cancelling goal ..")
record_client.cancel_goal()

rospy.loginfo("Waiting for result ..")
record_client.wait_for_result()

result = record_client.get_result()
rospy.loginfo("Recorded trajectory: %s", result)

if not yes_or_no("Do you want to replay it?"):
    sys.exit(0)

if not result.trajectory.points:
    rospy.logerr("Empty trajectory ...")
    sys.exit(1)

goal = FollowJointTrajectoryGoal(trajectory=result.trajectory,
                                 goal_time_tolerance=result.trajectory.points[-1].time_from_start)

play_client.send_goal(goal)

rospy.loginfo("Waiting for result ..")
play_client.wait_for_result()

rospy.loginfo("Result: %s", play_client.get_result())





