#! /usr/bin/env python

import rospy
import actionlib
import imitation_learning_msgs.msg
import sys
import time

if len(sys.argv) < 2:
    print "Usage: ./record_for_x_seconds.py [seconds]"
    sys.exit(1)

wait_seconds = int(sys.argv[1])

rospy.init_node('record_for_x_seconds')
client = actionlib.SimpleActionClient('imitation_learning', imitation_learning_msgs.msg.LearnAction)

# Waits until the action server has started up and started
# listening for goals.
rospy.loginfo("Waiting for actionlib server to come online ..")
client.wait_for_server()

# Creates a goal to send to the action server.
goal = imitation_learning_msgs.msg.LearnGoal()

# Sends the goal to the action server.
client.send_goal(goal)

rospy.loginfo("Sleeping for %d seconds .." % wait_seconds)
time.sleep(wait_seconds)

rospy.loginfo("Cancelling goal ..")
client.cancel_goal()

rospy.loginfo("Waiting for result ..")
client.wait_for_result()

print client.get_result()