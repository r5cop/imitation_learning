#!/usr/bin/env python

import rospy
import sys
import actionlib
import imitation_learning_msgs.msg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class Buffer(object):

    def __init__(self, dt, joint_names):
        self._joint_states = {name: None for name in joint_names}
        self._buffer = []
        self._dt = dt

    def add(self, msg):
        # try to zip the positions and the names
        positions = zip(msg.name, msg.position)

        # Check for each joint measurement if we want to store it in the buffer
        for name, position in positions:
            if name in self._joint_states:
                self._joint_states[name] = position

        # Get current timestamp float
        t_now = rospy.Time.now().to_sec()

        # Fill the buffer if dt passed
        if None not in self._joint_states.values():
            if not self._buffer or t_now - self._buffer[-1]["time"] > self._dt:
                self._buffer.append({"time": t_now, "positions": self._joint_states.values()})

    def clear(self):
        self._joint_states = {name: None for name in self._joint_states}
        self._buffer = []

    def get_action_result(self):
        result = imitation_learning_msgs.msg.LearnResult()

        result.trajectory.joint_names = self._joint_states.keys()

        if self._buffer:

            t_start = self._buffer[0]["time"]

            result.trajectory.points = [JointTrajectoryPoint(positions=p["positions"],
                                                             time_from_start=rospy.Time.from_seconds(p["time"] - t_start)) for p in self._buffer]
        else:
            result.error_msg = "No recorded points"

        return result


class ImitationLearningAction(object):
    _recording = False

    def __init__(self, robot_name, dt, joint_names):

        self._buffer = Buffer(dt, joint_names)

        self._joint_state_subscriber = rospy.Subscriber(robot_name + "/joint_states", JointState,
                                                        self._joint_state_cb, queue_size=1)

        self._action_name = robot_name + "/imitation_learning"
        self._as = actionlib.SimpleActionServer(self._action_name, imitation_learning_msgs.msg.LearnAction,
                                                execute_cb=self._goal_callback, auto_start=False)
        self._as.start()

    def _joint_state_cb(self, msg):
        if self._recording:
            self._buffer.add(msg)

    def _goal_callback(self, goal):
        self._recording = True

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Client wants to stop learning
            if self._as.is_preempt_requested():
                self._as.set_succeeded(self._buffer.get_action_result())
                break

            r.sleep()

        self._recording = False


if __name__ == '__main__':
    rospy.init_node('imitation_learning_node')

    if not rospy.has_param("~robot_name"):
        rospy.logerr("Please specify the ~robot_name parameter")
        sys.exit(1)

    if not rospy.has_param("~dt"):
        rospy.logerr("Please specify the ~dt parameter")
        sys.exit(1)

    if not rospy.has_param("~joint_names"):
        rospy.logerr("Please specify the ~joint_names parameter")
        sys.exit(1)

    ImitationLearningAction(rospy.get_param("~robot_name"),
                            rospy.get_param("~dt"),
                            rospy.get_param("~joint_names"))
    rospy.spin()
