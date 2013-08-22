#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('pr2_delivery')

import rospy

from trajectory_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from sensor_msgs.msg import JointState
import actionlib

class Joint:
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class ArmMover:
    def __init__(self):
        self.joint_names = ["shoulder_pan", 
                            "shoulder_lift",
                            "upper_arm_roll",
                            "elbow_flex", 
                            "forearm_roll",
                            "wrist_flex", 
                            "wrist_roll" ]

        # Get controller name and start joint trajectory action clients
        self.r_action_name = 'r_arm_controller/joint_trajectory_action'
        self.l_action_name = 'l_arm_controller/joint_trajectory_action'
        self.left_joint_client = actionlib.SimpleActionClient(self.l_action_name, JointTrajectoryAction)
        self.right_joint_client = actionlib.SimpleActionClient(self.r_action_name, JointTrajectoryAction)

        # Wait for joint clients to connect with timeout
        if not self.left_joint_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("ArmMover.py: left_joint_client action server did not come up within timelimit")
        if not self.right_joint_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("ArmMover.py: right_joint_client action server did not come up within timelimit")

        self.r_gripper_action_name = 'r_gripper_controller/gripper_action'
        self.right_gripper_client = actionlib.SimpleActionClient(self.r_gripper_action_name, Pr2GripperCommandAction)

        if not self.right_gripper_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("ArmMover.py: right_gripper_client action server did not come up within timelimit")

        self.joints = {}
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        # string[] name
        # float64[] position
        # float64[] velocity
        # float64[] effort
        i = 0
        for name in msg.name:
            state = Joint(name, msg.position[i], msg.velocity[i], msg.effort[i])
            self.joints[name] = state
            i += 1

    def go(self, side, joint_values, duration):
        """side is 'l' or 'r'.
           joint_values is an array of 7 joint values."""
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = [side+"_"+name+"_joint" for name in self.joint_names]
        goal.trajectory.points = []
        goal.trajectory.points.append(JointTrajectoryPoint( positions = joint_values,
                                                            velocities = [],
                                                            accelerations = [],
                                                            time_from_start = rospy.Duration(duration)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        client = {'l': self.left_joint_client, 'r': self.right_joint_client}[side]
        return client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

    def open_right(self):
        return self.right_grip(0.08)

    def close_right(self):
        return self.right_grip(0, 200.0)

    def right_grip(self, position, max_effort = -1):
        goal = Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        return self.right_gripper_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

    def print_arm_pose(self, which_arm):
        joint_pose = []
        for joint in self.joint_names:
            full_name = which_arm + "_" + joint + "_joint"
            joint_pose.append( self.joints[full_name].position )
        print(which_arm, "arm pose", joint_pose)
