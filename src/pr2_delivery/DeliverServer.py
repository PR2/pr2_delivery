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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

import roslib; roslib.load_manifest('pr2_delivery')
import rospy
import actionlib
import geometry_msgs.msg
import pr2_common_action_msgs.msg
from pr2_delivery.msg import DeliverAction, DeliverGoal
from Robot import Robot
from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorAction, PR2GripperEventDetectorGoal

class DeliverServer:

    (READY_TO_DELIVER, OBJECT_ACQUIRED, DELIVERY_ARRIVED, OBJECT_GIVEN) = (0, 1, 2, 3)

    def __init__(self):
        self.tuck_arm_client = actionlib.SimpleActionClient("tuck_arms", pr2_common_action_msgs.msg.TuckArmsAction)
        self.gripper_wiggle_detector_client = actionlib.SimpleActionClient('r_gripper_sensor_controller/event_detector', PR2GripperEventDetectorAction)

        self.robot = Robot()

        self.tuck_arm_client.wait_for_server(rospy.Duration(10.0))
        self.gripper_wiggle_detector_client.wait_for_server(rospy.Duration(10.0))

        self.server = actionlib.SimpleActionServer('deliver', DeliverAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        self.tuck_arms()
        self.navigate_to(goal.get_object_pose)
        self.say(self.READY_TO_DELIVER)
        self.get_object()
        self.say(self.OBJECT_ACQUIRED)
        self.navigate_to(goal.give_object_pose)
        self.say(self.DELIVERY_ARRIVED)
        self.give_object()
        self.say(self.OBJECT_GIVEN)
        self.tuck_arms()
        self.navigate_to(goal.return_home_pose)

        self.server.set_succeeded()

    def say(self, thing_to_say_code):
        rospy.loginfo("saying %d" % thing_to_say_code)
        # TODO eventually: say something

    def tuck_arms(self):
        rospy.loginfo("tucking arms")
        goal = pr2_common_action_msgs.msg.TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = True
        self.tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

    def navigate_to(self, nav_goal_pose):
        rospy.loginfo("navigating")
        rospy.sleep(3)
        # TODO

    def wait_for_gripper_wiggle(self):
        # contents of function ported from pr2_props
        goal = PR2GripperEventDetectorGoal()
        goal.command.trigger_conditions = 4 # use just acceleration as our contact signal
        goal.command.acceleration_trigger_magnitude = 6.0 # m/^2
        goal.command.slip_trigger_magnitude = 0.008 # slip gain
        self.gripper_wiggle_detector_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

    def get_object(self):
        rospy.loginfo("getting object")
        # TODO
        # - move right arm to accept-object pose
        self.robot.move_arm_to('r', [0.039, 1.1072, 0.0, -2.067, -1.231, -1.998, 0.369], 1) # intermediate pose to avoid self-collision
        self.robot.move_arm_to('r', [ -0.07666010001780543,   0.1622352230632809,   -0.31320771836735584,   -1.374860652847621,   -3.1324415863359545,   -1.078194355846691,   1.857217828689617], 2)

        object_gripped = False
        while not object_gripped:
            # - open gripper all the way
            self.robot.open_right_gripper()
            # - wait for externally-applied hand motion detected (ala "fist-pump" demo)
            self.wait_for_gripper_wiggle()
            rospy.loginfo("saw gripper wiggle")
            # - close gripper until we grab something
            if self.robot.grab_with_right_gripper():
                rospy.loginfo("grab succeeded, holding.")
                object_gripped = True
                self.robot.hold_with_right_gripper()
                rospy.loginfo("done holding.")
            else:
                rospy.loginfo("grab failed.")

    def give_object(self):
        rospy.loginfo("giving object")
        # TODO
        # - move right arm to give-object pose
        self.robot.move_arm_to('r', [-0.4,  1.1,   0.0,  -2.05,  0.0,  -0.1,  0.0], 2)
        # - wait for externally-applied hand motion detected (ala "fist-pump" demo)
        # - open gripper

# Question: what if sequence is pre-empted while robot is holding object?
# - maybe set the object on the floor?
# - v1 is certainly just "do nothing" so robot is still holding the object.
