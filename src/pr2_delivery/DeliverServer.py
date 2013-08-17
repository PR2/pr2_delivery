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
from pr2_delivery.msg import DeliverAction, DeliverGoal

class DeliverServer:

    (READY_TO_DELIVER, OBJECT_ACQUIRED, DELIVERY_ARRIVED, OBJECT_GIVEN) = (0, 1, 2, 3)

    def __init__(self):
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
        # TODO

    def navigate_to(self, nav_goal_pose):
        rospy.loginfo("navigating")
        # TODO

    def get_object(self):
        rospy.loginfo("getting object")
        # TODO
        # - move right arm to accept-object pose
        # - loop until object is gripped:
        #   - open gripper all the way
        #   - wait for externally-applied hand motion detected (ala "fist-pump" demo)
        #   - close gripper all the way
        #   - if gripper closes all the way, no object is gripped
        #   - else object is gripped

    def give_object(self):
        rospy.loginfo("giving object")
        # TODO
        # - move right arm to give-object pose
        # - wait for externally-applied hand motion detected (ala "fist-pump" demo)
        # - open gripper

# Question: what if sequence is pre-empted while robot is holding object?
# - maybe set the object on the floor?
# - v1 is certainly just "do nothing" so robot is still holding the object.
