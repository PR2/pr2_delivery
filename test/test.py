#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_delivery')
import rospy
import actionlib
import geometry_msgs.msg
from pr2_delivery.msg import DeliverAction, DeliverGoal

if __name__ == '__main__':
    rospy.loginfo("starting...")
    rospy.init_node('deliver_test_client')
    client = actionlib.SimpleActionClient('deliver', DeliverAction)
    rospy.loginfo("waiting for action server...")
    client.wait_for_server()

    goal = DeliverGoal()
    # Fill in goal here.
    rospy.loginfo("sending goal.")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(50.0))
    rospy.loginfo("got action finished.")
    rospy.sleep(50)
