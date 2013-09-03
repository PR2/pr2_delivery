#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_delivery')
import rospy
import actionlib
import geometry_msgs.msg
import tf.transformations
from pr2_delivery.msg import DeliverAction, DeliverGoal

import yaml
import os

def pose_from_yaml(filename):
    stream = open( filename )
    pose_dict = yaml.load( stream )
    
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = '/map'
    
    pose.pose.position.x = pose_dict['pose']['position']['x']
    pose.pose.position.y = pose_dict['pose']['position']['y']
    pose.pose.position.z = pose_dict['pose']['position']['z']
    
    pose.pose.orientation.x = pose_dict['pose']['orientation']['x']
    pose.pose.orientation.y = pose_dict['pose']['orientation']['y']
    pose.pose.orientation.z = pose_dict['pose']['orientation']['z']
    pose.pose.orientation.w = pose_dict['pose']['orientation']['w']
    
    return pose

if __name__ == '__main__':
    rospy.loginfo("starting...")
    rospy.init_node('deliver_test_client')
    client = actionlib.SimpleActionClient('deliver', DeliverAction)
    rospy.loginfo("waiting for action server...")
    client.wait_for_server()

    data_dir = rospy.get_param("~data_dir")
    rospy.loginfo("Using %s as data directory"%(data_dir))

    goal = DeliverGoal()
    # Fill in goal here.
    goal.get_object_pose = pose_from_yaml(os.path.join(data_dir,'get_object_pose.yaml'))
    goal.give_object_pose = pose_from_yaml(os.path.join(data_dir,'give_object_pose.yaml'))
    goal.return_home_pose = pose_from_yaml(os.path.join(data_dir,'return_home_pose.yaml'))

    rospy.loginfo("sending goal.")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(50.0))
    rospy.loginfo("got action finished.")
    rospy.sleep(50)
