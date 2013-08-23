# pr2_delivery #

This is a quick package put together to demonstrate a simple delivery task with the PR2.

It is written as an "action" server.

One run through the action does:

 * Navigate to the get-object pose
 * Put the right arm out and open the gripper
 * Wait for something to be put in the gripper (wait for gripper-wiggle)
 * Close gripper
 * If gripper closes too far, open again and repeat waiting for object
 * Tuck arms to a tucked-with-object pose
 * Navigate to the give-object pose
 * Put the right arm out to hand off the object
 * Wait for gripper-wiggle
 * Open gripper
 * Tuck arms to regular tuck
 * Navigate to "home" pose

This sequence is defined in src/pr2_delivery/DeliverServer.py.

## Programs ##

 * **scripts/deliver**: Runs the DeliverServer action server.

 * **test/test.py**: sends an action goal asking it to make a delivery.
   * The action goal contains the 3 poses: get-object, give-object, and home.
   * *This is a file you will need to copy and modify for use in a new place.*

 * **scripts/print_right_arm_pose**: repeatedly prints the pose of the right arm in the same order that is used inside DeliveryServer.py.  Use this with (or without) the run-stop pressed to record new arm poses if you need to.

 * **test/watch_robot_base_pose.sh**: repeatedly prints the pose of the robot base.  This can be convenient for recording new poses for get-object, give-object, and home.

## Launch files ##

 * **launch/deliver_server.launch**: runs scripts/deliver and a bunch of support nodes which it depends on.
   * Does *not* run map_server.

 * **launch/deliver_server_wg.launch**: runs deliver_server.launch and map_server with a map of Willow Garage.
   * *This is a file you will need to copy and modify for use in a new place.*

## Instructions for use ##

### Initial Setup ###

 * Map the environment
 * Copy launch/deliver_server_wg.launch to launch/deliver_server_location_name.launch and fill in the map info for the map you just made.
 * Record poses of the robot base to use for home, get-object, and give-object using test/watch_robot_base_pose.sh and either navigation using rviz or joystick teleop.
 * Copy test/test.py into scripts/do-one-delivery.py or similar and copy the base-pose info into it (x, y, yaw).

### Starting the Action Server ###

 * roslaunch /etc/ros/robot.launch # to bring up the robot
 * roslaunch pr2_delivery deliver_server_location_name.launch
 * *Robot should not immediately do anything*

### Making a Delivery ###

 * rosrun pr2_delivery scripts/do-one-delivery.py
 * *Robot will tuck arms and navigate to get-object location.*
 * *After delivery is done it will tuck arms and drive to home location.*
