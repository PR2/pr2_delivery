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

 * Clone this package (pr2_delivery) into your rosbuild workspace.
 * rosmake pr2_delivery # to build action/message files.
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

## Problems that may happen ##

### Hallucinated obstacles ###

The main problem I've had is the tilting laser hitting the
object-being-carried, which causes hallucinations of obstacles about
30 cm in front of the robot.  You should always keep a GridCells
display in RViz showing the local costmap obstacles so you can see if
this is happening.  It's also useful to show a pointcloud of
/tilt_scan_shadow_filtered with Decay Time set to 3 seconds or so.
That is what has the tilt laser data right after the filter.

To fix a problem like this, either modify the tucked_with_object_pose
in DeliverServer.py OR modify the parameters of downscan_filter in
launch/pr2_delivery_nav/tilt_laser_filters.yaml to make it ignore more
of the lower section of the tilt scan data.

The other important thing is that the current numbers have been tuned
for the storage clipboard.  If you use a different object that is
larger, it may trigger this again and need re-tuning.

### Navigating after arm being out ###

I haven't seen this particularly, but when the arm is out to give or
get, the nav stack will think there are obstacles close to the robot
in front.  When the arms tuck, there is some chance that some obstacle
cells in the costmap won't be cleared immediately and the robot will
try to navigate around them.

This can likely be solved by arranging the pick-up and drop-off poses
such that the robot generally turns around to go to the next pose,
instead of needing to drive straight ahead after getting or giving the
object.

### Gripper Cycling ###

There have been a few times where the gripper sensor nodes don't
launch properly.  When that happens, the robot always thinks its hand
is being shaken.  So when it is waiting for an object to be given, it
constantly closes the hand and re-opens it.  If you give it something
in that state, it is bad because when it is time to *give* the object,
it will think someone is pulling on it immediately and thus open the
gripper and drop the object on the floor.

The solution (that I know of) is to re-launch the deliver action
server.  It hasn't happened twice in a row, so this is all I've done
to solve it so far.

### Joystick Teleop Broken ###

I haven't looked into this, but joystick teleop seems to not work when
this is running.  Maybe a mux issue?
