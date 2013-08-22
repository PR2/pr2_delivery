#!/bin/bash

# This script is just a convenience for measuring current robot base position and orientation.

rosrun tf tf_echo /map /base_footprint
