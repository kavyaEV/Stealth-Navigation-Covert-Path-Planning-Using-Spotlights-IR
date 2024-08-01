#!/bin/bash

# helper script to terminate all ROS-related processes: run when done with
# reinforcement learning to have a clean environment
killall -9 roslaunch rostopic rosout rosmaster rviz stageros amcl map_server agent python3 2>/dev/null
