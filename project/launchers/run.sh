#!/bin/bash


source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

set -eux

dt-exec-BG roslaunch --wait agent agent_node.launch
# dt-exec-BG roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME

dt-exec-FG roslaunch --wait object_detection object_detector_node.launch veh:=$VEHICLE_NAME || true

copy-ros-logs
