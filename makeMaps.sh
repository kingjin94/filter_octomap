#!/bin/bash
cd "$(dirname "$0")"
mkdir maps
cd maps
source /home/catkin_ws/devel_isolated/setup.bash
rosrun filter_octomap octomap_generate_empty_start 0.02
