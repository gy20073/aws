#!/usr/bin/env bash
bash setup.sh
source devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=release
roslaunch path_follower pid_waypoints.launch
