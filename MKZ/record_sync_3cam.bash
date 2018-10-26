#!/usr/bin/env bash
output_video="/root/mount/home/bdd/intel/data/3cam.mp4"

source ../catkin_ws_docker/devel/setup.bash

roslaunch mkz_intel record_3cam_sync.launch \
    viz_output_path:=$output_video
