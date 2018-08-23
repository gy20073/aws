#!/usr/bin/env bash

# TODO: auto download the model: figure out the path on the target system
#   don't put private key in the docker, mount it
#   have a good set of mounting paths on the docker
# TODO: support the real mode: mainly change the launch file

source ../catkin_ws/devel/setup.bash

output_base="/scratch/yang/aws_data/ros_temp/"
# run this in a seperate window, since we need to record all the time
#bash ./record_rosbag_and_video.bash $output_base &

roslaunch mkz_intel CIL.launch \
    exp_id:="mm45_v5_ablate_base" \
    use_fake_image:="true" \
    use_real_image:="false"
