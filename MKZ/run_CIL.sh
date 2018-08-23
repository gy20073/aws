#!/usr/bin/env bash

# TODO: auto download the model
# TODO: support the real mode

source ../catkin_ws/devel/setup.bash

output_base="/scratch/yang/aws_data/ros_temp/"
#bash ./record_rosbag_and_video.bash $output_base &

roslaunch mkz_intel CIL.launch \
    exp_id:="mm45_v5_ablate_base" \
    use_fake_image:="true" \
    use_real_image:="false"
