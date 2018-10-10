#!/usr/bin/env bash

# TODO: auto download the model: figure out the path on the target system
#   don't put private key in the docker, mount it
#   have a good set of mounting paths on the docker

source ../catkin_ws/devel/setup.bash

output_base="/scratch/yang/aws_data/ros_temp/"
output_base="/root/mount/home/bdd/intel/data"
# run this in a seperate window, since we need to record all the time
#bash ./record_rosbag_and_video.bash $output_base &

roslaunch mkz_intel CIL.launch \
    exp_id:="mm45_v4_base_newseg_noiser_TL_lane_structure02_goodsteer_3cam" \
    use_fake_image:="false" \
    fake_video_path:="/scratch/yang/aws_data/mkz/mkz2/inverted_compress.avi" \
    use_auto_traj:="false" \
    gps_traj_file:="traj1.txt" \
    use_left_right:="true"
