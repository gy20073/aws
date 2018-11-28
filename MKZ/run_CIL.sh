#!/usr/bin/env bash

# TODO: auto download the model: figure out the path on the target system
#   don't put private key in the docker, mount it
#   have a good set of mounting paths on the docker

source ../catkin_ws_docker/devel/setup.bash

output_base="/scratch/yang/aws_data/ros_temp/"
output_base="/root/mount/home/bdd/intel/data"
# run this in a seperate window, since we need to record all the time
#bash ./record_rosbag_and_video.bash $output_base &

# mm45_v4_base_newseg_noiser_TL_lane_structure02_goodsteer_waypoint_zoom_stdnorm_v5_3cam.py
# mm45_v4_base_newseg_noiser_TL_lane_structure02_goodsteer_waypoint_zoom_stdnorm_v5.py
# mm45_v4_base_newseg_noiser_TL_lane_structure02_goodsteer_waypoint_zoom_stdnorm_v5_abn
# mm45_v4_base_newseg_noiser_TL_lane_structure02_goodsteer_waypoint_zoom_stdnorm_v5_abn_nozoom
# mm45_v4_base_newseg_noiser_TL_lane_structure02_goodsteer_waypoint_zoom_stdnorm_v5_3cam_abn
# mm45_v4_base_newseg_noiser_TL_lane_structure02_goodsteer_waypoint_zoom_stdnorm_v5_3cam_2town.py

# mm45_v4_wp2town3cam_rawcontrol
# mm45_v4_wp2town3cam_parallel_control
# mm45_v4_wp2town3cam_rawcontrol_1cam
# mm45_v4_wp2town3cam_stacked_control
# the waypoint model with right turns

# mm45_v4_wp2town3cam_correct_town2
# mm45_v4_wp2town3cam_parallel_control_correct_town2
# mm45_v4_wp2town3cam_3town
# mm45_v4_wp2town3cam_parallel_control_3towns
# mm45_v4_wp2town3cam_2p2town

# mm45_v4_wp2town3cam_2p2town
# mm45_v4_wp2town3cam_2p2town_real
# mm45_v4_wp2town3cam_parallel_control_2p2town_real
# mm45_v4_base_newseg_noiser_TL_lane_structure02_goodsteer_waypoint_zoom_stdnorm_v5_3cam

# mm45_v4_wp2town3cam_2p3town
# mm45_v4_wp2town3cam_parallel_control_2p3town

# mm45_v4_wp2town3cam_parallel_control_human3hours

# models with map and with sensor dropout
# mm45_v4_wp2town3cam_2p3town_map
# mm45_v4_wp2town3cam_2p3town_map_sensor_dropout
# mm45_v4_wp2town3cam_parallel_control_2p3town_map
# mm45_v4_wp2town3cam_parallel_control_2p3town_sensor_dropout

roslaunch mkz_intel CIL.launch \
    exp_id:="mm45_v4_wp2town3cam_2p3town_map_sensor_dropout" \
    use_fake_image:="false" \
    fake_video_path:="/scratch/yang/aws_data/mkz/mkz2/inverted_compress.avi" \
    use_auto_traj:="false" \
    gps_traj_file:="traj1.txt" \
    use_left_right:="true" \
    use_waypoint:="true"
