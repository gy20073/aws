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

# mm45_v4_wp2town3cam_2p3town_map_sensor_dropout_rfssim
# mm45_v4_wp2town3cam_parallel_control_2p3town_map_sensor_dropout_rfssim
# mm45_v4_wp2town3cam_parallel_control_2p3town_map_sensor_dropout_rfssim_moremap

# 2019-01-11
# "mm45_v4_wp2town3cam_parallel_control_2p3town_map_sensor_dropout_rfssim_lessmap_simv2"
# "mm45_v4_wp2town3cam_parallel_control_2p3town_map_sensor_dropout_rfssim_moremap_simv2"
# "mm45_v4_wp2town3cam_parallel_control_2p3town_map_sensor_dropout_moremap"
# "mm45_v4_wp2town3cam_parallel_control_2p3town_map_sensor_dropout_rfssim_moremap"

# 2019-01-18: swing a lot, the center camera rotation is bad
# mm45_v4_PcSensordropLessmap_rfsv4_extra_structure_noise_lanecolor
# mm45_v4_PcSensordropLessmap_rfsv4_extra_structure_noise

# 2019-01-25 TODO, debug the swing problem and adjusted the P parameter
# the 01-11 and the 01-18 models

# 2019-02-01, missing on my laptop, since it's stored on the MKZ
# mm45_v4_PcSensordropLessmap_rfsv5_lanecolor_accurate_map
# mm45_v4_PcSensordropLessmap_rfsv5_extra_structure_noise_lanecolor
# mm45_v4_PcSensordropLessmap_rfsv5_extra_structure_noise

# 2019-02-11 TODO
# mm45_v4_PcSensordropLessmap_rfsv45_extra_structure_noise_lanecolor
# mm45_v4_PcSensordropLessmap_rfsv45_extra_structure_noise_lanecolor_drivable

# 2019-02-20 TODO
# mm45_v4_PcSensordropLessmap_rfsv45_extra_structure_noise_nocolor_drivable
# mm45_v4_PcSensordropLessmap_rfsv45_extra_structure_noise_nocolor_onroad
# mm45_v4_PcSensordropLessmap_rfsv45_extra_structure_noise_nocolor_onroad_accurate_map
# mm45_v4_PcSensordropLessmap_rfsv45_extra_structure_noise_nocolor_shoulder
# mm45_v4_PcSensordropLessmap_rfsv45_extra_structure_noise_nocolor_shoulder_accurate_map

# 2019-03-01
# mm45_v4_SqnoiseShoulder_rfsv2
# mm45_v4_SqnoiseShoulder_rfsv25
# mm45_v4_SqnoiseShoulder_rfsv25_accuratemap
# mm45_v4_SqnoiseShoulder_rfsv2_accuratemap

roslaunch mkz_intel CIL.launch \
    exp_id:="mm45_v4_PcSensordropLessmap_rfsv4_extra_structure_noise" \
    use_fake_image:="false" \
    fake_video_path:="/scratch/yang/aws_data/mkz/mkz2/inverted_compress.avi" \
    use_auto_traj:="false" \
    gps_traj_file:="traj1.txt" \
    use_left_right:="true" \
    use_waypoint:="false"
