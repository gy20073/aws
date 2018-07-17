#!/usr/bin/env bash

OUTPUT_PREFIX="/root/mount/media/bdd/ExternalSto/mkz_20180717"
# we are saving the images into a video
ORIGINAL_IMAGE_TOPIC="/image_sender_0/flipped"
OUTPUT_ORIGINAL_VIDEO=$OUTPUT_PREFIX"/original_images.avi"
# all topics with downsampled images into a rosbag
DOWNSAMPLED_IMAGE_TOPIC="/image_sender_0/H576W768"
OUTPUT_BAG=$OUTPUT_PREFIX"/recording.bag"
# also the visualization image with a video
VIZ_IMAGE_TOPIC="/vis_continuous_full"
OUTPUT_VIZ_VIDEO=$OUTPUT_PREFIX"/visualization_images.avi"

# rosbag command goes here
rosbag record \
--split --duration 5m \
-b 30720 \
--output-name $OUTPUT_BAG \
/nmea_sentence \
/vehicle/brake_cmd /vehicle/brake_info_report /vehicle/brake_report \
/vehicle/gear_report /vehicle/joint_states /vehicle/misc_1_report /vehicle/req_accel \
/vehicle/gps/fix /vehicle/gps/time /vehicle/gps/vel \
/vehicle/imu/data_raw \
/vehicle/steering_cmd /vehicle/steering_report \
/vehicle/surround_report /vehicle/suspension_report \
/vehicle/throttle_cmd /vehicle/throttle_info_report /vehicle/throttle_report \
/vehicle/tire_pressure_report /vehicle/turn_signal_cmd \
/vehicle/wheel_position_report /vehicle/wheel_speed_report \
/vehicle/twist /vehicle/twist_controller/parameter_descriptions /vehicle/twist_controller/parameter_updates \
/xsens/fix /xsens/imu/data /xsens/imu/mag /xsens/imu_data_str /xsens/time_reference \
$DOWNSAMPLED_IMAGE_TOPIC &

rosrun image_view video_recorder image:=$ORIGINAL_IMAGE_TOPIC _filename:=$OUTPUT_ORIGINAL_VIDEO _codec:="X264" _encoding:="bgr8" &

rosrun image_view video_recorder image:=$VIZ_IMAGE_TOPIC _filename:=$OUTPUT_VIZ_VIDEO _codec:="X264" _encoding:="bgr8" &
