#!/usr/bin/env bash

timestamp() {
  date +"%Y-%m-%d_%H-%M-%S"
}

ts=$(timestamp)

OUTPUT_PREFIX="/media/bdd/Samsung_T5/human/"$ts
mkdir $OUTPUT_PREFIX

OUTPUT_BAG=$OUTPUT_PREFIX"/bag"
IMAGE_MIDDLE="/compressed0"
IMAGE_LEFT="/compressed1"
IMAGE_RIGHT="/compressed2"

python nodes/image_downsampler_compressed.py &

# rosbag command goes here
rosbag record \
--split --duration 5m \
-b 30720 \
--lz4 \
--output-name $OUTPUT_BAG \
/nmea_sentence \
/vehicle/joint_states /vehicle/req_accel /vehicle/suspension_report \
/vehicle/brake_info_report /vehicle/brake_report \
/vehicle/gear_report /vehicle/misc_1_report  \
/vehicle/gps/fix /vehicle/gps/time /vehicle/gps/vel \
/vehicle/imu/data_raw \
/vehicle/steering_report \
/vehicle/surround_report  \
/vehicle/throttle_info_report /vehicle/throttle_report \
/vehicle/tire_pressure_report \
/vehicle/wheel_position_report /vehicle/wheel_speed_report \
/vehicle/twist /vehicle/twist_controller/parameter_descriptions /vehicle/twist_controller/parameter_updates \
/vehicle/filtered_accel /vehicle/req_accel \
/xsens/fix /xsens/imu/data /xsens/imu/mag /xsens/imu_data_str /xsens/pressure /xsens/time_reference /xsens/velocity \
$IMAGE_MIDDLE $IMAGE_LEFT $IMAGE_RIGHT
