#!/usr/bin/env bash

# fake mode
exp_id="mm45_v4_perception_straight3constantaug_lessdrop_yangv2net_segonly"

python ./nodes/drive_MKZ.py $exp_id &

python ./nodes/keyboard_input.py &

bash ./record_rosbag_and_video.bash

python ./nodes/fake_video_publisher.py &