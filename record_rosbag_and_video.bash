#!/usr/bin/env bash

IMG_TOPIC="/temp_image_topic"
OUTPUT="/data/store.avi"

# rosbag command goes here

rosrun image_view video_recorder image:=$IMG_TOPIC _filename=$OUTPUT _codec="X264"
