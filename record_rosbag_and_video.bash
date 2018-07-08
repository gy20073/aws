#!/usr/bin/env bash

IMG_TOPIC="/vis_continuous_full"
OUTPUT="/root/mount/store.avi"

# rosbag command goes here

rosrun image_view video_recorder image:=$IMG_TOPIC _filename:=$OUTPUT _codec:="X264" _encoding:="rgb8"
