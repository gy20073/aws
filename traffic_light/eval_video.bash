#!/usr/bin/env bash

# calling format
GPU=$1
VIDEO_PATH="/scratch/yang/aws_data/mkz/video_highqual.mp4"
VIDEO_PATH=$2
THRES=$3

DarknetPath="/home/yang/code/aws/yolo/darknet"

export CUDA_VISIBLE_DEVICES=$GPU
CACHE_PATH=$VIDEO_PATH"-cache-thres="$THRES
mkdir $CACHE_PATH

current=$(pwd)
cd $DarknetPath

# this does not produce the correct result since, the output is different from this command, and I haven't figure out why
# ~/code/aws/yolo/darknet
# sudo ./darknet detect ../../traffic_light/yolov3-TL.cfg.test /shared/yang/data1/aws_data/bdd100k/yolo_format/backup/yolov3-TL_10000.weights /shared/yang/data1/aws_data/bdd100k/yolo_format/images/val/video/video.mp4-cache-thres=0.5/det_00000000.jpg
$DarknetPath"/"darknet \
    detector demo \
    /data/yang/code/aws/traffic_light/TL.data \
    /data/yang/code/aws/traffic_light/yolov3-TL.cfg.test \
    /shared/yang/data1/aws_data/bdd100k/yolo_format/backup/yolov3-TL_10000.weights \
    $VIDEO_PATH \
    -prefix $CACHE_PATH"/det" \
    -thresh $THRES

cd $current

ffmpeg -threads 16 -framerate 30 -pattern_type glob -i $CACHE_PATH"/*.jpg" -c:v libx264 -crf 28 -preset veryfast $VIDEO_PATH"-det-thres="$THRES".mp4"
