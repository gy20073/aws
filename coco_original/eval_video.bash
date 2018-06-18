#!/usr/bin/env bash

# calling format
GPU=$1
VIDEO_PATH="/scratch/yang/aws_data/mkz/video_highqual_coco.mp4"
VIDEO_PATH=$2
THRES=$3

export CUDA_VISIBLE_DEVICES=$GPU
CACHE_PATH=$VIDEO_PATH"-cache-thres="$THRES
mkdir $CACHE_PATH

current=$(pwd)
cd ~/darknet

~/darknet/darknet \
    detector demo \
    /home/yang/darknet/cfg/coco.data \
    /home/yang/darknet/cfg/yolov3.cfg \
    /data/yang/code/aws/data/yolov3.weights \
    $VIDEO_PATH \
    -prefix $CACHE_PATH"/det" \
    -thresh $THRES

cd $current

ffmpeg -threads 16 -framerate 30 -pattern_type glob -i $CACHE_PATH"/*.jpg" -c:v libx264 -crf 40 -preset veryfast $VIDEO_PATH"-det-thres="$THRES".mp4"
