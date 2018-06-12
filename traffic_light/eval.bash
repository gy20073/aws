#!/usr/bin/env bash

export CUDA_VISIBLE_DEVICES=7
current=$(pwd)
cd ~/darknet

~/darknet/darknet \
    detector test \
    /data/yang/code/aws/traffic_light/TL.data \
    /data/yang/code/aws/traffic_light/yolov3-TL.cfg.test \
    /scratch/yang/aws_data/bdd100k/yolo_format/backup/yolov3-TL.backup \
    -thresh 0.05

cd $current
    #/scratch/yang/aws_data/coco/images/val2014/COCO_val2014_000000478868.jpg \
    #/scratch/yang/aws_data/coco/images/val2014/COCO_val2014_000000151267.jpg \
    #/scratch/yang/aws_data/coco/images/val2014/COCO_val2014_000000402313.jpg \
