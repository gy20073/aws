#!/usr/bin/env bash

export CUDA_VISIBLE_DEVICES=7
current=$(pwd)
cd ~/darknet

~/darknet/darknet \
    detector test \
    /data/yang/code/aws/traffic_sign/CL.data \
    /data/yang/code/aws/traffic_sign/yolov3-CL.cfg.test \
    /scratch/yang/aws_data/coco_lisa_v2/backup/yolov3-CL.backup \
    -thresh 0.1

    #/scratch/yang/aws_data/coco/images/val2014/COCO_val2014_000000478868.jpg \
    #/scratch/yang/aws_data/coco/images/val2014/COCO_val2014_000000151267.jpg \
    #/scratch/yang/aws_data/coco/images/val2014/COCO_val2014_000000402313.jpg \
cd $current