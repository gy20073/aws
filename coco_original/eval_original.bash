#!/usr/bin/env bash

export CUDA_VISIBLE_DEVICES=5

~/darknet/darknet \
    detector test \
    /home/yang/darknet/cfg/coco.data \
    /home/yang/darknet/cfg/yolov3.cfg \
    /data/yang/code/aws/data/yolov3.weights \
    -thresh 0.001
