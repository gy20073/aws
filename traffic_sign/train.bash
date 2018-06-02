#!/usr/bin/env bash

~/darknet/darknet \
    detector train \
    CL.data \
    yolov3-CL.cfg \
    /data/yang/code/aws/data/darknet53.conv.74 \
    -gpus 6,7
