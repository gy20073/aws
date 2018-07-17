#!/usr/bin/env bash

~/darknet/darknet \
    detector train \
    TL.data \
    yolov3-TL.cfg \
    /data/yang/code/aws/data/darknet53.conv.74 \
    -gpus 7
