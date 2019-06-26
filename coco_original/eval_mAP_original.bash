#!/usr/bin/env bash

export CUDA_VISIBLE_DEVICES=7

~/darknet/darknet \
    detector valid \
    /data/yang/code/aws/coco_original/coco.data \
    /data/yang/code/aws/coco_original/yolov3.cfg \
    /data/yang/code/aws/data/yolov3.weights

#mv ~/darknet/results /data/yang/code/aws/scratch/coco_lisa/results_coco
# unable to evaluate with my script, because, the data in val2014 is missing some files