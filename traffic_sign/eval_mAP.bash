#!/usr/bin/env bash
declare -a arr=("yolov3-CL_200.weights" "yolov3-CL_400.weights" "yolov3-CL_600.weights" "yolov3-CL_800.weights" "yolov3-CL_10000.weights")
for MODEL_NAME in "${arr[@]}"
do
    export CUDA_VISIBLE_DEVICES=7
    current=$(pwd)
    cd ~/darknet
    ~/darknet/darknet \
        detector valid \
        /data/yang/code/aws/traffic_sign/CL.data \
        /data/yang/code/aws/traffic_sign/yolov3-CL.cfg.test \
        /scratch/yang/aws_data/coco_lisa/backup/$MODEL_NAME
    cd $current

    mv ~/darknet/results /data/yang/code/aws/scratch/coco_lisa/results_coco_$MODEL_NAME
    mkdir ~/darknet/results

    python /data/yang/code/aws/voc_eval.py \
        /data/yang/code/aws/traffic_sign/CL.names \
        /data/yang/code/aws/scratch/coco_lisa/results_coco_$MODEL_NAME \
        /data/yang/code/aws/scratch/coco/labels/val2014/

done
