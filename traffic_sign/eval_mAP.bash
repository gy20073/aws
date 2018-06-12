#!/usr/bin/env bash
declare -a arr=("yolov3-CL_900.weights" "yolov3-CL_10000.weights" "yolov3-CL.backup")
for MODEL_NAME in "${arr[@]}"
do
    export CUDA_VISIBLE_DEVICES=7
    current=$(pwd)
    cd ~/darknet
    ~/darknet/darknet \
        detector valid \
        /data/yang/code/aws/traffic_sign/CL.data \
        /data/yang/code/aws/traffic_sign/yolov3-CL.cfg.test \
        /scratch/yang/aws_data/coco_lisa_v2/backup/$MODEL_NAME

    cd $current

    #mv ~/darknet/results /data/yang/code/aws/scratch/coco_lisa_v2/results_$MODEL_NAME
    mv ~/darknet/results /data/yang/code/aws/scratch/coco_lisa_v2/results_coco_$MODEL_NAME
    mkdir ~/darknet/results

    : '
    python /data/yang/code/aws/voc_eval.py \
        /data/yang/code/aws/traffic_sign/CL.names \
        /data/yang/code/aws/scratch/coco_lisa_v2/results_$MODEL_NAME \
        /scratch/yang/aws_data/LISA_EXT/labels \
        /scratch/yang/aws_data/LISA_EXT/val.txt
    '

    python /data/yang/code/aws/voc_eval.py \
        /data/yang/code/aws/traffic_sign/CL.names \
        /data/yang/code/aws/scratch/coco_lisa_v2/results_coco_$MODEL_NAME \
        /data/yang/code/aws/scratch/coco/labels/val2014/

done
