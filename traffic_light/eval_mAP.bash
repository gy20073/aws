#!/usr/bin/env bash
#declare -a arr=("yolov3-TL_900.weights" "yolov3-TL_10000.weights" "yolov3-TL_20000.weights" "yolov3-TL.backup")
#declare -a arr=("yolov3-TL_10000.weights" "yolov3-TL_20000.weights" "yolov3-TL.backup")
declare -a arr=("yolov3-TL_10000.weights")

for MODEL_NAME in "${arr[@]}"
do
    export CUDA_VISIBLE_DEVICES=7
    current=$(pwd)
    cd ~/darknet
    ~/darknet/darknet \
        detector valid \
        /data/yang/code/aws/traffic_light/TL.data \
        /data/yang/code/aws/traffic_light/yolov3-TL.cfg.test \
        /scratch/yang/aws_data/bdd100k/yolo_format/backup/$MODEL_NAME
    cd $current

    mv ~/darknet/results /scratch/yang/aws_data/bdd100k/yolo_format/mAPs/results_$MODEL_NAME
    mkdir ~/darknet/results

    python /data/yang/code/aws/voc_eval.py \
        /data/yang/code/aws/traffic_light/TL.names \
        /scratch/yang/aws_data/bdd100k/yolo_format/mAPs/results_$MODEL_NAME \
        /data/yang/code/aws/scratch/bdd100k/yolo_format/labels/val

done
