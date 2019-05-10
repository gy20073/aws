#!/usr/bin/env bash
docker run \
    -it \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/:/root/mount/:rw" \
    --volume="/home/bdd/intel/data/:/scratch/yang/aws_data/:rw" \
    --volume="/home/bdd/intel/aws:/data/yang/code/aws:rw" \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES="0,1" \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    --net=host \
    gy20073/ros \
    /data/yang/code/aws/MKZ/run3.sh
