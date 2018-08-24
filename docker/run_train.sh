#!/usr/bin/env bash
docker run \
    -it \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/scratch/yang/:/scratch/yang/:rw" \
    --volume="/data2/yang/code/aws:/data/yang/code/aws:rw" \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES="0,1,2,3,4,5,6,7" \
    --net=host \
    gy20073/ros \
    /bin/bash

    #docker run -it --rm --net=host --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES="0,1" -e ROS_MASTER_URI=$ROS_MASTER_URI -v "/:/root/mount:rw" gy20073/ros /bin/bash