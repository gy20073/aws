#!/usr/bin/env bash
docker run \
    -it \
    --rm \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/scratch/yang/aws_data/:/root/mount:rw" \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES="6,7" \
    --name machine0 \
    --network tnet \
    gy20073/ros \
    /bin/bash

    # docker run -it --rm --net=host --name=yang_main --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES="0,1" -e ROS_MASTER_URI=$ROS_MASTER_URI -v "/:/root/mount:rw" -v "/dev:/dev:rw" gy20073/ros /bin/bash
    #docker run -it --rm --net=host --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES="0,1" -e ROS_MASTER_URI=$ROS_MASTER_URI -v "/:/root/mount:rw" -v "/dev:/dev:rw" gy20073/ros /bin/bash