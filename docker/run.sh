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

# sudo docker run -it --rm --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES="0,1" gy20073/ros /bin/bash