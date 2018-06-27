#!/usr/bin/env bash
docker run \
    -it \
    --rm \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --runtime=nvidia \
    --device /dev/nvidia1:/dev/nvidia0 \
    --device /dev/nvidiactl:/dev/nvidiactl \
    --device /dev/nvidia-uvm:/dev/nvidia-uvm \
    --device /dev/nvidia-uvm-tools:/dev/nvidia-uvm-tools \
    --device /dev/nvidia-modeset:/dev/nvidia-modeset \
    -e NVIDIA_VISIBLE_DEVICES=7 \
    --name machine2 \
    --network tnet \
    gy20073/ros \
    /bin/bash
