#!/usr/bin/env bash
docker run \
    -it \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=2 \
    --name machine2 \
    --network tnet \
    gy20073/ros \
    /bin/bash
