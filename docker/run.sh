#!/usr/bin/env bash
docker run \
    -it \
    --rm \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=7 \
    --name machine0 \
    --network tnet \
    gy20073/ros \
    /bin/bash
