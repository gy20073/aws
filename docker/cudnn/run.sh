#!/usr/bin/env bash
docker run \
    -it \
    --rm \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=1 \
    --name machine4 \
    gy20073/cudnn_torch \
    /bin/bash
