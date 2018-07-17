#!/usr/bin/env bash
docker run \
    -it \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/usr/local/cuda:/usr/local/cuda" \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=1 \
    --name machine5 \
    --network tnet \
    ubuntu:16.04 \
    /bin/bash
