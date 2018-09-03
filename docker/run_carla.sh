#!/usr/bin/env bash

carla_gpu=$1
city_name=$2
port=$3

docker run \
    --rm \
    --volume="/scratch/yang/:/scratch/yang/:rw" \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=$carla_gpu \
    --net=host \
    gy20073/ros \
    /scratch/yang/aws_data/carla_0.8.4/CarlaUE4.sh \
    /Game/Maps/$city_name \
    -carla-server \
    -carla-settings="/scratch/yang/aws_data/carla_0.8.4/carla_long_timeout.ini" \
    -carla-world-port=$port


