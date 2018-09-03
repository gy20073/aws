#!/usr/bin/env bash

carla_gpu=$1
city_name=$2
port=$3

sudo docker run -p 2000-2002:2000-2002 --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=2 carla:0.8.4 ./CarlaUE4.sh \
/Game/Maps/Town01 -benchmark -carla-server -fps=5 -world-port=2000 -carla-no-hud

sudo docker run -p 2000-2002:2000-2002 --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=2 -e SDL_VIDEODRIVER=offscreen carlasim/carla:0.8.4  ./CarlaUE4.sh /Game/Maps/Town01 -benchmark -carla-server -fps=10 -world-port=2000 -windowed -ResX=100 -ResY=100 -carla-no-hud
