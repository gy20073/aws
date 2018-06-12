#!/usr/bin/env bash

current=$(pwd)
cd ~/drn

python3 segment.py test \
    -d /data/yang/code/aws/drivable_area/data_mkz \
    -c 3 \
    --arch drn_d_22 \
    --pretrained /data2/yang_cache/aws_data/drn_d_22_drivable-e5d3dc9c.pth \
    --phase test \
    --batch-size 1

cd $current
