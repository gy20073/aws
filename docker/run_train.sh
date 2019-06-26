#!/usr/bin/env bash

# begin of path specifications
CODE_PATH="/home/yang/aws"
DATA="/home/yang/data/aws_data2/"
mount_dataset_base="/shared/yang/data1/aws_data/carla_collect"
mount_dataset_id1="steer103_v5_way_v2_town02"
# end of path specification

# make dirs
mkdir -p $DATA
mkdir $DATA"carla_collect"
mkdir $DATA"CIL_modular_data"
mkdir $DATA"CIL_modular_data/benchmark_all"
mkdir $DATA"CIL_modular_data/_benchmarks_results"
mkdir $DATA"CIL_modular_data/models"
mkdir $DATA"CIL_modular_data/results"

PORT=$DATA"CIL_modular_data/port.txt"
if [ ! -f $PORT ]; then
    echo "3000" > $PORT
fi

docker run \
    -it \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES="0,1,2,3,4,5,6,7" \
    --volume=$CODE_PATH":/data/yang/code/aws:rw" \
    --volume=$DATA":/home/yang/data/aws_data:rw" \
    --volume=$DATA":/scratch/yang/aws_data:rw" \
    --volume=$mount_dataset_base$mount_dataset_id1":/home/yang/data/aws_data/carla_collect/"$mount_dataset_id1":rw" \
    --volume=$mount_dataset_base$mount_dataset_id1":/scratch/yang/aws_data/carla_collect/"$mount_dataset_id1":rw" \
    gy20073/ros \
    /bin/bash

    #--volume="/scratch/yang/:/scratch/yang/:rw" \
    #
