#!/usr/bin/env bash
declare -a images=("/home/yang/monodepth/samples/Trigger-1529019612790029-0" "/home/yang/monodepth/samples/Trigger-1529019653759981-0" "/home/yang/monodepth/samples/Trigger-1529019810323870-0")
declare -a models=("model_city2eigen" "model_city2kitti" "model_cityscapes" "model_eigen" "model_kitti")

for an_image in "${images[@]}"
do
    for a_model in "${models[@]}"
    do
        python /home/yang/monodepth/monodepth_simple.py \
         --checkpoint_path /home/yang/monodepth/models/$a_model/$a_model \
         --image_path $an_image".jpg"
         # move the image
         mv $an_image"_disp.png" $an_image"_"$a_model"_disp.png"
         rm $an_image"_disp.npy"
    done
done