#!/usr/bin/env bash

# when run on the VM, change 4 things: CACHE_BASE, private_key, use_fake_image, post_config_name

MODEL_NAME="RFS_v7data_lstm" #"RFS_v7data_tcnn1_normalAug" #"RFS_v7data_lstm"
IS_LSTM=false
PREDICTION_HZ=3
STOP_FUTURE_FRAMES=2

CACHE_BASE="/home/bdd/yang/$MODEL_NAME/"
PRIVATE_KEY="/home/bdd/yang/yang_private"
#CACHE_BASE="/home/yang/Downloads/$MODEL_NAME/"
#PRIVATE_KEY="/home/yang/.ssh/id_rsa"

if ls $CACHE_BASE*bestmodel 1> /dev/null 2>&1; then
    echo "have already downloaded the model"
else
    echo "downloading the model..."
    mkdir $CACHE_BASE

    scp -P 222 -i $PRIVATE_KEY yang@aspidochelone.ist.berkeley.edu:/data/yang_cache/driving_model/$MODEL_NAME/"*bestmodel" $CACHE_BASE
fi

MODEL_ACTUAL_PATH=$CACHE_BASE"*bestmodel"
dt=$(date| sed 's/ /_/g')
VIZ_OUTPUT_PATH=$CACHE_BASE"/"$dt".avi"

roslaunch MKZ bdd.launch \
    driving_model_path:=$MODEL_ACTUAL_PATH \
    config_name:=$MODEL_NAME \
    is_lstm:=$IS_LSTM \
    use_real_image:=true \
    use_fake_image:=false \
    fake_tfrecord_path:=/home/yang/Desktop/real_car/vm_share/48.tfrecords \
    viz_output_path:=$VIZ_OUTPUT_PATH \ #post_config_name:=common_config_ubuntuVM
    prediction_hz:=$PREDICTION_HZ \
    stop_future_frames:=$STOP_FUTURE_FRAMES
