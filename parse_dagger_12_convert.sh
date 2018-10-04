#!/usr/bin/env bash

input_path="/Volumes/conditionR6/dagger_yang"
output_path="/Volumes/Data/Downloads/vladlen/data/gta_batch3"
num_threads="4"

python3 gta_tools/hybrid_client/parse_dagger.py $input_path
python3 gta_tools/hybrid_client/parse_dagger02.py $input_path

declare -a arr=("val" "train")
for set in "${arr[@]}"
do
    python2 CIL_modular/convert_dqwang_to_h5.py $input_path $output_path"/"$set $set $num_threads
done

