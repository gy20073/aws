#!/usr/bin/env bash

cd /data/yang/code/aws/MKZ/
source /root/.profile
source /root/.bashrc
bash ./run_raw_control_pid.sh &
bash ./record_mkz2.bash &
bash ./run_CIL.sh
