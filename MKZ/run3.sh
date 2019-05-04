#!/usr/bin/env bash


bash ./run_raw_control_pid.sh &
bash ./record_mkz2.bash &
bash ./run_CIL.sh
