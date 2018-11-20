#!/usr/bin/env bash

PATH=$1

for f in $PATH; do
    echo $f
    /usr/local/bin/ffmpeg -i $f -pix_fmt yuv420p -c:v libx264 $f".mp4"
done
