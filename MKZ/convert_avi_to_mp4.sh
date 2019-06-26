#!/usr/bin/env bash

PATH=$1

for f in $PATH; do
    echo $f
    /usr/local/bin/ffmpeg -i $f -pix_fmt yuv420p -c:v libx264 $f".mp4"
done

# a 16:9 to 4:3 conversion script
# ffmpeg -i input.mp4 -filter:v 'crop=ih/3*4:ih' -c:v libx264 -crf 23 -preset verfast output.mp4
