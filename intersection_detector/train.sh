#!/bin/bash

python perception_intersection/train.py  /media/grossanc/5ee96718-a500-4356-988b-9f6a66773bff/inter_detection/train_images_42140.tensor /media/grossanc/5ee96718-a500-4356-988b-9f6a66773bff/inter_detection/train_catLabels_42140.tensor /media/grossanc/5ee96718-a500-4356-988b-9f6a66773bff/inter_detection/test_images_12284.tensor /media/grossanc/5ee96718-a500-4356-988b-9f6a66773bff/inter_detection/test_catLabels_12284.tensor --arch=alexnet 

