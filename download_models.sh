#!/usr/bin/env bash

CMD="/root/gdown.pl/gdown.pl"
mkdir models
cd models
$CMD https://drive.google.com/file/d/159W0FtmYUbs0byjN5GI2Rf6EotstRGe9/view TrafficSign-yolov3-CL.backup
$CMD https://drive.google.com/file/d/1FAECml0gEZuGvSkarlu2v47cMx4f3iPZ/view Segmentation_LinkNet_576_768.stat.t7
$CMD https://drive.google.com/file/d/1Q_amXRt7GLsrb-j0HxCobH-rk2QhCn3q/view Segmentation-LinkNet-model-152.net
$CMD https://drive.google.com/file/d/1iZPcpeWNtSz1azvy1PDEFC4m-tcexO-b/view TrafficLight-yolov3-TL.backup
$CMD https://drive.google.com/file/d/1vOLggzNUfgbu1TVewLANG9mXN57tlK8R/view COCO-yolov3.weights
