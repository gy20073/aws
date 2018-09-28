#!python2
"""
Add bounding boxes and control signals to videos
"""
import os
import pickle
import argparse
import json

import cv2
import numpy as np

from utils.utils import *
from utils.data_utils import find_time_match_info
from glob import glob
import os

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description=None)
    parser.add_argument(
        '-p',
        '--path',
        help='Directory to retrieve images',
        required=True)
    args = parser.parse_args()


    output_demo = True
    # read info list
    imagelist = sorted(glob(os.path.join(args.path,'*','*_final.png')))

    fourcc = cv2.VideoWriter_fourcc(*'H264')
    out = cv2.VideoWriter(os.path.join(args.path,'out.mp4'), fourcc, 15.0, (1920, 1080))

    for image in imagelist:
        frame = cv2.imread(image)
        out.write(frame)
        #cv2.imshow('frame', frame)
        # time.sleep(0.1)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break

    out.release()
    #cv2.destroyAllWindows()