import math
import os
import pickle
import random
import shutil
import time

import cv2
import numpy as np
import json

from flask import jsonify
from tqdm import tqdm

from utils.roadinfoparser import parseroadinfo
from utils.utils import *
from utils.data_utils import find_time_match_info

vec_class = {
    0: "car",
    1: "bike",
    2: "bicycle",
    3: "quadbike",
    4: "boat",
    5: "plane",
    6: "helicopter",
    7: "train",
    8: "submersible",
    9: "unknown"
}

vec_detail_class = {
    0: 'Compacts',
    1: 'Sedans',
    2: 'SUVs',
    3: 'Coupes',
    4: 'Muscle',
    5: 'Sports Classics',
    6: 'Sports',
    7: 'Super',
    8: 'Motorcycles',
    9: 'Off-road',
    10: 'Industrial',
    11: 'Utility',
    12: 'Vans',
    13: 'Cycles',
    14: 'Boats',
    15: 'Helicopters',
    16: 'Planes',
    17: 'Service',
    18: 'Emergency',
    19: 'Military',
    20: 'Commercial',
    21: 'Trains'
}


def count_subcategories(dset_dir):
    subc_bbox_num = dict()
    subc_track_num = dict()

    dset_list = os.listdir(dset_dir)
    dset_list = [dset for dset in dset_list if 'urban' in dset]
    for dset in tqdm(dset_list):
        tracked_vec = []
        info_list = pickle.load(
            open(
                os.path.join(
                    dset_dir,
                    dset,
                    'info_match.p'),
                'rb'))
        for key, frame in info_list.items():
            vec_list = frame['vehicles']
            for vec in vec_list:
                vec_cls = vec_detail_class[vec['class']]
                vec_id = vec['arrID']

                if vec_cls not in subc_bbox_num.keys():
                    subc_bbox_num[vec_cls] = 1
                else:
                    subc_bbox_num[vec_cls] += 1

                if vec_id not in tracked_vec:
                    if vec_cls not in subc_track_num.keys():
                        subc_track_num[vec_cls] = 1
                    else:
                        subc_track_num[vec_cls] += 1
                    tracked_vec.append(vec_id)
        del dset
        print(subc_bbox_num, subc_track_num)


def count_peds(dset_dir):
    ped_bbox_num = 0
    ped_inst_num = 0

    dset_list = os.listdir(dset_dir)
    dset_list = [dset for dset in dset_list if 'urban' in dset]
    for dset in tqdm(dset_list):
        tracked_vec = []
        info_list = pickle.load(
            open(
                os.path.join(
                    dset_dir,
                    dset,
                    'info_match.p'),
                'rb'))
        for key, frame in info_list.items():
            vec_list = frame['peds']
            for vec in vec_list:
                ped_bbox_num += 1
                vec_id = vec['arrID']
                if vec_id not in tracked_vec:
                    ped_inst_num += 1
                    tracked_vec.append(vec_id)
        del dset
        print(ped_bbox_num, ped_inst_num)


def get_coverage(dset_dir):
    dset_list = os.listdir(dset_dir)
    loc = []
    dset_list = [dset for dset in dset_list if 'urban' in dset]
    for dset in tqdm(dset_list):
        info_dict = pickle.load(
            open(
                os.path.join(
                    dset_dir,
                    dset,
                    'info_match.p'),
                'rb'))
        ego_loc_list = [np.array(info['location'])
                        for _, info in info_dict.items()]
        loc.append(ego_loc_list)
    with open('loc_list.p', 'wb') as f:
        pickle.dump(loc, f, protocol=2)