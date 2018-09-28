import math
import os
import pickle
import random
import shutil
import time

import cv2
import numpy as np
from tqdm import tqdm

from utils.utils import *
from utils.data_utils import find_time_match_info
IMWIDTH=640
IMHEIGHT=352
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

valid_vec_class = [0, 1, 2, 3, 7]

# the order of the vertices: (suppose the vehicle is moving forward)
# the upper 4 vertices: 2 4
#                       3 5
# the lower 4 vertices: 1 7
#                       0 6
vertex_idx = [[0, 1],  # l
              [0, 3],  # h
              [1, 2],
              [2, 3],
              [4, 5],
              [4, 7],
              [5, 6],
              [6, 7],
              [0, 6],
              [1, 7],
              [2, 4],
              [3, 5]]

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


##################################
# OUTER FUNCTION OF DATA PROCESS #
##################################

join3_test_set = [
    'urban_11071815_rain_8h38m_x2137y4373',
    'urban_11071845_thunder_13h27m_x5152y2672',
    'urban_11071916_clear_7h50m_x2640y4174',
    'urban_11071946_clear_6h28m_x2820y2611',
    'urban_11072016_clear_6h17m_x4264y2889',
    'urban_11072047_clouds_6h1m_x3603y2984',
    'urban_11072117_clear_13h10m_x4265y3475',
    'urban_11072148_clouds_9h15m_x4605y3684',
    'urban_11072218_clouds_14h42m_x5275y1988',
    'urban_11072248_clearing_7h40m_x4336y2903',
    'urban_11072319_clouds_11h28m_x4265y4224',
    'urban_11072349_clouds_7h44m_x3785y3117',
    'urban_11080019_clouds_16h46m_x2702y2715',
    'urban_11080050_thunder_12h46m_x4796y3522',
    'urban_11102155_clear_7h1m_x5308y3732',
    'urban_11102225_rain_7h25m_x4292y2918',
    'urban_11102256_clear_17h12m_x4915y3404',
    'urban_11102329_extrasunny_12h59m_x4173y2919',
    'urban_11102359_clouds_15h51m_x5005y2024',
    'urban_11110030_extrasunny_7h32m_x3984y3800',
    'urban_11110100_clear_10h49m_x2545y2647']


def single_copy(z):
    s, t = z
    shutil.copy2(s, t)


def copy_folder(z):
    s, t = z
    print(s)
    os.makedirs(t)
    filelists = os.listdir(s)
    for i in filelists:
        shutil.copy2(os.path.join(s, i), os.path.join(t, i))


def parse_data_multi_process(dset, dset_dir='D:/raw_ims'):
    this_dset = []
    dset_path = os.path.join(dset_dir, dset)
    if not os.path.exists(os.path.join(dset_path, 'info.gz')):
        print("This dataset is not copied yet!")
        return
    if os.path.exists(os.path.join(dset_path, 'full_parsed_info.p')):
        print("This dataset has already been processed!")
        return
    print('==> Processing ', dset)
    # build the info dict
    if not os.path.exists(os.path.join(dset_path, 'info_match.p')):
        find_time_match_info(dset_path)
    info_dict_list = pickle.load(
        open(
            os.path.join(
                dset_path,
                'info_match.p'),
            'rb'))
    ts_list = sorted(info_dict_list.keys())

    for ts in tqdm(ts_list):
        info = info_dict_list[ts]
        info['im_path'] = os.path.join(
            dset, str(ts) + '_final.png')  # im path with father dir

        info = build_vec_gt_from_map(
            info, os.path.join(
                dset_path, str(ts) + '_id.png'))
        if info is None:
            continue
        info = parse_frame_data(info)
        this_dset.append(info)
    pickle.dump(
        this_dset,
        open(
            os.path.join(
                dset_path,
                'full_parsed_info.p'),
            'wb'))


def sub_job(info_set):

    info, ts, dset, dset_path = info_set
    # print('parseing', ts)
    info['im_path'] = os.path.join(
        dset, str(ts) + '_final.png')  # im path with father dir

    info = build_vec_gt_from_map(
        info, os.path.join(
            dset_path, str(ts) + '_id.png'))
    if info is None:
        assert False, ts
        return [ts, None]
    info = parse_frame_data(info)
    return [ts, info]


def parse_data_sub_multi_process(dset, dset_dir):
    this_dset = []
    dset_path = os.path.join(dset_dir, dset)
    print(dset_path)
    if not os.path.exists(os.path.join(dset_path, 'info.gz')):
        print("This dataset is not copied yet!")
        return
    if os.path.exists(os.path.join(dset_path, 'full_parsed_info.p')):
        print("This dataset has already been processed!")
        return
    print('==> Processing ', dset)
    # build the info dict
    if not os.path.exists(os.path.join(dset_path, 'info_match.p')):
        find_time_match_info(dset_path)
    info_dict_list = pickle.load(
        open(
            os.path.join(
                dset_path,
                'info_match.p'),
            'rb'))
    ts_list = sorted(info_dict_list.keys())

    tmp_dict = {}

    ts_list = [
        ts for ts in ts_list if os.path.exists(
            os.path.join(
                dset_path,
                str(ts) +
                '_id.png'))]  # some images are deleted
    param_list = [(info_dict_list[ts], ts, dset, dset_path)
                  for ii, ts in enumerate(ts_list)]
    from multiprocessing import Pool
    p = Pool(8)
    results = p.map(sub_job, param_list)
    p.close()
    p.join()

    for t in results:
        ts, data = t
        tmp_dict[ts] = data
    for ts in ts_list:
        if not tmp_dict[ts] is None:
            this_dset.append(tmp_dict[ts])
    pickle.dump(
        this_dset,
        open(
            os.path.join(
                dset_path,
                'full_parsed_info.p'),
            'wb'))


def parse_data_and_split(
        dset_dir,
        test_ratio=0.2,
        output_dir='C:/Users/bair/Desktop',
        remove_truncated=False,
        n_dup=5):
    train_info = []
    test_info = []
    dset_list = os.listdir(dset_dir)
    for dset in tqdm(dset_list):
        print('==> Current len: ', len(train_info) + len(test_info))
        this_dset = []
# 1. only get the easy data (here we only filter out the weather)
        if not is_easy_data(dset):
            continue
        dset_path = os.path.join(dset_dir, dset)
        if os.path.exists(os.path.join(dset_path, 'filtered_info.p')):
            print("This dataset has already been processed!")
            this_dset = pickle.load(
                open(
                    os.path.join(
                        dset_path,
                        'filtered_info.p'),
                    'rb'))
            # merge the whole dataset
            valid_len = len(this_dset)
            split_point = int(valid_len * (1 - test_ratio))
            train_info = train_info + this_dset[:split_point]
            test_info = test_info + this_dset[split_point:]
            continue
        print('==> Processing ', dset)
        # build the info dict
        if not os.path.exists(os.path.join(dset_path, 'info_match.p')):
            find_time_match_info(dset_path)
        info_dict_list = pickle.load(
            open(
                os.path.join(
                    dset_path,
                    'info_match.p'),
                'rb'))
        ts_list = sorted(info_dict_list.keys())
        for ts in tqdm(ts_list):
            info = info_dict_list[ts]
            info['im_path'] = os.path.join(
                dset, str(ts) + '_final.png')  # im path with father dir

# 2. only get the day time data
            # frame_hour = info['time']
            # if frame_hour < 7 or frame_hour > 17:
            #     continue
            # build vehicle data
            info = build_vec_gt_from_map(
                info, os.path.join(
                    dset_path, str(ts) + '_id.p'))
            if info is None:
                continue

            info = parse_frame_data(info)

# (3). remove truncated vehicles
            if remove_truncated:
                info = remove_truncated(info)
# 3. only keep the frame with near distance
            if not is_in_valid_distance(info, 60):
                continue
            this_dset.append(info)
        pickle.dump(
            this_dset,
            open(
                os.path.join(
                    dset_path,
                    'parsed_info.p'),
                'wb'))
# 4. delete the duplicated vehicle
        dirty_flag = np.array([True] * len(this_dset))
        vec_frame = dict()
        # build trackingID: frameIDs dict
        for i, info in enumerate(this_dset):
            for vec in info['vehicle_data']:
                track_id = vec['tracking_id']
                # if vec['kitti']['occluded'] or vec['kitti']['truncated']:
                #     continue
                if track_id not in vec_frame.keys():
                    vec_frame[track_id] = [i]
                else:
                    vec_frame[track_id] = vec_frame[track_id] + [i]
        # filter according to tracking id
        max_len = 0
        for track_id, frame_ids in vec_frame.items():
            if len(frame_ids) > max_len:
                max_len = len(frame_ids)
            still_alive_ids = [i for i in frame_ids if dirty_flag[i]]
            if len(still_alive_ids) > n_dup:
                invalid_id = random.sample(
                    still_alive_ids, len(still_alive_ids) - n_dup)
                dirty_flag[invalid_id] = False
        print('ori len: ', len(dirty_flag), ' clean len: ', len(
            np.where(dirty_flag)[0]), ' max appearance: ', max_len)

        # merge to the large list and split train/test
        this_dset = [info for i, info in enumerate(this_dset) if dirty_flag[i]]
        # save this dset
        pickle.dump(
            this_dset,
            open(
                os.path.join(
                    dset_path,
                    'filtered_info.p'),
                'wb'))

        # merge the whole dataset
        valid_len = len(this_dset)
        split_point = int(valid_len * (1 - test_ratio))
        train_info = train_info + this_dset[:split_point]
        test_info = test_info + this_dset[split_point:]

    print('train total valid: ', len(train_info))
    print('test total valid: ', len(test_info))

    selected_train_ind = random.sample(range(len(train_info)), 10000).sort()
    selected_test_ind = random.sample(range(len(test_info)), 1000).sort()

    train_info = [info for i, info in enumerate(
        train_info) if i in selected_train_ind]
    test_info = [info for i, info in enumerate(
        test_info) if i in selected_test_ind]

    # train_info = random.sample(train_info, 10000)
    # test_info = random.sample(test_info, 2000)

    output_name = os.path.join(output_dir, 'merge_new.p')
    pickle.dump((train_info, test_info), open(output_name, 'wb'))


def filter_for_detection_and_split(dset_dir):
    is_restrict_easy_data = False
    is_restrict_hour = False
    is_remove_truncated = False
    is_remove_duplicated = False
    is_remove_still = True
    thres_n = 50
    thres_diff = 0.5
    depth_thres = 100
    n_dup = 5
    pixel_thres = 16

    first_sample_rate = 0.25

    output_dir = dset_dir
    train_info = []
    test_info = []
    dset_list = os.listdir(dset_dir)
    dset_list = sorted([dset for dset in dset_list if 'urban' in dset])
    split_point = int(len(dset_list) * 0.86)
    train_dset = dset_list[:split_point]
    test_dest = dset_list[split_point:]
    print(test_dest)
    print('==> Processing train set...')
    for dset in tqdm(train_dset):
        print('==> Current len: ', len(train_info) + len(test_info))
# 1. only get the easy data (here we only filter out the weather)
        if is_restrict_easy_data:
            if not is_easy_data(dset):
                continue
        dset_path = os.path.join(dset_dir, dset)
        print('==> Processing ', dset)
        this_dset = []
        info_list = pickle.load(
            open(
                os.path.join(
                    dset_path,
                    'full_parsed_info.p'),
                'rb'))
        for i_frame, info in enumerate(info_list):
            if i_frame < 5:
                continue
            info['frame_id'] = i_frame


# 2. only get the day time data
            if is_restrict_hour:
                frame_hour = info['time']
                if frame_hour < 7 or frame_hour > 17:
                    continue
# (3). remove truncated vehicles
            if is_remove_truncated:
                info = remove_truncated(info)
# 3. only keep the frame with near distance
            if not contain_valid_vehicle(info, depth_thres, pixel_thres):
                continue
            this_dset.append(info)
# 4. delete the duplicated vehicle
        if is_remove_duplicated:
            dirty_flag = np.array([True] * len(this_dset))
            vec_frame = dict()
            # build trackingID: frameIDs dict
            for i, info in enumerate(this_dset):
                for vec in info['vehicle_data']:
                    track_id = vec['tracking_id']
                    # if vec['kitti']['occluded'] or vec['kitti']['truncated']:
                    #     continue
                    if track_id not in vec_frame.keys():
                        vec_frame[track_id] = [i]
                    else:
                        vec_frame[track_id] = vec_frame[track_id] + [i]
            # filter according to tracking id
            max_len = 0
            for track_id, frame_ids in vec_frame.items():
                if len(frame_ids) > max_len:
                    max_len = len(frame_ids)
                still_alive_ids = [i for i in frame_ids if dirty_flag[i]]
                if len(still_alive_ids) > n_dup:
                    invalid_id = random.sample(
                        still_alive_ids, len(still_alive_ids) - n_dup)
                    dirty_flag[invalid_id] = False
            print('ori len: ', len(dirty_flag), ' clean len: ', len(
                np.where(dirty_flag)[0]), ' max appearance: ', max_len)

            # merge to the large list and split train/test
            this_dset = [info for i, info in enumerate(
                this_dset) if dirty_flag[i]]
            # save this dset
            # pickle.dump(this_dset, open(os.path.join(dset_path,
            # 'dup5_info.p'), 'wb'))
        if is_remove_still:
            old_len = len(this_dset)
            ego_loc_list = [np.array(info['ego_state']['location'])
                            for info in this_dset]
            loc_diff_list = []
            for i in range(len(ego_loc_list)):
                if i == 0:
                    loc_diff_list.append(100.)
                else:
                    last_loc = ego_loc_list[i - 1]
                    this_loc = ego_loc_list[i]
                    loc_diff_list.append(np.linalg.norm(this_loc - last_loc))

            valid_list = [True] * len(loc_diff_list)
            valid_list = np.array(valid_list)
            count_still = []
            for i in range(len(loc_diff_list)):
                count_this = 0
                j = i
                while j < len(loc_diff_list):
                    if loc_diff_list[j] < thres_diff:
                        count_this = count_this + 1
                        j = j + 1
                    else:
                        break
                count_still.append(count_this)
            for i in range(len(count_still)):
                if count_still[i] > thres_n:
                    valid_list[i:i + count_still[i]] = False
            this_dset = [info for i, info in enumerate(
                this_dset) if valid_list[i]]
            print('After removing still {}/{}'.format(len(this_dset), old_len))

        ori_len = len(this_dset)
        new_len = int(ori_len * first_sample_rate)
        valid_index = np.linspace(0, ori_len, new_len).astype('int')
        this_dset = [dset for dd, dset in enumerate(
            this_dset) if dd in valid_index]

        # save dataset
        train_info = train_info + this_dset

    for dset in tqdm(test_dest):
        print('==> Current len: ', len(train_info) + len(test_info))
# 1. only get the easy data (here we only filter out the weather)
        if is_restrict_easy_data:
            if not is_easy_data(dset):
                continue
        dset_path = os.path.join(dset_dir, dset)
        print('==> Processing ', dset)
        this_dset = []
        info_list = pickle.load(
            open(
                os.path.join(
                    dset_path,
                    'full_parsed_info.p'),
                'rb'))
        for i_frame, info in enumerate(info_list):
            if i_frame < 5:
                continue
            info['frame_id'] = i_frame


# 2. only get the day time data
            if is_restrict_hour:
                frame_hour = info['time']
                if frame_hour < 7 or frame_hour > 17:
                    continue
# (3). remove truncated vehicles
            if is_remove_truncated:
                info = remove_truncated(info)
# 3. only keep the frame with near distance
            if not contain_valid_vehicle(info, depth_thres, pixel_thres):
                continue
            this_dset.append(info)
# 4. delete the duplicated vehicle
        if is_remove_duplicated:
            dirty_flag = np.array([True] * len(this_dset))
            vec_frame = dict()
            # build trackingID: frameIDs dict
            for i, info in enumerate(this_dset):
                for vec in info['vehicle_data']:
                    track_id = vec['tracking_id']
                    # if vec['kitti']['occluded'] or vec['kitti']['truncated']:
                    #     continue
                    if track_id not in vec_frame.keys():
                        vec_frame[track_id] = [i]
                    else:
                        vec_frame[track_id] = vec_frame[track_id] + [i]
            # filter according to tracking id
            max_len = 0
            for track_id, frame_ids in vec_frame.items():
                if len(frame_ids) > max_len:
                    max_len = len(frame_ids)
                still_alive_ids = [i for i in frame_ids if dirty_flag[i]]
                if len(still_alive_ids) > n_dup:
                    invalid_id = random.sample(
                        still_alive_ids, len(still_alive_ids) - n_dup)
                    dirty_flag[invalid_id] = False
            print('ori len: ', len(dirty_flag), ' clean len: ', len(
                np.where(dirty_flag)[0]), ' max appearance: ', max_len)

            # merge to the large list and split train/test
            this_dset = [info for i, info in enumerate(
                this_dset) if dirty_flag[i]]
            # save this dset
            # pickle.dump(this_dset, open(os.path.join(dset_path,
            # 'dup5_info.p'), 'wb'))

        ori_len = len(this_dset)
        new_len = int(ori_len * first_sample_rate)
        valid_index = np.linspace(0, ori_len, new_len).astype('int')
        this_dset = [dset for dd, dset in enumerate(
            this_dset) if dd in valid_index]

        # save dataset
        test_info = test_info + this_dset

    print('train total valid: ', len(train_info))
    print('test total valid: ', len(test_info))

    selected_train_ind = random.sample(range(len(train_info)), 40000)
    selected_test_ind = random.sample(range(len(test_info)), 5000)
    selected_train_ind.sort()
    selected_test_ind.sort()
    train_info = [info for i, info in enumerate(
        train_info) if i in selected_train_ind]
    test_info = [info for i, info in enumerate(
        test_info) if i in selected_test_ind]

    # train_info = random.sample(train_info, 20000)
    # test_info = random.sample(test_info, 1000)

    output_name = os.path.join(output_dir,
                               'merge_linspace_byvid_nonstop_join3.p')
    pickle.dump((train_info, test_info), open(output_name, 'wb'), protocol=2)


def filter_for_tracking_and_split(dset_dir):
    is_restrict_easy_data = False
    is_restrict_hour = False
    is_remove_truncated = False
    is_remove_duplicated = False
    is_remove_still = True
    thres_n = 50
    thres_diff = 0.5
    depth_thres = 100
    n_dup = 5
    pixel_thres = 16
    max_skip = 2
    n_shortest = 20
    clip_len = 100

    output_dir = dset_dir
    train_info = []
    test_info = []
    dset_list = os.listdir(dset_dir)
    dset_list = sorted([dset for dset in dset_list if 'urban' in dset])
    # split_point = int(len(dset_list) * 0.8)
    # train_dset = dset_list[:split_point]
    # test_dest = dset_list[split_point:]
    train_dset = [dset for dset in dset_list if dset not in join3_test_set]
    test_dest = [dset for dset in dset_list if dset in join3_test_set]

    print('==> Processing train set...')
    for dset in tqdm(train_dset):
        print('==> Current len: ', len(train_info) + len(test_info))
# 1. only get the easy data (here we only filter out the weather)
        if is_restrict_easy_data:
            if not is_easy_data(dset):
                continue
        dset_path = os.path.join(dset_dir, dset)
        print('==> Processing ', dset)
        this_dset = []
        info_list = pickle.load(
            open(
                os.path.join(
                    dset_path,
                    'full_parsed_info.p'),
                'rb'))
        for i_frame, info in enumerate(info_list):
            if i_frame < 5:
                continue
            info['frame_id'] = i_frame

# 2. only get the day time data
            if is_restrict_hour:
                frame_hour = info['time']
                if frame_hour < 7 or frame_hour > 17:
                    continue
# (3). remove truncated vehicles
            if is_remove_truncated:
                info = remove_truncated(info)
# 3. only keep the frame with near distance
            if not contain_valid_vehicle(info, depth_thres, pixel_thres):
                continue
            this_dset.append(info)
# 4. delete the duplicated vehicle
        if is_remove_duplicated:
            dirty_flag = np.array([True] * len(this_dset))
            vec_frame = dict()
            # build trackingID: frameIDs dict
            for i, info in enumerate(this_dset):
                for vec in info['vehicle_data']:
                    track_id = vec['tracking_id']
                    # if vec['kitti']['occluded'] or vec['kitti']['truncated']:
                    #     continue
                    if track_id not in vec_frame.keys():
                        vec_frame[track_id] = [i]
                    else:
                        vec_frame[track_id] = vec_frame[track_id] + [i]
            # filter according to tracking id
            max_len = 0
            for track_id, frame_ids in vec_frame.items():
                if len(frame_ids) > max_len:
                    max_len = len(frame_ids)
                still_alive_ids = [i for i in frame_ids if dirty_flag[i]]
                if len(still_alive_ids) > n_dup:
                    invalid_id = random.sample(
                        still_alive_ids, len(still_alive_ids) - n_dup)
                    dirty_flag[invalid_id] = False
            print('ori len: ', len(dirty_flag), ' clean len: ', len(
                np.where(dirty_flag)[0]), ' max appearance: ', max_len)

            # merge to the large list and split train/test
            this_dset = [info for i, info in enumerate(
                this_dset) if dirty_flag[i]]
            # save this dset
            # pickle.dump(this_dset, open(os.path.join(dset_path,
            # 'dup5_info.p'), 'wb'))
        if is_remove_still:
            old_len = len(this_dset)
            ego_loc_list = [np.array(info['ego_state']['location'])
                            for info in this_dset]
            loc_diff_list = []
            for i in range(len(ego_loc_list)):
                if i == 0:
                    loc_diff_list.append(100.)
                else:
                    last_loc = ego_loc_list[i - 1]
                    this_loc = ego_loc_list[i]
                    loc_diff_list.append(np.linalg.norm(this_loc - last_loc))

            valid_list = [True] * len(loc_diff_list)
            valid_list = np.array(valid_list)
            count_still = []
            for i in range(len(loc_diff_list)):
                count_this = 0
                j = i
                while j < len(loc_diff_list):
                    if loc_diff_list[j] < thres_diff:
                        count_this = count_this + 1
                        j = j + 1
                    else:
                        break
                count_still.append(count_this)
            for i in range(len(count_still)):
                if count_still[i] > thres_n:
                    valid_list[i:i + count_still[i]] = False
            this_dset = [info for i, info in enumerate(
                this_dset) if valid_list[i]]
            print('After removing still {}/{}'.format(len(this_dset), old_len))

        redundant_len = len(this_dset) - int(len(this_dset) *
                                             1. / clip_len) * clip_len
        remove_head = int(redundant_len / 2.)
        remove_tail = redundant_len - remove_head
        ooo_len = len(this_dset)
        this_dset = [dset for ii, dset in enumerate(
            this_dset) if ii >= remove_head and ii < ooo_len - remove_tail]
        assert len(this_dset) % 100 == 0
        for ii in range(int(len(this_dset) / 100)):
            train_info.append(this_dset[ii * 100: (ii + 1) * 100])
        # now build frame ids for sequence sampling
        # frame_ids = [d['frame_id'] for d in this_dset]
        # piece = []
        # for f in frame_ids:
        #     if len(piece) == 0:  # piece empty
        #         piece.append(this_dset[frame_ids.index(f)])
        #     else:  # already something in the piece
        #         assert f > piece[-1]['frame_id']
        #         if f <= piece[-1]['frame_id'] + 1 + max_skip:
        #             piece.append(this_dset[frame_ids.index(f)])
        #         else:  # break the piece
        #             if len(piece) > n_shortest:
        #                 print('getting piece length: ', len(piece))
        #                 train_info.append(piece)
        #             piece = []

    for dset in tqdm(test_dest):
        print('==> Current len: ', len(train_info) + len(test_info))
# 1. only get the easy data (here we only filter out the weather)
        if is_restrict_easy_data:
            if not is_easy_data(dset):
                continue
        dset_path = os.path.join(dset_dir, dset)
        print('==> Processing ', dset)
        this_dset = []
        info_list = pickle.load(
            open(
                os.path.join(
                    dset_path,
                    'full_parsed_info.p'),
                'rb'))
        for i_frame, info in enumerate(info_list):
            if i_frame < 5:
                continue
            info['frame_id'] = i_frame


# 2. only get the day time data
            if is_restrict_hour:
                frame_hour = info['time']
                if frame_hour < 7 or frame_hour > 17:
                    continue
# (3). remove truncated vehicles
            if is_remove_truncated:
                info = remove_truncated(info)
# 3. only keep the frame with near distance
            if not contain_valid_vehicle(info, depth_thres, pixel_thres):
                continue
            this_dset.append(info)
# 4. delete the duplicated vehicle
        if is_remove_duplicated:
            dirty_flag = np.array([True] * len(this_dset))
            vec_frame = dict()
            # build trackingID: frameIDs dict
            for i, info in enumerate(this_dset):
                for vec in info['vehicle_data']:
                    track_id = vec['tracking_id']
                    # if vec['kitti']['occluded'] or vec['kitti']['truncated']:
                    #     continue
                    if track_id not in vec_frame.keys():
                        vec_frame[track_id] = [i]
                    else:
                        vec_frame[track_id] = vec_frame[track_id] + [i]
            # filter according to tracking id
            max_len = 0
            for track_id, frame_ids in vec_frame.items():
                if len(frame_ids) > max_len:
                    max_len = len(frame_ids)
                still_alive_ids = [i for i in frame_ids if dirty_flag[i]]
                if len(still_alive_ids) > n_dup:
                    invalid_id = random.sample(
                        still_alive_ids, len(still_alive_ids) - n_dup)
                    dirty_flag[invalid_id] = False
            print('ori len: ', len(dirty_flag), ' clean len: ', len(
                np.where(dirty_flag)[0]), ' max appearance: ', max_len)

            # merge to the large list and split train/test
            this_dset = [info for i, info in enumerate(
                this_dset) if dirty_flag[i]]
            print('After removing still {}/{}'.format(len(this_dset), old_len))
            # save this dset
            # pickle.dump(this_dset, open(os.path.join(dset_path,
            # 'dup5_info.p'), 'wb'))

        redundant_len = len(this_dset) - int(len(this_dset) *
                                             1. / clip_len) * clip_len
        remove_head = int(redundant_len / 2.)
        remove_tail = redundant_len - remove_head
        ooo_len = len(this_dset)
        this_dset = [dset for ii, dset in enumerate(
            this_dset) if ii >= remove_head and ii < ooo_len - remove_tail]
        assert len(this_dset) % 100 == 0
        for ii in range(int(len(this_dset) / 100)):
            test_info.append(this_dset[ii * 100: (ii + 1) * 100])
        # now build frame ids for sequence sampling
        # frame_ids = [d['frame_id'] for d in this_dset]
        # piece = []
        # for f in frame_ids:
        #     if len(piece) == 0:  # piece empty
        #         piece.append(this_dset[frame_ids.index(f)])
        #     else:  # already something in the piece
        #         assert f > piece[-1]['frame_id']
        #         if f <= piece[-1]['frame_id'] + 1 + max_skip:
        #             piece.append(this_dset[frame_ids.index(f)])
        #         else:  # break the piece
        #             if len(piece) > n_shortest:
        #                 print('getting piece length: ', len(piece))
        #                 test_info.append(piece)
        #             piece = []

    print('train total sequence: ', len(train_info),
          ' total frames: ', np.sum([len(s) for s in train_info]))
    print('test total sequence: ', len(test_info),
          ' total frames: ', np.sum([len(s) for s in test_info]))

    output_name = os.path.join(output_dir, 'merge_nonstop_seq_join3_clip100.p')
    pickle.dump((train_info, test_info), open(output_name, 'wb'), protocol=2)


def fill_flicker_holes(info_list):
    visible_dict = dict()
    for i_f, info in enumerate(info_list):
        for vec in info['vehicle_data']:
            tracking_id = vec['tracking_id']
            if tracking_id in visible_dict.keys():
                visible_dict[tracking_id].append(i_f)
            else:
                visible_dict[tracking_id] = [i_f]

    smooth_bbox = {}

    for tracking_id, frames in visible_dict.items():
        frames.sort()
        v_new = []
        for fid in frames:
            v_new.append(fid)
            vs = info_list[fid]['vehicle_data']
            for v in vs:
                if v['tracking_id'] == tracking_id:
                    b1 = np.array(v['kitti']['bbox'])
                    break

            if fid + 2 in frames and fid + 1 not in frames:
                vs = info_list[fid + 2]['vehicle_data']
                for v in vs:
                    if v['tracking_id'] == tracking_id:
                        b2 = np.array(v['kitti']['bbox'])
                        break
                smooth_bbox[(fid + 1, tracking_id)] = b1 + (b2 - b1) * 1 / 2.

                v_new.append(fid + 1)
            if fid + 3 in frames and fid + 1 not in frames \
                    and fid + 2 not in frames:
                vs = info_list[fid + 3]['vehicle_data']
                for v in vs:
                    if v['tracking_id'] == tracking_id:
                        b2 = np.array(v['kitti']['bbox'])
                        break
                smooth_bbox[(fid + 1, tracking_id)] = b1 + (b2 - b1) * 1 / 3.
                smooth_bbox[(fid + 2, tracking_id)] = b1 + (b2 - b1) * 2 / 3.

                v_new.append(fid + 1)
                v_new.append(fid + 2)
            if fid + 4 in frames and fid + 1 not in frames and fid + \
                    2 not in frames and fid + 3 not in frames:
                vs = info_list[fid + 4]['vehicle_data']
                for v in vs:
                    if v['tracking_id'] == tracking_id:
                        b2 = np.array(v['kitti']['bbox'])
                        break
                smooth_bbox[(fid + 1, tracking_id)] = b1 + (b2 - b1) * 1 / 4.
                smooth_bbox[(fid + 2, tracking_id)] = b1 + (b2 - b1) * 2 / 4.
                smooth_bbox[(fid + 3, tracking_id)] = b1 + (b2 - b1) * 3 / 4.

                v_new.append(fid + 1)
                v_new.append(fid + 2)
                v_new.append(fid + 3)

        # print(len(v), len(v_new))
        visible_dict[tracking_id] = v_new

    n_fill = 0
    for i_f, info in enumerate(info_list):
        new_track_id = [key for key, v in visible_dict.items() if i_f in v]
        sanity_bits = [False] * len(new_track_id)
        for vec in info['vehicle_data']:
            tracking_id = vec['tracking_id']
            assert tracking_id in new_track_id
            ind = new_track_id.index(tracking_id)
            sanity_bits[ind] = True

        for i_s, s in enumerate(sanity_bits):
            if not s:
                track_id_to_add = new_track_id[i_s]
                # now will add
                prev_info = info_list[i_f - 1]
                for vv in prev_info['vehicle_data']:
                    if vv['tracking_id'] == track_id_to_add:
                        prev_v = vv
                        # break
                prev_v['kitti']['bbox'] = list(
                    smooth_bbox[(i_f, track_id_to_add)].astype('int'))
                # TODO: currently just copy the prev vehicle
                info_list[i_f]['vehicle_data'].append(prev_v)
                n_fill = n_fill + 1
    print('=======> ', n_fill)
    return info_list


def show_parsed_ims(dataset_dir):
    font_size = 1.0
    font = cv2.FONT_HERSHEY_DUPLEX

    train_info_list = pickle.load(
        open(
            os.path.join(
                dataset_dir,
                'full_parsed_info.p'),
            'rb'))
    train_info_list = fill_flicker_holes(train_info_list)
    print('info loaded')

    for info in train_info_list:
        im_path = info['im_path']
        if '\\' in im_path:
            im_path = im_path.split('\\')[-1]
        frame = cv2.imread(os.path.join(dataset_dir, im_path))
        # print(os.path.join(dataset_dir, im_path))
        vec_info = info['vehicle_data']
        # [vec for vec in vec_info if vec['visible']]
        visible_vec_info = vec_info
        # print(len(vec_info))

        # near_clip = info['cam_param']['near_clip']
        # print info['ego_state']['location'], info['cam_param']['coord']
        cam_near_clip = info['cam_param']['near_clip']
        cam_field_of_view = info['cam_param']['field_of_view']

        for vec in visible_vec_info:
            kitti_data = vec['kitti']
            depth = kitti_data['location'][2]
            if depth > 100:
                continue
            x1, y1, x2, y2 = kitti_data['bbox']
            color = (
                0, 0, 255) if (
                kitti_data['truncated'] or kitti_data['occluded']) else (
                0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # rot_y = kitti_data['rotation_y']
            # alpha = rot_y2alpha_gta(rot_y, (x1+x2)/2., cam_near_clip,
            # cam_field_of_view)
            # cv2.putText(frame,str(round(kitti_data['alpha']/np.pi, 2)),
            # (x1,y1), font, font_size,(255,255,255),1,cv2.LINE_AA)
            # cv2.putText(frame,str(round(kitti_data['location'][0], 2)) +
            # ' ' + str(round(kitti_data['location'][2], 2)),(x1,y1), font,
            # font_size,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(
                frame, str(
                    kitti_data['type']), (x1, y1), font, 0.5, (255, 255, 255),
                1, cv2.LINE_AA)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


##################################
# PARSE DATA AND ADD ALPHA & ROT #
##################################

def parse_frame_data(info):
    frame_data = {}  # a dict to store frame data

    # im path
    frame_data['im_path'] = info['im_path']

    # time
    frame_data['time'] = info['time']

    # ego state
    ego_state = {}
    ego_state['speed'] = info['speed']
    ego_state['throttle'] = info['throttle']
    ego_state['brake'] = info['brake']
    ego_state['steering'] = info['steering']
    ego_state['collision'] = info['collision']
    ego_state['yaw_rate'] = info['yawRate']
    ego_state['location'] = info['location']
    frame_data['ego_state'] = ego_state

    # camera intrinsics
    cam_param = {}
    cam_coords = np.array(info['cam_coords'], 'double')
    cam_rotation = np.array(info['cam_rotation'], 'double')
    cam_near_clip = np.array(info['cam_near_clip'], 'double')
    cam_field_of_view = np.array(info['cam_field_of_view'], 'double')
    cam_info = [cam_coords, cam_rotation, cam_near_clip, cam_field_of_view]
    cam_param['field_of_view'] = cam_field_of_view
    cam_param['rotation'] = cam_rotation
    cam_param['coord'] = cam_coords
    cam_param['near_clip'] = cam_near_clip
    frame_data['cam_param'] = cam_param

    # vehicle data (same format as kitti)
    vehicle_list = []
    v_list = info['vehicles']
    if len(v_list) > 0:
        for v_obj in v_list:
            if v_obj['classID'] in valid_vec_class:
                dd = get_obj_struct(v_obj, cam_info, type='vehicle')
                if dd is not None:
                    vehicle_list.append(dd)
    frame_data['vehicle_data'] = vehicle_list

    # ped data (same format as vehicle)
    ped_list = []
    v_list = info['peds']
    if len(v_list) > 0:
        for v_obj in v_list:
            dd = get_obj_struct(v_obj, cam_info, type='ped')
            if dd is not None:
                ped_list.append(dd)
    frame_data['ped_data'] = ped_list

    return frame_data


def get_obj_struct(v_info, cam_info, type='vehicle'):
    obj_struct = {}
    cam_coords, cam_rotation, cam_near_clip, cam_field_of_view = cam_info

    # tracking id
    obj_struct['tracking_id'] = v_info['arrID']

    # class id
    obj_struct['class_id'] = v_info['classID']

    # heading
    obj_struct['heading'] = v_info['heading']

    # speed
    obj_struct['speed'] = v_info['speed']

    # location
    obj_struct['location'] = v_info['your_pos']

    # visibility
    # obj_struct['visible'] = v_info['visible']  # TODO: now we only save the
    # visible data

    if type == 'vehicle':
        obj_struct['hash'] = v_info['hash']
        obj_struct['class'] = v_info['class']

    # 3d coords and corrected
    obj_struct['xxx'] = v_info['xxx']
    obj_struct['yyy'] = v_info['yyy']
    obj_struct['zzz'] = v_info['zzz']

    # 3d center
    xxx = v_info['xxx']
    yyy = v_info['yyy']
    zzz = v_info['zzz']
    center_3d = [
        (xxx[2] + xxx[6]) / 2.,
        (yyy[2] + yyy[6]) / 2.,
        (zzz[2] + zzz[6]) / 2.]
    obj_struct['center_3d'] = center_3d

    if type == 'vehicle':  # TODO: now only support vehicle
        # 2d bbox
        obj_struct['bbox'] = v_info['bbox']
        obj_struct['bbox_loose'] = v_info['bbox_loose']

        # center 2d
        x1, y1, x2, y2 = obj_struct['bbox']
        obj_struct['center_2d'] = [(x1 + x2) / 2., (y1 + y2) / 2.]

    # kitti format data
    if type == 'vehicle':
        kitti_format = {}

        kitti_format['type'] = vec_detail_class[v_info['class']]
        kitti_format['truncated'] = v_info['truncated']
        kitti_format['occluded'] = v_info['occluded']

        kitti_format['bbox'] = obj_struct['bbox']

        # dimension
        dim_vertix_idx = [[0, 3],  # h
                          [2, 4],  # w
                          [0, 1]]  # l
        dim_list = []
        for idx_pair in dim_vertix_idx:
            idx1, idx2 = idx_pair
            p1 = np.array([xxx[idx1], yyy[idx1], zzz[idx1]])
            p2 = np.array([xxx[idx2], yyy[idx2], zzz[idx2]])
            dim_list.append(np.linalg.norm(p1 - p2))
        kitti_format['dimensions'] = dim_list

        # cam-coord location
        kitti_cam_coord = get_kitti_format_camera_coords(
            np.array(center_3d), cam_coords, cam_rotation, cam_near_clip)
        kitti_format['location'] = kitti_cam_coord

        # alpha & rot-y
        rot = get_rotation_y(cam_info, xxx, yyy, zzz)
        kitti_format['rotation_y'] = rot

        kitti_format['alpha'] = rot_y2alpha_gta(
            rot, (x1 + x2) / 2., cam_near_clip, cam_field_of_view)

        obj_struct['kitti'] = kitti_format

    return obj_struct


def get_rotation_y(cam_info, xxx, yyy, zzz):
    cam_coords, cam_rotation, cam_near_clip, cam_field_of_view = cam_info
    cam_dir, cam_up, cam_east = get_cam_dir_vecs(cam_rotation)
    p_forward = np.array([xxx[1], yyy[1], zzz[1]], 'double')
    p_backward = np.array([xxx[0], yyy[0], zzz[0]], 'double')
    incam_forward_vec1 = get_kitti_format_camera_coords(
        p_forward, cam_coords, cam_rotation, cam_near_clip)
    incam_forward_vec2 = get_kitti_format_camera_coords(
        p_backward, cam_coords, cam_rotation, cam_near_clip)
    incam_forward_vec = incam_forward_vec1 - incam_forward_vec2
    cos_x = vec_cos(incam_forward_vec[[0, 2]], np.array([1, 0]))
    cos_z = vec_cos(incam_forward_vec[[0, 2]], np.array([0, 1]))

    rot = np.arccos(cos_x)  # 0 - pi
    if cos_z > 0:
        rot = - rot
    return rot


def rot_y2alpha_gta(rot_y, x, cam_near_clip, cam_field_of_view):
    # x in pixel coord
    # rot_y in (-pi, pi)
    x = x * 1. / IMWIDTH  # x is a value in (0, 1)
    x = x - 0.5
    plane_x = screen_x_to_view_plane(x, cam_near_clip, cam_field_of_view)

    theta = math.atan(plane_x / cam_near_clip)

    # theta_ray = np.pi / 2 - theta
    # alpha = rot_y + np.pi - theta_ray + 0.5 * np.pi
    alpha = rot_y + np.pi - theta

    alpha = alpha % (2 * np.pi)
    alpha = alpha - np.pi

    return alpha


def get_legal_content(forename, thres_n=50, thres_diff=1.):
    info_path = os.path.join(forename, 'info.p')
    info_list = pickle.load(open(info_path, 'rb'))
    print('Total number: ', len(info_list))

    # build the loc diff list
    ego_loc_list = [np.array(info['ego_state']['location'])
                    for info in info_list]
    loc_diff_list = []
    for i in range(len(ego_loc_list)):
        if i == 0:
            loc_diff_list.append(100.)
        else:
            last_loc = ego_loc_list[i - 1]
            this_loc = ego_loc_list[i]
            loc_diff_list.append(np.linalg.norm(this_loc - last_loc))

    valid_list = [True] * len(loc_diff_list)
    valid_list = np.array(valid_list)

    # 1. Remove the too long still period
    count_still = []
    for i in range(len(loc_diff_list)):
        count_this = 0
        j = i
        while j < len(loc_diff_list):
            if loc_diff_list[j] < thres_diff:
                count_this = count_this + 1
                j = j + 1
            else:
                break
        count_still.append(count_this)
    for i in range(len(count_still)):
        if count_still[i] > thres_n:
            valid_list[i:i + count_still[i]] = False

    # 2. Remove the image without vehicles
    for i, info in enumerate(info_list):
        flag = True
        for vec in info['vehicle_data']:
            if vec['visible']:
                flag = False
                break
        if flag:
            valid_list[i] = False

    # get the valid index return
    valid_info = []
    for i in range(len(info_list)):
        if valid_list[i]:
            valid_info.append(info_list[i])

    return valid_info


def extract_valid_det_dset(
        dset,
        ori_dir,
        output_dir,
        thres_n=50,
        thres_diff=1.):
    dset_list = os.listdir(ori_dir)
    # for dset in dset_list:
    if os.path.exists(os.path.join(output_dir + dset, 'info.p')):
        print('*' * 10 + ' File already processed!')
        return
    pickle_path = ori_dir + dset
    print(pickle_path)
    if os.path.exists(pickle_path):
        t1 = time.time()
        legal_info = get_legal_content(ori_dir + dset, thres_n, thres_diff)
        print('Valid number: ', len(legal_info))
        print('Begin copying...')
        if os.path.exists(output_dir + dset):
            print('WARNING! Output folder already exists.')
        else:
            os.makedirs(output_dir + dset)
        for info in tqdm(legal_info):
            im_path = info['im_path']
            shutil.copy2(
                os.path.join(
                    ori_dir + dset,
                    im_path),
                os.path.join(
                    output_dir + dset,
                    im_path))
        pickle.dump(
            legal_info,
            open(
                os.path.join(
                    output_dir +
                    dset,
                    'info.p'),
                'wb'))
        t2 = time.time()
        print('Copying done, total time: ', t2 - t1)
        print('*' * 30)


def remove_truncated(info):
    vec_list = info['vehicle_data']
    new_vec_list = []
    for i, vec in enumerate(vec_list):
        x1, y1, x2, y2 = vec['kitti']['bbox']
        if 0 in [x1, x2] or IMWIDTH in [x1, x2]:
            continue
        new_vec_list.append(vec)
    info['vehicle_data'] = new_vec_list
    return info


def build_vec_gt_from_map(info, map_path):
    id_map = cv2.imread(map_path, -1)
    if id_map is None:
        print('****************************', map_path)

    v_list = info['vehicles']
    # get the camera parameters
    cam_coords = np.array(info['cam_coords'], 'double')
    cam_rotation = np.array(info['cam_rotation'], 'double')
    cam_near_clip = np.array(info['cam_near_clip'], 'double')
    cam_field_of_view = np.array(info['cam_field_of_view'], 'double')

    new_v_list = []

    if len(v_list) == 0:
        info['vehicles'] = []
        print('getting no vehicles in {}'.format(map_path))
        return info

    # firstly get the new v list with valid vec list and extra annotation
    if 'bbox_loose' in v_list[0].keys():
        return info

    # now get the projected loose bbox for every vehicle
    if len(v_list) > 0:
        for j in range(len(v_list)):
            v_info = v_list[j]
            if not v_info['classID'] == 0:  # not vehicle
                continue

            xxx = v_info['xxx']
            yyy = v_info['yyy']
            zzz = v_info['zzz']

            dd_x, dd_y = get_projected_2d_vertices(
                xxx, yyy, zzz, cam_coords, cam_rotation, cam_near_clip,
                cam_field_of_view)

            if len(dd_x) >= 2 and len(dd_y) >= 2:
                x1 = int(min(dd_x) * IMWIDTH)
                x2 = int(max(dd_x) * IMWIDTH)
                y1 = int(min(dd_y) * IMHEIGHT)
                y2 = int(max(dd_y) * IMHEIGHT)
                x1 = max(0, x1)
                x2 = max(0, x2)
                y1 = max(0, y1)
                y2 = max(0, y2)
                x1 = min(IMWIDTH, x1)
                x2 = min(IMWIDTH, x2)
                y1 = min(IMHEIGHT, y1)
                y2 = min(IMHEIGHT, y2)

                assert 'bbox_loose' not in v_info.keys(), v_list
                v_info['bbox_loose'] = [x1, y1, x2, y2]
                new_v_list.append(v_info)

    # now further filter the bbox according to the
    final_v_list = []  # final list to store the selected v_info
    id_vec_bbox_list = get_vec_bbox_list_from_map(id_map)
    # construct iou mat
    if len(id_vec_bbox_list) == 0 or len(new_v_list) == 0:
        info['vehicles'] = []
        print('getting no vehicles in {}'.format(map_path))
        return info
    iou_mat = np.zeros([len(id_vec_bbox_list), len(new_v_list)])
    for i1, bbox in enumerate(
            id_vec_bbox_list):  # only part of id_vec_bbox_list will
                                # be regarded as valid
        for i2, v_info in enumerate(new_v_list):
            bbox2 = v_info['bbox_loose']
            if bbox2[0] <= bbox[0] and bbox2[1] <= bbox[1] \
                    and bbox2[2] >= bbox[2] \
                    and bbox2[3] >= bbox[3] \
                    or compute_iou(bbox, bbox2) > 0.7:
                iou_mat[i1, i2] = compute_iou(bbox, bbox2)

    while np.amax(iou_mat) > 0.3:
        max_ind = np.argmax(iou_mat)
        max_i = int(max_ind / len(new_v_list))
        max_j = max_ind % len(new_v_list)

        x1, y1, x2, y2 = id_vec_bbox_list[max_i]
        is_truncated = False
        is_occluded = False
        if x1 == 0 or x2 == IMWIDTH or y1 == 0 or y2 == IMHEIGHT:
            is_truncated = True
        if np.amax(iou_mat) < 0.5:
            is_occluded = True
        v_info = new_v_list[max_j]
        assert 'bbox' not in v_info.keys()
        v_info['bbox'] = [x1, y1, x2, y2]
        assert 'occluded' not in v_info.keys()
        assert 'truncated' not in v_info.keys()
        v_info['occluded'] = is_occluded
        v_info['truncated'] = is_truncated
        final_v_list.append(v_info)

        iou_mat[max_i, :] = 0.
        iou_mat[:, max_j] = 0.
    info['vehicles'] = final_v_list
    return info


def get_projected_2d_vertices(
        xxx,
        yyy,
        zzz,
        cam_coords,
        cam_rotation,
        cam_near_clip,
        cam_field_of_view):
    dd_x = []
    dd_y = []

    for i in range(len(vertex_idx)):
        idx = vertex_idx[i]
        idx1 = idx[0]
        idx2 = idx[1]
        p1 = np.array([xxx[idx1], yyy[idx1], zzz[idx1]], 'double')
        p2 = np.array([xxx[idx2], yyy[idx2], zzz[idx2]], 'double')
        before1 = is_before_clip_plane(
            p1,
            cam_coords,
            cam_rotation,
            cam_near_clip,
            cam_field_of_view)
        before2 = is_before_clip_plane(
            p2,
            cam_coords,
            cam_rotation,
            cam_near_clip,
            cam_field_of_view)

        if not (before1 or before2):
            continue
        if before1 and before2:
            cp1 = get_2d_from_3d(
                p1,
                cam_coords,
                cam_rotation,
                cam_near_clip,
                cam_field_of_view)
            cp2 = get_2d_from_3d(
                p2,
                cam_coords,
                cam_rotation,
                cam_near_clip,
                cam_field_of_view)
            x1 = int(cp1[0] * IMWIDTH)
            x2 = int(cp2[0] * IMWIDTH)
            y1 = int(cp1[1] * IMHEIGHT)
            y2 = int(cp2[1] * IMHEIGHT)

            min_x, max_x, min_y, max_y = get_min_max_x_y_from_line(
                cp1[0], cp1[1], cp2[0], cp2[1])
            if min_x is not None:
                dd_x.append(min_x)
                dd_x.append(max_x)
                dd_y.append(min_y)
                dd_y.append(max_y)
            continue

        center_pt, cam_dir = get_clip_center_and_dir(
            cam_coords, cam_rotation, cam_near_clip)

        if before1 and not before2:
            inter2 = get_intersect_point(center_pt, cam_dir, p1, p2)
            cp1 = get_2d_from_3d(
                p1,
                cam_coords,
                cam_rotation,
                cam_near_clip,
                cam_field_of_view)
            cp2 = get_2d_from_3d(
                inter2,
                cam_coords,
                cam_rotation,
                cam_near_clip,
                cam_field_of_view)
            x1 = int(cp1[0] * IMWIDTH)
            x2 = int(cp2[0] * IMWIDTH)
            y1 = int(cp1[1] * IMHEIGHT)
            y2 = int(cp2[1] * IMHEIGHT)

            min_x, max_x, min_y, max_y = get_min_max_x_y_from_line(
                cp1[0], cp1[1], cp2[0], cp2[1])
            if min_x is not None:
                dd_x.append(min_x)
                dd_x.append(max_x)
                dd_y.append(min_y)
                dd_y.append(max_y)
            continue

        if before2 and not before1:
            inter1 = get_intersect_point(center_pt, cam_dir, p1, p2)
            cp2 = get_2d_from_3d(
                p2,
                cam_coords,
                cam_rotation,
                cam_near_clip,
                cam_field_of_view)
            cp1 = get_2d_from_3d(
                inter1,
                cam_coords,
                cam_rotation,
                cam_near_clip,
                cam_field_of_view)
            # print p1, p2, inter1, (inter1 - center_pt).dot(cam_dir), cp2, cp1
            x1 = int(cp1[0] * IMWIDTH)
            x2 = int(cp2[0] * IMWIDTH)
            y1 = int(cp1[1] * IMHEIGHT)
            y2 = int(cp2[1] * IMHEIGHT)

            min_x, max_x, min_y, max_y = get_min_max_x_y_from_line(
                cp1[0], cp1[1], cp2[0], cp2[1])
            if min_x is not None:
                dd_x.append(min_x)
                dd_x.append(max_x)
                dd_y.append(min_y)
                dd_y.append(max_y)
            continue
    return dd_x, dd_y


def get_vec_bbox_list_from_map(id_map):
    id_vec_bbox_list = []
    u_id = np.unique(id_map)
    ego_id = id_map[IMHEIGHT-1, IMWIDTH // 2]
    for uu in u_id:
        if uu == 0 or uu == ego_id or int(uu / 2e4) not in [0]:
            continue
        y_list, x_list = np.where(id_map == uu)
        bbox = [
            np.amin(x_list),
            np.amin(y_list),
            np.amax(x_list),
            np.amax(y_list)]  # x1, y1, x2, y2
        x1, y1, x2, y2 = bbox

        density = len(x_list) * 1. / ((x2 - x1 + 1) * (y2 - y1 + 1))
        if density < 0.3:  # density < 0.3 => too much occlusion
            continue
        id_vec_bbox_list.append([x1, y1, x2, y2])
    return id_vec_bbox_list


def is_easy_data(folder_name):
    easy_weather = ['clouds', 'clear', 'extrasunny']
    for weather in easy_weather:
        if weather in folder_name:
            return True
    return False


def is_in_valid_distance(info, distance_thres=100):
    vehicle_data = info['vehicle_data']
    for vec in vehicle_data:
        # print vec['visible'], vec['kitti']['location']
        if vec['kitti']['location'][2] < distance_thres \
                and vec['kitti']['location'][2] > 0:
            return True
    return False


def contain_valid_vehicle(info, distance_thres=100, pixel_thres=16):
    parser_dict = {
        'Compacts': 'Car',
        'Sedans': 'Car',
        'SUVs': 'Van',
        'Coupes': 'Car',
        'Muscle': 'Car',
        'Sports Classics': 'Car',
        'Sports': 'Car',
        'Super': 'Car',
        # 8: 'Motorcycles',
        'Industrial': 'Truck',
        'Utility': 'Truck',  # usally truck
        'Vans': 'Van',
        'Service': 'Car',  # usually taxi
        'Emergency': 'Car',  # usually police car
        'Commercial': 'Truck'
    }
    vehicle_data = info['vehicle_data']
    for vec in vehicle_data:
        location = vec['kitti']['location']
        bbox = vec['kitti']['bbox']
        if location[2] < distance_thres \
                and location[2] > 0 \
                and vec['kitti']['type'] in parser_dict.keys() \
                and bbox[2] - bbox[0] + 1 > pixel_thres / 0.65 \
                and bbox[3] - bbox[1] + 1 > pixel_thres / 0.65:
            return True
    return False


def get_test_video_set(dset_dir):
    target_len = 100
    vid_list = os.listdir(dset_dir)
    dset_list = os.listdir(dset_dir)
    dset_list = sorted([dset for dset in dset_list if 'urban' in dset])
    split_point = int(len(dset_list) * 0.8)
    test_vid_list = dset_list[split_point:]
    # test_vid_list = random.sample(test_vid_list, 10)
    test_set = []
    for i, dset in enumerate(test_vid_list):
        dset_path = os.path.join(dset_dir, dset)
        print('==> Processing ', dset)
        info_list = pickle.load(
            open(
                os.path.join(
                    dset_path,
                    'full_parsed_info.p'),
                'rb'))
        test_set = test_set + target_clip
        continue
        # info_list = fill_flicker_holes(info_list)
        dset_len = len(info_list)
        start_ind = np.random.randint(dset_len - target_len)
        target_clip = info_list[start_ind: start_ind + target_len]
        test_set = test_set + target_clip
    pickle.dump(
        test_set,
        open(
            os.path.join(
                dset_dir,
                'test_gta_vid_full.p'),
            'wb'),
        protocol=2)


def count_tracks(dset_dir):
    n_track = 0
    n_frame = 0
    n_vec = 0
    track_list = []
    dset_list = os.listdir(dset_dir)
    dset_list = [dset for dset in dset_list if 'urban' in dset]
    for dset in tqdm(dset_list):
        info_list = pickle.load(
            open(
                os.path.join(
                    dset_dir,
                    dset,
                    'full_parsed_info.p'),
                'rb'))
        n_frame += len(info_list)
        for frame in info_list:
            vec_list = frame['vehicle_data']
            n_vec += len(vec_list)
            for vec in vec_list:
                if vec['tracking_id'] not in track_list:
                    track_list.append(vec['tracking_id'])
        n_track = n_track + len(track_list)
        track_list = []
        print(n_track, n_frame, n_vec)


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


if __name__ == '__main__':
    count_subcategories('/data/lj/gta1')

