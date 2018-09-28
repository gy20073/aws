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

IMWIDTH = 1920
IMHEIGHT = 1080
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

classid2string = {
    0: "car",
    1: "bike",
    2: "bicycle",
    3: "quadbike",
    4: "boat",
    5: "plane",
    6: "helicopter",
    7: "train",
    8: "submersible",
    9: "none",
    10: "pedestrians",
    11: "riders",
    12: "buildings",
    13: "fences",
    14: "poles",
    15: "roadlines",
    16: "roads",
    17: "sidewalks",
    18: "vegetation",
    19: "walls",
    20: "trafficsigns",
    21: "lane",
    255: "unknown"
}

valid_vec_class = [0, 1, 2, 3, 4, 5, 6, 7, 8]

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

def sub_job(info_set):
    info, ts, dset, dset_path = info_set
    # print('parseing', ts)
    try:
        info['im_path'] = os.path.join(dset, str(ts) + '_final.png')
        info = build_vec_gt_from_map(info, os.path.join(dset_path, str(ts) + '_id.png'))
        info = build_ped_gt_from_map(info, os.path.join(dset_path, str(ts) + '_id.png'))
    except:
        info = None
    if info is None:
        return {'timestamp':ts}
    info['timestamp'] = ts
    info = parse_frame_data(info)
    return info

def parse_data_sub_multi_process(dset, dset_dir):
    dset_path = os.path.join(dset_dir, dset)
    if os.path.exists(os.path.join(dset_path, 'track.json')):
        return
    print(dset_path)
    if not os.path.exists(os.path.join(dset_path, 'info.gz')):
        print("This dataset is not copied yet!")
        return
    print('==> Processing ', dset)
    # build the info dict
    if not os.path.exists(os.path.join(dset_path, 'info_match.p')):
        find_time_match_info(dset_path)
    info_dict_list = pickle.load(open(os.path.join(dset_path,'info_match.p'),'rb'))
    ts_list = sorted(info_dict_list.keys())

    ts_list = [ts for ts in ts_list if os.path.exists(os.path.join(dset_path,str(ts) +'_id.png'))]  # some images are deleted
    param_list = [(info_dict_list[ts], ts, dset, dset_path)for ii, ts in enumerate(ts_list)]
    from multiprocessing import Pool
    p = Pool(8)
    results = p.map(sub_job, param_list)
    p.close()
    p.join()
    results = list(sorted(results, key=lambda x:x['timestamp']))
    for idx, info in enumerate(results):
        info['frame_id'] = idx
        info['dset_name'] = dset
    with open(os.path.join(dset_path, 'track.json'), 'w') as fp:
        json.dump(results, fp, indent=2, sort_keys=True)

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

    obj_struct['n_pixel'] = v_info['n_pixel']
    # ignore
    obj_struct['ignore'] = v_info['ignore']  # TODO: now we only save the
    # visible data

    if type == 'vehicle':
        obj_struct['hash'] = v_info['hash']
        obj_struct['class'] = v_info['class']
        obj_struct['color'] = v_info['color']

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

    if type == 'vehicle' or type == 'ped':  # TODO: now only support vehicle
        # 2d bbox
        obj_struct['bbox'] = v_info['bbox']
        obj_struct['bbox_loose'] = v_info['bbox_loose']

        # center 2d
        x1, y1, x2, y2 = obj_struct['bbox']
        obj_struct['center_2d'] = [(x1 + x2) / 2., (y1 + y2) / 2.]

    # kitti format data
    if type == 'vehicle' or type == 'ped':
        kitti_format = {}
        if type == 'vehicle':
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
        kitti_format['location'] = kitti_cam_coord.tolist()

        # alpha & rot-y
        rot = get_rotation_y(cam_info, xxx, yyy, zzz)
        kitti_format['rotation_y'] = rot

        kitti_format['alpha'] = rot_y2alpha_gta(
            rot, (x1 + x2) / 2., cam_near_clip, cam_field_of_view)

        obj_struct['kitti'] = kitti_format

    return obj_struct

##################################
# PARSE DATA AND ADD ALPHA & ROT #
##################################

def parse_frame_data(info):
    frame_data = {}  # a dict to store frame data

    # time
    frame_data['timestamp'] = info['timestamp']

    # ego state
    frame_data['speed'] = info['speed']
    frame_data['control'] = {
        'throttle': info['throttle'],
        'brake': info['brake'],
        'steering': info['steering']}
    frame_data['ego_state'] = {'speed':info['speed'],
                               'control':{
                                   'throttle': info['throttle'],
                                   'brake': info['brake'],
                                   'steering': info['steering']}}
    # camera intrinsics
    frame_data['pose'] = {
        'position': info['cam_coords'],
        'rotation': info['cam_rotation']}
    frame_data['camera'] = {
        'nearClip': info['cam_near_clip'],
        'fov': info['cam_field_of_view']}


    cam_coords = np.array(info['cam_coords'], 'double')
    cam_rotation = np.array(info['cam_rotation'], 'double')
    cam_near_clip = np.array(info['cam_near_clip'], 'double')
    cam_field_of_view = np.array(info['cam_field_of_view'], 'double')
    cam_info = [cam_coords, cam_rotation, cam_near_clip, cam_field_of_view]

    # vehicle data (same format as kitti)
    vehicle_list = []
    v_list = info['vehicles']
    if len(v_list) > 0:
        for v_obj in v_list:
            if v_obj['classID'] in valid_vec_class:
                dd = get_obj_struct(v_obj, cam_info, type='vehicle')
                if dd is not None:
                    vehicle_list.append(dd)
    frame_data['object'] = vehicle_list

    ped_list = []
    v_list = info['peds']
    if len(v_list) > 0:
        for v_obj in v_list:
            dd = get_obj_struct(v_obj, cam_info, type='ped')
            if dd is not None:
                ped_list.append(dd)
    frame_data['ped_data'] = ped_list
    return frame_data

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
    id_map = generate_id_map(map_path)
    final_map = cv2.imread(map_path.replace('id.png','final.jpg'))
    visible_vehs = list(np.unique(id_map))
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
        #print('getting no vehicles in {}'.format(map_path))
        return info

    # firstly get the new v list with valid vec list and extra annotation
    if 'bbox_loose' in v_list[0].keys():
        print('might be error')
        return info

    # now get the projected loose bbox for every vehicle
    if len(v_list) > 0:
        for j in range(len(v_list)):
            v_info = v_list[j]
            if not v_info['classID'] == 0:  # not vehicle
                continue
            if not v_info['arrID'] in visible_vehs:
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
                x1 = min(IMWIDTH-1, x1)
                x2 = min(IMWIDTH-1, x2)
                y1 = min(IMHEIGHT-1, y1)
                y2 = min(IMHEIGHT-1, y2)

                assert 'bbox_loose' not in v_info.keys(), v_list
                v_info['bbox_loose'] = [x1, y1, x2, y2]
                new_v_list.append(v_info)

    id_vec_bbox_list = get_vec_bbox_list_from_map(id_map, final_map)

    if len(id_vec_bbox_list) == 0 or len(new_v_list) == 0:
        info['vehicles'] = []
        #print('getting no vehicles in {}'.format(map_path))
        return info

    for veh in new_v_list:
        vehid = veh['arrID']
        x1, y1, x2, y2, npix, color = id_vec_bbox_list[vehid]
        veh['bbox'] = [x1, y1, x2, y2]
        veh['n_pixel'] = npix
        veh['color'] = color
        is_truncated = False
        if x1 == 0 or x2 == IMWIDTH-1 or y1 == 0 or y2 == IMHEIGHT-1:
            is_truncated = True
        veh['truncated'] = is_truncated
        bboxloose = veh['bbox_loose']
        size =  (bboxloose[2] - bboxloose[0])*(bboxloose[3] - bboxloose[1])
        if size == 0:
            veh['occluded'] = True
            veh['ignore'] = True
        else:
            veh['occluded'] = float(veh['n_pixel'])/size < 0.2
            veh['ignore'] = check_ignore(veh['n_pixel'], 64)
    info['vehicles'] = new_v_list
    return info

def build_ped_gt_from_map(info, map_path):
    id_map = generate_id_map(map_path)
    final_map = cv2.imread(map_path.replace('id.png','final.jpg'))
    visible_peds = list(np.unique(id_map))
    if id_map is None:
        print('****************************', map_path)

    p_list = info['peds']
    # get the camera parameters
    cam_coords = np.array(info['cam_coords'], 'double')
    cam_rotation = np.array(info['cam_rotation'], 'double')
    cam_near_clip = np.array(info['cam_near_clip'], 'double')
    cam_field_of_view = np.array(info['cam_field_of_view'], 'double')

    new_p_list = []

    if len(p_list) == 0:
        info['peds'] = []
        #print('getting no peds in {}'.format(map_path))
        return info

    # firstly get the new v list with valid vec list and extra annotation
    if 'bbox_loose' in p_list[0].keys():
        print('might be error')
        return info

    # now get the projected loose bbox for every vehicle
    if len(p_list) > 0:
        for j in range(len(p_list)):
            p_info = p_list[j]
            if not p_info['classID'] == 10:  # not vehicle
                continue
            if not p_info['arrID'] in visible_peds:
                continue
            xxx = p_info['xxx']
            yyy = p_info['yyy']
            zzz = p_info['zzz']

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
                x1 = min(IMWIDTH-1, x1)
                x2 = min(IMWIDTH-1, x2)
                y1 = min(IMHEIGHT-1, y1)
                y2 = min(IMHEIGHT-1, y2)

                assert 'bbox_loose' not in p_info.keys(), p_list
                p_info['bbox_loose'] = [x1, y1, x2, y2]
                new_p_list.append(p_info)

    id_ped_bbox_list = get_vec_bbox_list_from_map(id_map, final_map)

    if len(id_ped_bbox_list) == 0 or len(new_p_list) == 0:
        info['peds'] = []
        #print('getting no peds in {}'.format(map_path))
        return info

    for ped in new_p_list:
        pedid = ped['arrID']
        x1, y1, x2, y2, npix, color = id_ped_bbox_list[pedid]
        ped['bbox'] = [x1, y1, x2, y2]
        ped['n_pixel'] = npix
        is_truncated = False
        if x1 == 0 or x2 == IMWIDTH-1 or y1 == 0 or y2 == IMHEIGHT-1:
            is_truncated = True
        ped['truncated'] = is_truncated
        bboxloose = ped['bbox_loose']
        size =  (bboxloose[2] - bboxloose[0])*(bboxloose[3] - bboxloose[1])
        if size == 0:
            ped['ignore'] = True
        else:
            ped['ignore'] = False
    info['peds'] = new_p_list
    return info

def check_ignore(num_pixel, tho):
    return False if num_pixel>tho else True

def get_projected_2d_vertices(
        xxx,
        yyy,
        zzz,
        cam_coords,
        cam_rotation,
        cam_near_clip,
        cam_field_of_view
):
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


def get_vec_bbox_list_from_map(id_map, final_map):
    id_vec_bbox_list = {}
    u_id = np.unique(id_map)
    id_map = id_map.squeeze()
    ego_id = id_map[IMHEIGHT-1, IMWIDTH // 2]
    for uu in u_id:
        if uu == 0 or uu == ego_id:
            continue
        y_list, x_list = np.where(id_map == uu)
        bbox = [
            np.amin(x_list),
            np.amin(y_list),
            np.amax(x_list),
            np.amax(y_list)]  # x1, y1, x2, y2
        x1, y1, x2, y2 = bbox
        npix = int((id_map == uu).sum())
        color = final_map[y_list,x_list,:].mean(axis=0).tolist()
        id_vec_bbox_list[uu]=[int(x1), int(y1), int(x2), int(y2), npix, color]
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





if __name__ == '__main__':
    count_subcategories('/data/lj/gta1')
