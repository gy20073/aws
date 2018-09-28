#!python2
"""
Another visualization
"""

import os
import pickle
import argparse

import cv2
import matplotlib.pyplot as plt
import numpy as np

from utils.utils import *
from utils.data_utils import find_time_match_info
import math

def get_rotation_y(cam_rotation, xxx, yyy, zzz):
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

def rot_y2alpha_gta(rot_y, x, cam_near_clip, cam_field_of_view, IM_WIDTH=1920):
    # x in pixel coord
    # rot_y in (-pi, pi)
    x = x * 1. / IM_WIDTH  # x is a value in (0, 1)
    x = x - 0.5
    plane_x = screen_x_to_view_plane(x, cam_near_clip, cam_field_of_view)

    theta = math.atan(plane_x / cam_near_clip)

    # theta_ray = np.pi / 2 - theta
    # alpha = rot_y + np.pi - theta_ray + 0.5 * np.pi
    alpha = rot_y + np.pi - theta

    alpha = alpha % (2 * np.pi)
    alpha = alpha - np.pi

    return alpha

def get_random_color():
    color = [np.random.rand() for _ in range(3)]
    return tuple(color)


def fig2data(fig):
    # ax = fig.gca().axis('off')
    fig.canvas.draw()

    w, h = fig.canvas.get_width_height()
    buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)

    buf.shape = (h, w, 4)
    # buf = buf[:, :, 1:]

    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to
    # have it in RGBA mode
    buf = np.roll(buf, 3, axis=2)
    buf = cv2.resize(buf[:, :, :3], (1920, 1080))
    return buf


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description=None)
    parser.add_argument(
        '-p',
        '--path',
        help='Directory to retrieve images',
        required=True)
    parser.add_argument(
        '-m',
        '--map_type',
        default='id',
        help='Types of map: id, depth')
    parser.add_argument('-o', '--output_demo', default=True)
    args = parser.parse_args()

    # read info list
    #dir_name = args.base_dir + args.folder_name
    dir_name = args.path
    map_type = 'id'

    if not os.path.exists(os.path.join(dir_name, 'info_match.p')):
        find_time_match_info(dir_name)
    info_dict_list = pickle.load(
        open(
            os.path.join(
                dir_name,
                'info_match.p'),
            'rb'))

    # correspond to classID key.
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
    plt.ion()
    arrid_color_dict = dict()

    # set output video
    if args.output_demo:
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        out = cv2.VideoWriter(dir_name + '_birdview.mp4',
                              fourcc, 15.0, (1920, 1080 * 2))

    ts_list = sorted(info_dict_list.keys())
    fig = plt.figure(figsize=(16, 9))
    for ts in ts_list:
        info = info_dict_list[ts]
        #temp = cv2.imread(os.path.join(dir_name, str(ts) + '_' + args.map_type + '.png'), -1).astype('uint8')
        id_map = generate_id_map(os.path.join(dir_name, str(ts) + '_' + args.map_type + '.png'))

        if True:
            v_list = info['vehicles']
            cam_coords = np.array(info['cam_coords'], 'double')
            cam_rotation = np.array(info['cam_rotation'], 'double')
            cam_near_clip = np.array(info['cam_near_clip'], 'double')
            cam_field_of_view = np.array(info['cam_field_of_view'], 'double')

            info_vec_bbox_list = []

            if len(v_list) > 0:
                for j in range(len(v_list)):
                    v_info = v_list[j]
                    if not v_info['classID'] == 0:
                        continue

                    xxx = v_info['xxx']
                    yyy = v_info['yyy']
                    zzz = v_info['zzz']

                    line_color = (0, 255, 0)
                    line_width = 2

                    dd_x = []
                    dd_y = []

                    for i in range(len(vertex_idx)):
                        idx = vertex_idx[i]
                        idx1 = idx[0]
                        idx2 = idx[1]
                        p1 = np.array(
                            [xxx[idx1], yyy[idx1], zzz[idx1]], 'double')
                        p2 = np.array(
                            [xxx[idx2], yyy[idx2], zzz[idx2]], 'double')
                        before1 = is_before_clip_plane(
                            p1, cam_coords, cam_rotation, cam_near_clip,
                            cam_field_of_view)
                        before2 = is_before_clip_plane(
                            p2, cam_coords, cam_rotation, cam_near_clip,
                            cam_field_of_view)

                        if not (before1 or before2):
                            continue
                        if before1 and before2:
                            cp1 = get_2d_from_3d(
                                p1, cam_coords, cam_rotation, cam_near_clip,
                                cam_field_of_view)
                            cp2 = get_2d_from_3d(
                                p2, cam_coords, cam_rotation, cam_near_clip,
                                cam_field_of_view)
                            x1 = int(cp1[0] * 1920)
                            x2 = int(cp2[0] * 1920)
                            y1 = int(cp1[1] * 1080)
                            y2 = int(cp2[1] * 1080)

                            min_x, max_x, min_y, max_y = \
                                get_min_max_x_y_from_line(
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
                            inter2 = get_intersect_point(
                                center_pt, cam_dir, p1, p2)
                            cp1 = get_2d_from_3d(
                                p1, cam_coords, cam_rotation, cam_near_clip,
                                cam_field_of_view)
                            cp2 = get_2d_from_3d(
                                inter2, cam_coords, cam_rotation,
                                cam_near_clip, cam_field_of_view)
                            # print p1, p2, inter2, (inter2 -
                            # center_pt).dot(cam_dir), cp1, cp2
                            x1 = int(cp1[0] * 1920)
                            x2 = int(cp2[0] * 1920)
                            y1 = int(cp1[1] * 1080)
                            y2 = int(cp2[1] * 1080)

                            min_x, max_x, min_y, max_y = \
                                get_min_max_x_y_from_line(
                                    cp1[0], cp1[1], cp2[0], cp2[1])
                            if min_x is not None:
                                dd_x.append(min_x)
                                dd_x.append(max_x)
                                dd_y.append(min_y)
                                dd_y.append(max_y)
                            continue

                        if before2 and not before1:
                            inter1 = get_intersect_point(
                                center_pt, cam_dir, p1, p2)
                            cp2 = get_2d_from_3d(
                                p2, cam_coords, cam_rotation, cam_near_clip,
                                cam_field_of_view)
                            cp1 = get_2d_from_3d(
                                inter1, cam_coords, cam_rotation,
                                cam_near_clip, cam_field_of_view)
                            # print p1, p2, inter1, (inter1 -
                            # center_pt).dot(cam_dir), cp2, cp1
                            x1 = int(cp1[0] * 1920)
                            x2 = int(cp2[0] * 1920)
                            y1 = int(cp1[1] * 1080)
                            y2 = int(cp2[1] * 1080)

                            min_x, max_x, min_y, max_y = \
                                get_min_max_x_y_from_line(
                                    cp1[0], cp1[1], cp2[0], cp2[1])
                            if min_x is not None:
                                dd_x.append(min_x)
                                dd_x.append(max_x)
                                dd_y.append(min_y)
                                dd_y.append(max_y)
                            continue

                    if len(dd_x) >= 2 and len(dd_y) >= 2:
                        x1 = int(min(dd_x) * 1920)
                        x2 = int(max(dd_x) * 1920)
                        y1 = int(min(dd_y) * 1080)
                        y2 = int(max(dd_y) * 1080)
                        dist = get_depth(
                            v_info['your_pos'],
                            cam_coords,
                            cam_rotation,
                            cam_near_clip,
                            cam_field_of_view)
                        if True:  # dist < 100:
                            x1 = max(0, x1)
                            x2 = max(0, x2)
                            y1 = max(0, y1)
                            y2 = max(0, y2)
                            x1 = min(1919, x1)
                            x2 = min(1919, x2)
                            y1 = min(1079, y1)
                            y2 = min(1079, y2)

                            info_dict = dict()
                            # bbox
                            info_dict['bbox'] = [x1, y1, x2, y2]
                            # get dimension
                            dim_vertix_idx = [[0, 3],  # h
                                              [2, 4],  # w
                                              [0, 1]]  # l
                            dim_list = []
                            for idx_pair in dim_vertix_idx:
                                idx1, idx2 = idx_pair
                                p1 = np.array(
                                    [xxx[idx1], yyy[idx1], zzz[idx1]])
                                p2 = np.array(
                                    [xxx[idx2], yyy[idx2], zzz[idx2]])
                                dim_list.append(np.linalg.norm(p1 - p2))
                            info_dict['dim'] = dim_list
                            # get translation
                            info_dict['tran'] = get_kitti_format_camera_coords(
                                v_info['your_pos'], cam_coords, cam_rotation,
                                cam_near_clip)

                            info_dict['arr_id'] = v_info['arrID']
                            info_dict['rotation_y'] = get_rotation_y(
                                cam_rotation, xxx, yyy, zzz)
                            info_vec_bbox_list.append(info_dict)

            if True:
                frame = cv2.imread(
                    os.path.join(
                        dir_name,
                        str(ts) +
                        '_final.png'))
                id_vec_bbox_list = []
                u_id = np.unique(id_map)
                ego_id = id_map[1079, 960]
                for uu in u_id:
                    if uu == ego_id:
                        continue
                    y_list, x_list,_ = np.where(id_map == uu)
                    bbox = [
                        np.amin(x_list),
                        np.amin(y_list),
                        np.amax(x_list),
                        np.amax(y_list)]  # x1, y1, x2, y2
                    x1, y1, x2, y2 = bbox

                    density = len(x_list) * 1. / \
                        ((x2 - x1 + 1) * (y2 - y1 + 1))
                    #if density < 0.3:  # density < 0.3 => too much occlusion
                    #    continue
                    # print('id box density: {}'.format(len(x_list) * 1./
                    # ((x2-x1+1) *(y2- y1+1))))

                    id_vec_bbox_list.append([x1, y1, x2, y2])
                for bbox in id_vec_bbox_list:
                    x1, y1, x2, y2 = bbox
                    id_occluded = False
                    if x1 == 0 or x2 == 1919 or y1 == 0 or y2 == 1079:
                        id_occluded = True

                    match_bbox_list = []
                    for info_dict in info_vec_bbox_list:
                        bbox2 = info_dict['bbox']
                        if bbox2[0] <= bbox[0] and bbox2[1] <= bbox[1] and \
                                bbox2[2] >= bbox[2] and \
                                bbox2[3] >= bbox[3] \
                                or compute_iou(bbox, bbox2) > 0.7:
                            match_bbox_list.append(info_dict)

                    if len(match_bbox_list) == 0:
                        continue
                    elif len(match_bbox_list) == 1:
                        bbox_info = match_bbox_list[0]
                        bbox_ratio = compute_iou(bbox, bbox_info['bbox'])
                        # NOTE: <0.3 not visible, <0.6 occluded
                        #if bbox_ratio < 0.3:
                        #    continue
                    else:
                        bbox_ratio_list = [
                            compute_iou(
                                bbox,
                                bbox2['bbox']) for bbox2 in match_bbox_list]
                        #if np.max(bbox_ratio_list) < 0.3:
                        #    continue
                        ind = np.argmax(bbox_ratio_list)
                        bbox_info = match_bbox_list[ind]

                    h, w, z = bbox_info['dim']
                    rot_y = bbox_info['rotation_y']
                    trans = bbox_info['tran']
                    vec_l = [z * np.cos(rot_y), -z * np.sin(rot_y)]
                    vec_w = [-w * np.cos(rot_y - np.pi / 2),
                             w * np.sin(rot_y - np.pi / 2)]
                    vec_l = np.array(vec_l)
                    vec_w = np.array(vec_w)
                    center = np.array([trans[0], trans[2]])

                    arrid = bbox_info['arr_id']
                    if arrid not in arrid_color_dict.keys():
                        arrid_color_dict[arrid] = get_random_color()

                    current_color = arrid_color_dict[arrid]

                    p1 = center + 0.5 * vec_l - 0.5 * vec_w
                    p2 = center + 0.5 * vec_l + 0.5 * vec_w
                    p3 = center - 0.5 * vec_l + 0.5 * vec_w
                    p4 = center - 0.5 * vec_l - 0.5 * vec_w

                    # ßßprint('===>')
                    alpha = rot_y2alpha_gta(rot_y, (x1+x2)/2., cam_near_clip, cam_field_of_view, IM_WIDTH=1920)
                    plt.text(p1[0], p1[1], str(alpha*180/np.pi))
                    #plt.text(p1[0], p1[1], str(rot_y*180/np.pi))
                    plt.plot([p1[0], p2[0]], [p1[1], p2[1]],
                             c=current_color, ls='-', linewidth=2 * line_width)
                    plt.plot([p1[0], p4[0]], [p1[1], p4[1]],
                             c=current_color, ls='-', linewidth=line_width)
                    plt.plot([p3[0], p2[0]], [p3[1], p2[1]],
                             c=current_color, ls='-', linewidth=line_width)
                    plt.plot([p3[0], p4[0]], [p3[1], p4[1]],
                             c=current_color, ls='-', linewidth=line_width)

                    current_color = list(current_color)
                    for i_c, c in enumerate(current_color):
                        current_color[i_c] = int(c * 255)
                    cv2.rectangle(
                        frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]),
                        tuple(current_color), 2)
                plt.axis('equal')
                # plt.set_xlim([-100, 100])
                # plt.set_ylim([100, 100])
                plt.axis([-100, 100, -5, 100])

                # plot the forward arrow
                plt.plot([0, 0], [0, 3], 'k-')
                plt.plot([-1, 0], [2, 3], 'k-')
                plt.plot([1, 0], [2, 3], 'k-')

        fig_data = fig2data(plt.gcf())
        # print(fig_data.max())
        fig_data = np.vstack((frame, fig_data))
        out.write(fig_data)
        cv2.imshow('frame', fig_data)
        plt.clf()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if args.output_demo:
        out.release()
        cv2.destroyAllWindows()
