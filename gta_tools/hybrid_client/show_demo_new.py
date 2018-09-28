#!python2
"""
Add bounding boxes and control signals to videos
"""
import os
import pickle
import argparse

import cv2
import numpy as np

from utils.utils import *
from utils.data_utils import find_time_match_info
import seaborn as sns
from os.path import join
from glob import glob

def get_random_color():
    color = [int(np.random.rand()*255) for _ in range(3)]
    return tuple(color)

if __name__ == '__main__':

    # read info list
    dir_name = "G:\\comp\\rec_05162335_extrasunny_12h51m_x-412y-1459tox-2749y1385\\needrender"
    out_dir = "G:\\a"
    output_demo = True
    write_count = 1
    if not os.path.exists(os.path.join(dir_name, 'info_match.p')):
        find_time_match_info(dir_name)
    info_dict_list = pickle.load(open(os.path.join(dir_name, 'info_match.p'), 'rb'))
    add_info_panel = True
    add_vehicle_bbox = True
    add_ped_bbox = False
    add_vehicle_cube = True
    add_vehicle_info = False
    add_vehicle_head = False  # deprecated
    add_ped_cube = True
    add_collide_info = False
    add_vec_bbox_from_id_map = False
    colormap = {}
    ins_count = 0
    # correspond to classID key.
    vec_class = {0: "car", 1: "bike", 2: "bicycle", 3: "quadbike", 4: "boat", 5: "plane", 6: "helicopter", 7: "train", 8: "submersible", 9: "unknown"}
    plot_list1 = []
    plot_list2 = []
    vertex_idx = [[0, 1], [0, 3], [1, 2], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7], [0, 6], [1, 7], [2, 4], [3, 5], [2, 7], [1, 4]]

    timeseries = sorted(info_dict_list)
    if output_demo:
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        out = cv2.VideoWriter(dir_name + '.mp4',
                                  fourcc, 15.0, (1920, 1080))
    for ts in info_dict_list:
        info = info_dict_list[ts]
        frame = cv2.imread(os.path.join(dir_name, str(ts) + '_final.png'))
        if frame is None:
            continue
        idmap = generate_id_map(os.path.join(dir_name, str(ts) + '_id.png'))

        visibleset =set(np.unique(idmap))
        # time.sleep(0.2)
        ori_frame = frame.copy()
        if add_info_panel:
            # add info to visualize
            panel_coord = [20, 20, 280, 205]
            cv2.rectangle(frame, (panel_coord[0], panel_coord[1]), (panel_coord[2], panel_coord[3]), (0, 0, 0), -1)
            left_x1 = 60  # for title
            left_x2 = 230  # for value
            top_y = 70
            pad_y = 40
            n_digit = 2
            font_size = 1.0
            font = cv2.FONT_HERSHEY_DUPLEX
            speed = info['speed']
            # cv2.putText(frame, 'speed: ', (left_x1, top_y), font,
            #            font_size, (255, 255, 255), 1, cv2.LINE_AA)
            # cv2.putText(frame, '%05.02f' % (speed), (left_x2, top_y),
            #            font, font_size, (255, 255, 255), 1, cv2.LINE_AA)
            top_y = top_y + pad_y
            throttle = info['throttle']
            # cv2.putText(frame, 'throttle: ', (left_x1, top_y), font,
            #            font_size, (255, 255, 255), 1, cv2.LINE_AA)
            # cv2.putText(
            #    frame, '%05.02f' %
            #           (throttle), (left_x2, top_y), font, font_size,
            #    (255, 255, 255),
            #    1, cv2.LINE_AA)
            top_y = top_y + pad_y
            brake = info['brake']
            # cv2.putText(frame, 'brake: ', (left_x1, top_y), font,
            #            font_size, (255, 255, 255), 1, cv2.LINE_AA)
            # cv2.putText(frame, '%05.02f' % (brake), (left_x2, top_y),
            #            font, font_size, (255, 255, 255), 1, cv2.LINE_AA)
            top_y = top_y + pad_y
            steering = info['steering']
            if steering == 0:
                str_st = '00.00'
            elif steering > 0:
                str_st = '+' + '%04.02f' % (steering)
            else:
                str_st = '-' + '%04.02f' % (np.abs(steering))
            # cv2.putText(frame, 'steering: ', (left_x1, top_y), font,
            #            font_size, (255, 255, 255), 1, cv2.LINE_AA)
            # cv2.putText(frame, str_st, (left_x2, top_y), font,
            #            font_size, (255, 255, 255), 1, cv2.LINE_AA)
            top_y = top_y + pad_y
            # draw the four direction key visualization
            init_gap = 0
            block_gap = 75
            block_size = 60
            top_y = 45
            total_width = block_gap * 2 + block_size
            left_bound = int(
                (panel_coord[2] -
                 panel_coord[0] -
                 total_width) /
                2 +
                panel_coord[0])
            if throttle > 0:
                cv2.rectangle(frame, (left_bound + block_gap, top_y), (left_bound + block_gap + block_size, top_y + block_size), (255, 255, 255),
                              -1)  # up
            else:
                cv2.rectangle(frame, (left_bound + block_gap, top_y), (left_bound + block_gap + block_size, top_y + block_size), (30, 30, 30),
                              -1)  # up
            if brake > 0:
                cv2.rectangle(frame, (left_bound + block_gap, top_y + block_gap),
                              (left_bound + block_gap + block_size, top_y + block_gap + block_size), (255, 255, 255), -1)  # down
            else:
                cv2.rectangle(frame, (left_bound + block_gap, top_y + block_gap),
                              (left_bound + block_gap + block_size, top_y + block_gap + block_size), (30, 30, 30), -1)  # down
            if steering < 0:
                cv2.rectangle(frame, (left_bound, top_y + block_gap), (left_bound + block_size, top_y + block_gap + block_size), (255, 255, 255),
                              -1)  # left
            else:
                cv2.rectangle(frame, (left_bound, top_y + block_gap), (left_bound + block_size, top_y + block_gap + block_size), (30, 30, 30),
                              -1)  # left
            if steering > 0:
                cv2.rectangle(frame, (left_bound + 2 * block_gap, top_y + block_gap),
                              (left_bound + block_gap * 2 + block_size, top_y + block_gap + block_size), (255, 255, 255), -1)  # right
            else:
                cv2.rectangle(frame, (left_bound + 2 * block_gap, top_y + block_gap),
                              (left_bound + block_gap * 2 + block_size, top_y + block_gap + block_size), (30, 30, 30), -1)  # right
            opacity = 0.8
            cv2.addWeighted(frame, opacity, ori_frame, 1 - opacity, 0, frame)
        if add_vehicle_cube:
            v_list = info['vehicles']
            cam_coords = np.array(info['cam_coords'], 'double')
            cam_rotation = np.array(info['cam_rotation'], 'double')
            cam_near_clip = np.array(info['cam_near_clip'], 'double')
            cam_field_of_view = np.array(info['cam_field_of_view'], 'double')
            if add_vehicle_bbox:
                info_vec_bbox_list = []
            timeloc = timeseries.index(ts)
            succloc = []
            for succtime in range(timeloc, min(len(timeseries),timeloc + 30)):
                if succtime % 2 == 0:
                    succtime = timeseries[succtime]
                    succloc.append(info_dict_list[succtime]['location'])
            for (idx, loc) in enumerate(succloc):
                beforeplane = is_before_clip_plane(loc, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                loc_p = get_2d_from_3d(loc, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                ori_frame = frame.copy()
                if beforeplane:
                    cv2.circle(frame, (int(loc_p[0] * 1920), int(loc_p[1] * 1080)), int(-1 * idx + 20), (255, 255, 255), -1)
                    opacity = 0.4
                    cv2.addWeighted(frame, opacity, ori_frame, 1 - opacity, 0, frame)
            if len(v_list) > 0:
                for j in range(len(v_list)):
                    v_info = v_list[j]
                    visible = v_info['arrID'] in visibleset
                    dist = get_depth(v_info['your_pos'], cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                    if not visible:
                        continue
                    if not v_info['classID'] == 0:
                        continue
                    if v_info['arrID'] in colormap:
                        line_color = colormap[v_info['arrID']]
                    else:
                        color_ins = get_random_color()
                        line_color = color_ins
                        ins_count += 1
                        colormap[v_info['arrID']] = line_color
                    xxx = v_info['xxx']
                    yyy = v_info['yyy']
                    zzz = v_info['zzz']
                    line_width = 4
                    dd_x = []
                    dd_y = []
                    timeloc = timeseries.index(ts)
                    prevloc = []
                    x_list, y_list, _ = np.where(idmap==v_info['arrID'])
                    mask = frame.copy()
                    mask[x_list, y_list,:] = line_color[:3]
                    cv2.addWeighted(mask, 0.5, ori_frame, 0.5, 0, frame)
                    for prevtime in range(timeloc - 30, timeloc):
                        if prevtime % 2 == 0:
                            prevtime = timeseries[prevtime]
                            prevvlist = info_dict_list[prevtime]['vehicles']
                            for prevv_info in prevvlist:
                                if prevv_info['arrID'] == v_info['arrID']:
                                    prevloc.append(prevv_info['your_pos'])
                    for (idx, loc) in enumerate(prevloc):
                        beforeplane = is_before_clip_plane(loc, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                        loc_p = get_2d_from_3d(loc, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                        ori_frame = frame.copy()
                        #if beforeplane:
                            #cv2.circle(frame, (int(loc_p[0] * 1920), int(loc_p[1] * 1080)), int(0.5 * idx + 10), line_color, -1)
                            #opacity = 0.6
                            #cv2.addWeighted(frame, opacity, ori_frame, 1 - opacity, 0, frame)
                    for i in range(len(vertex_idx)):
                        idx = vertex_idx[i]
                        idx1 = idx[0]
                        idx2 = idx[1]
                        p1 = np.array([xxx[idx1], yyy[idx1], zzz[idx1]], 'double')
                        p2 = np.array([xxx[idx2], yyy[idx2], zzz[idx2]], 'double')
                        before1 = is_before_clip_plane(p1, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                        before2 = is_before_clip_plane(p2, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                        if not (before1 or before2):
                            continue
                        if before1 and before2:
                            cp1 = get_2d_from_3d(p1, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                            cp2 = get_2d_from_3d(p2, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                            x1 = int(cp1[0] * 1920)
                            x2 = int(cp2[0] * 1920)
                            y1 = int(cp1[1] * 1080)
                            y2 = int(cp2[1] * 1080)
                            if line_width > 0:
                                cv2.line(frame, (x1, y1), (x2, y2), line_color, line_width)
                            min_x, max_x, min_y, max_y = \
                                get_min_max_x_y_from_line(cp1[0], cp1[1], cp2[0], cp2[1])
                            if min_x is not None:
                                dd_x.append(min_x)
                                dd_x.append(max_x)
                                dd_y.append(min_y)
                                dd_y.append(max_y)
                            continue
                        center_pt, cam_dir = get_clip_center_and_dir(cam_coords, cam_rotation, cam_near_clip)
                        if before1 and not before2:
                            inter2 = get_intersect_point(center_pt, cam_dir, p1, p2)
                            cp1 = get_2d_from_3d(p1, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                            cp2 = get_2d_from_3d(inter2, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                            x1 = int(cp1[0] * 1920)
                            x2 = int(cp2[0] * 1920)
                            y1 = int(cp1[1] * 1080)
                            y2 = int(cp2[1] * 1080)
                            if line_width > 0:
                                cv2.line(frame, (x1, y1), (x2, y2), line_color, line_width)
                            min_x, max_x, min_y, max_y = get_min_max_x_y_from_line(cp1[0], cp1[1], cp2[0], cp2[1])
                            if min_x is not None:
                                dd_x.append(min_x)
                                dd_x.append(max_x)
                                dd_y.append(min_y)
                                dd_y.append(max_y)
                            continue
                        if before2 and not before1:
                            inter1 = get_intersect_point(center_pt, cam_dir, p1, p2)
                            cp2 = get_2d_from_3d(p2, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                            cp1 = get_2d_from_3d(inter1, cam_coords, cam_rotation, cam_near_clip, cam_field_of_view)
                            # print p1, p2, inter1, (inter1 -
                            # center_pt).dot(cam_dir), cp2, cp1
                            x1 = int(cp1[0] * 1920)
                            x2 = int(cp2[0] * 1920)
                            y1 = int(cp1[1] * 1080)
                            y2 = int(cp2[1] * 1080)
                            if line_width > 0:
                                cv2.line(frame, (x1, y1), (x2, y2), line_color, line_width)
                            min_x, max_x, min_y, max_y = get_min_max_x_y_from_line(cp1[0], cp1[1], cp2[0], cp2[1])
                            if min_x is not None:
                                dd_x.append(min_x)
                                dd_x.append(max_x)
                                dd_y.append(min_y)
                                dd_y.append(max_y)
                            continue
            if output_demo:
                out.write(frame)
            cv2.imshow('frame', frame)
            cv2.imwrite(os.path.join(dir_name,str(ts)+'render.png'),frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    # time.sleep(0.1)
