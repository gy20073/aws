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

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description=None)
    parser.add_argument(
        '-p',
        '--path',
        help='Directory to retrieve images',
        required=True)
    parser.add_argument('--far', default=100.0, type=float)
    parser.add_argument('--occ', default=0.2, type=float)
    parser.add_argument('--small', default=64, type=int)

    args = parser.parse_args()


    output_demo = True
    # read info list
    dir_name = args.path
    info_dict_list = json.load(
        open(
            os.path.join(
                dir_name,
                'track.json'),
            'rb')
    )

    add_info_panel = True
    add_vehicle_bbox = True
    add_ped_bbox = False
    add_vehicle_cube = True
    add_vehicle_info = False
    add_vehicle_head = False  # deprecated
    add_ped_cube = True
    add_collide_info = False
    add_vec_bbox_from_id_map = False

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

    plot_list1 = []
    plot_list2 = []
    vertex_idx = [[0, 1],
                  [0, 3],
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

    # set output video
    if output_demo:
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        out = cv2.VideoWriter(dir_name + '.mp4', fourcc, 15.0, (1920, 1080))

    for info in info_dict_list:
        ts = info['timestamp']
        frame = cv2.imread(os.path.join(dir_name, str(ts) + '_final.png'))
        id_map =generate_id_map(os.path.join(dir_name, str(ts) + '_id.png'))
        # time.sleep(0.2)
        ori_frame = frame.copy()
        if add_info_panel:
            # add info to visualize
            panel_coord = [20, 20, 380, 400]
            cv2.rectangle(
                frame, (panel_coord[0], panel_coord[1]),
                (panel_coord[2], panel_coord[3]), (0, 0, 0), -1)
            left_x1 = 60  # for title
            left_x2 = 230  # for value
            top_y = 70
            pad_y = 40
            n_digit = 2
            font_size = 1.0

            font = cv2.FONT_HERSHEY_DUPLEX
            speed = info['speed']
            cv2.putText(frame, 'speed: ', (left_x1, top_y), font,font_size, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, '%05.02f' % (speed), (left_x2, top_y),font, font_size, (255, 255, 255), 1, cv2.LINE_AA)
            top_y = top_y + pad_y

            throttle = info['control']['throttle']
            cv2.putText(frame, 'throttle: ', (left_x1, top_y), font,font_size, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, '%05.02f' % (throttle), (left_x2, top_y), font, font_size,(255, 255, 255),1, cv2.LINE_AA)
            top_y = top_y + pad_y

            brake = info['control']['brake']
            cv2.putText(frame, 'brake: ', (left_x1, top_y), font,
                        font_size, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, '%05.02f' % (brake), (left_x2, top_y),
                        font, font_size, (255, 255, 255), 1, cv2.LINE_AA)
            top_y = top_y + pad_y

            steering = info['control']['steering']
            if steering == 0:
                str_st = '00.00'
            elif steering > 0:
                str_st = '+' + '%04.02f' % (steering)
            else:
                str_st = '-' + '%04.02f' % (np.abs(steering))
            cv2.putText(frame, 'steering: ', (left_x1, top_y), font,
                        font_size, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, str_st, (left_x2, top_y), font,
                        font_size, (255, 255, 255), 1, cv2.LINE_AA)
            top_y = top_y + pad_y

            # draw the four direction key visualization
            init_gap = 0
            block_gap = 75
            block_size = 60

            top_y = 230

            total_width = block_gap * 2 + block_size
            left_bound = int(
                (panel_coord[2] -
                 panel_coord[0] -
                 total_width) /
                2 +
                panel_coord[0])

            if throttle > 0:
                cv2.rectangle(frame, (left_bound +
                                      block_gap, top_y), (left_bound +
                                                          block_gap +
                                                          block_size, top_y +
                                                          block_size),
                              (255, 255, 255), -1)  # up
            else:
                cv2.rectangle(frame, (left_bound +
                                      block_gap, top_y), (left_bound +
                                                          block_gap +
                                                          block_size, top_y +
                                                          block_size),
                              (30, 30, 30), -1)  # up

            if brake > 0:
                cv2.rectangle(frame, (left_bound +
                                      block_gap, top_y +
                                      block_gap), (left_bound +
                                                   block_gap +
                                                   block_size, top_y +
                                                   block_gap +
                                                   block_size),
                              (255, 255, 255), -1)  # down
            else:
                cv2.rectangle(frame, (left_bound +
                                      block_gap, top_y +
                                      block_gap), (left_bound +
                                                   block_gap +
                                                   block_size, top_y +
                                                   block_gap +
                                                   block_size),
                              (30, 30, 30), -1)  # down
            if steering < 0:
                cv2.rectangle(frame, (left_bound, top_y +
                                      block_gap), (left_bound +
                                                   block_size, top_y +
                                                   block_gap +
                                                   block_size),
                              (255, 255, 255), -1)  # left
            else:
                cv2.rectangle(frame, (left_bound, top_y +
                                      block_gap), (left_bound +
                                                   block_size, top_y +
                                                   block_gap +
                                                   block_size),
                              (30, 30, 30), -1)  # left
            if steering > 0:
                cv2.rectangle(frame, (left_bound +
                                      2 *
                                      block_gap, top_y +
                                      block_gap), (left_bound +
                                                   block_gap *
                                                   2 +
                                                   block_size, top_y +
                                                   block_gap +
                                                   block_size),
                              (255, 255, 255), -1)  # right
            else:
                cv2.rectangle(frame, (left_bound +
                                      2 *
                                      block_gap, top_y +
                                      block_gap), (left_bound +
                                                   block_gap *
                                                   2 +
                                                   block_size, top_y +
                                                   block_gap +
                                                   block_size),
                              (30, 30, 30), -1)
            opacity = 0.8
            cv2.addWeighted(frame, opacity, ori_frame, 1 - opacity, 0, frame)

        if add_vehicle_cube:
            v_list = info['object']
            cam_coords = np.array(info['pose']['position'], 'double')
            cam_rotation = np.array(info['pose']['rotation'], 'double')
            cam_near_clip = np.array(info['camera']['nearClip'], 'double')
            cam_field_of_view = np.array(info['camera']['fov'], 'double')
            visible_set = set(np.unique(id_map))

            if add_vehicle_bbox:
                info_vec_bbox_list = []

            if len(v_list) > 0:
                for j in range(len(v_list)):
                    v_info = v_list[j]
                    line_color = (0, 255, 0)
                    xxx = v_info['xxx']
                    yyy = v_info['yyy']
                    zzz = v_info['zzz']

                    bboxloose = v_info['bbox_loose']
                    size = (bboxloose[2] - bboxloose[0])*(bboxloose[3] - bboxloose[1])
                    dist = get_depth(v_info['location'],cam_coords,cam_rotation,cam_near_clip,cam_field_of_view)
                    if v_info['kitti']['occluded']:
                        line_color = (255, 0, 0)
                    if v_info['ignore']:
                        line_color = (0,0,255)


                    corner_info = False

                    if corner_info:
                        coord = np.array([xxx[0], yyy[0], zzz[0]], 'double')
                        before = is_before_clip_plane(
                            coord, cam_coords, cam_rotation, cam_near_clip,
                            cam_field_of_view)
                        if not before:
                            continue
                        cp1 = get_2d_from_3d(
                            coord, cam_coords, cam_rotation, cam_near_clip,
                            cam_field_of_view)
                        x1 = int(cp1[0] * 1920)
                        y1 = int(cp1[1] * 1080)
                        cv2.putText(
                            frame, str(
                                v_info['arrID']), (x1, y1), font, font_size,
                            (255, 255, 255), 1, cv2.LINE_AA)


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
                        '''
                        pp = np.array([xxx[0], yyy[0], zzz[0]], 'double')
                        before1 = is_before_clip_plane(pp, cam_coords,
                        cam_rotation, cam_near_clip, cam_field_of_view)
                        if before1:
                            cp = get_2d_from_3d(pp, cam_coords, cam_rotation,
                            cam_near_clip, cam_field_of_view)
                            x1 = int(cp[0] * 1920)
                            y1 = int(cp[1] * 1080)
                            cv2.putText(frame,str(v_info['class']),(x1, y1),
                            font, 0.3,(255,255,255),1,cv2.LINE_AA)
                        '''

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
                            if line_width > 0:
                                cv2.line(
                                    frame, (x1, y1), (x2, y2), line_color,
                                    line_width)

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
                            if line_width > 0:
                                cv2.line(
                                    frame, (x1, y1), (x2, y2), line_color,
                                    line_width)

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
                            if line_width > 0:
                                cv2.line(
                                    frame, (x1, y1), (x2, y2), line_color,
                                    line_width)

                            min_x, max_x, min_y, max_y = \
                                get_min_max_x_y_from_line(
                                    cp1[0], cp1[1], cp2[0], cp2[1])
                            if min_x is not None:
                                dd_x.append(min_x)
                                dd_x.append(max_x)
                                dd_y.append(min_y)
                                dd_y.append(max_y)
                    x1,x2,y1,y2 = int(min(dd_x)*1920),  int(max(dd_x)*1080),  int(min(dd_y)*1920),  int(max(dd_y)*1080)
                    x1,x2,y1,y2 = min(1919,max(0,x1)), min(1919,max(0,x2)), min(1079,max(0,y1)), min(1079,max(0,y2))

        if output_demo:
            out.write(frame)
        cv2.imshow('frame', frame)
        # time.sleep(0.1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if output_demo:
        out.release()
        cv2.destroyAllWindows()