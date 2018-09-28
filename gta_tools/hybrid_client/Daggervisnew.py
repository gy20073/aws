#!python2
"""
Add bounding boxes and control signals to videos
"""
import os
import pickle
import argparse
import pickle
import cv2
import numpy as np

from utils.utils import *
from utils.data_utils import find_time_match_info
import json
direction_info = {
    0: 'You Have Arrive',
    1: 'Recalculating Route, Please make a u-turn where safe',
    2: 'Please Proceed the Highlighted Route',
    3: 'Keep Left',
    4: 'In {a} Turn Left',
    5: 'In {a} Turn Right',
    6: 'Keep Right',
    7: 'In {a} Go Straight Ahead',
    8: 'In {a} Join the freeway',
    9: 'In {a} Exit Freeway'
}


def parsedirectioninfo(directioninfo):
    return direction_info[directioninfo[0]].format(
        a=directioninfo[1]) + ', remaining {b}'.format(b=directioninfo[2])

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description=None)
    parser.add_argument(
        '-p',
        '--path',
        help='Directory to retrieve images',
        required=True)
    args = parser.parse_args()

    output_demo = False
    # read info list
    dir_name = args.path
    info_dict_list = json.load(
        open(
            os.path.join(
                dir_name,
                'parsedinfo.json'),
            'r'))

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

    ts_list = sorted(info_dict_list.keys())
    for idx ,ts in enumerate(ts_list):
        info = info_dict_list[ts]
        frame = cv2.imread(os.path.join(dir_name, str(ts) + '_final.png'))
        id_map =generate_id_map(os.path.join(dir_name, str(ts) + '_id.png'))
        # time.sleep(0.2)
        ori_frame = frame.copy()
        if info['dagger']:
          continue
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
            cv2.putText(frame, 'speed: ', (left_x1, top_y), font,
                        font_size, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, '%05.02f' % (speed), (left_x2, top_y),
                        font, font_size, (255, 255, 255), 1, cv2.LINE_AA)
            top_y = top_y + pad_y

            throttle = info['throttle']
            cv2.putText(frame, 'throttle: ', (left_x1, top_y), font,
                        font_size, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(
                frame, '%05.02f' %
                       (throttle), (left_x2, top_y), font, font_size,
                (255, 255, 255),
                1, cv2.LINE_AA)
            top_y = top_y + pad_y

            brake = info['brake']
            cv2.putText(frame, 'brake: ', (left_x1, top_y), font,
                        font_size, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, '%05.02f' % (brake), (left_x2, top_y),
                        font, font_size, (255, 255, 255), 1, cv2.LINE_AA)
            top_y = top_y + pad_y

            steering = info['steering']
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

            yawrate = info['yawRate']
            if yawrate == 0:
                str_st = '00.00'
            elif yawrate > 0:
                str_st = '+' + '%04.02f' % (yawrate)
            else:
                str_st = '-' + '%04.02f' % (np.abs(yawrate))
            cv2.putText(frame, 'yawrate: ', (left_x1, top_y), font,
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
                              (30, 30, 30), -1)  # right
            if output_demo:
                out.write(frame)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break