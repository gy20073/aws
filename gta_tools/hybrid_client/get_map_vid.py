#!python2
"""
Reconstruct GTA messages to videos
"""
import argparse
import os
import pickle

import cv2
import matplotlib.pyplot as plt
import numpy as np
from os.path import join

from utils.data_utils import find_time_match_info
from utils.flowutils import flow_to_image, readFlow2
from utils.roadinfoparser import parseroadinfo
from utils.utils import get_2d_from_3d, draw3dbbox
from glob import glob

color_dict = dict()


def get_random_color():
    color = [np.random.randint(0, 256) for _ in range(3)]
    return tuple(color)


def process_frame(data, map_type, output_prefix, apply_color_map, final,
                  info, lanepoints, ts):
    if map_type == 'depth':
        min_thres = 1e-3
        data[data < min_thres] = min_thres
        data = 1. / data
        if apply_color_map:
            norm = plt.Normalize(vmin=data.min(), vmax=data.max())
            data = cmap(norm(data))
            frame = data[:, :, :3]
            frame = (frame * 255).astype('uint8')
        else:
            data = (data / np.amax(data) * 255).astype('uint8')
            frame = np.zeros([1080, 1920, 3], 'uint8')
            for i in range(3):
                frame[:, :, i] = data
    elif map_type == 'id':
        if output_prefix == 'segment':
            data = data[:,:,0]+data[:,:,1]<<8+data[:,:,2]<<16
        global color_dict
        frame = np.zeros([1080, 1920, 3], 'uint8')
        u_id = np.unique(data)
        for uu in u_id:
            if uu == 0:
                continue
            if uu not in color_dict.keys():
                color_dict[uu] = get_random_color()
            y_list, x_list,_ = np.where(data == uu)
            frame[y_list, x_list, :] = color_dict[uu]
    elif map_type == 'flow':
        frame = flow_to_image(data)
    elif map_type == '3dbbox':
        cam_coords = info[ts]['cam_coords']
        cam_rotation = np.array(info[ts]['cam_rotation'])
        cam_near_clip = info[ts]['cam_near_clip']
        cam_field_of_view = info[ts]['cam_field_of_view']
        frame = data
        for vechicle in info[ts]['vehicles']:
            if vechicle['visible']:
                position = np.vstack((np.array(vechicle['xxx']),
                                      np.array(vechicle['yyy']),
                                      np.array(vechicle['zzz'])))
                position = position.T
                bboxpoint = []
                for i in position:
                    point = get_2d_from_3d(i, cam_coords, cam_rotation,
                                       cam_near_clip,
                                       cam_field_of_view)
                    if point is not None:
                        bboxpoint.append([point[0] * 1920, point[1] * 1080])
                bboxpoint = np.array(bboxpoint).T
                if bboxpoint.size > 0 and bboxpoint.shape[1] == 8:
                    draw3dbbox(frame, bboxpoint, (0, 255, 0))
        for ped in info[ts]['peds']:
            if ped['visible']:
                position = np.vstack((np.array(ped['xxx']),
                                  np.array(ped['yyy']),
                                  np.array(ped['zzz'])))
                position = position.T
                bboxpoint = []
                for i in position:
                    point = get_2d_from_3d(i, cam_coords, cam_rotation,
                                       cam_near_clip,
                                       cam_field_of_view)
                    if point is not None:
                        bboxpoint.append([point[0] * 1920, point[1] * 1080])
                bboxpoint = np.array(bboxpoint).T
                if bboxpoint.size > 0 and bboxpoint.shape[1] == 8:
                    draw3dbbox(frame, bboxpoint, (255, 255, 0))
        countpoints=0
        roadinfo = parseroadinfo(info[ts]['roadinfo'])
        if "rightroadpoint" in roadinfo:
            pointidx = lanepoints.index(roadinfo["rightroadpoint"])
            for i in range(pointidx,min(pointidx+20,len(lanepoints))):
                p=lanepoints[i]
                point2d = get_2d_from_3d(p, cam_coords, cam_rotation,
                                       cam_near_clip,
                                       cam_field_of_view)
                if point2d is not None and point2d[0]>0 and point2d[1]>0:
                    cv2.circle(data,(int(point2d[0]*1920),int(point2d[1]*1080)),0,(0,0,255),thickness=10)
    else:
        raise NotImplementedError
    if final is not None:
        frame = np.concatenate((final, frame), axis=1)
    return frame


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description=None)
    parser.add_argument(
        '-p',
        '--path',
        help='Directory to retrieve images',
        required=True)
    parser.add_argument(
        '-o',
        '--output_demo',
        action="store_true",
        default=False)
    parser.add_argument(
        '-f',
        '--with_final',
        action="store_true",
        default=False)
    parser.add_argument(
        '-m',
        '--map_type',
        default='id',
        help='Types of map: id, depth')
    parser.add_argument(
        '-op',
        '--output_prefix',
        default='s',
        help='segment or not segment')
    parser.add_argument(
        '-a',
        '--apply_color_map',
        action="store_true",
        default=True,
        help='Base directory to retrieve images')
    args = parser.parse_args()

    if args.apply_color_map:
        cmap = plt.cm.plasma

    # read info list
    dir_name = args.path
    dir_name_list = glob(os.path.join(dir_name,'rec*'))
    for dir_name in dir_name_list:
        if not os.path.exists(os.path.join(dir_name, 'info_match.p')):
            try:
                find_time_match_info(dir_name)
            except:
                continue
        info_dict = pickle.load(open(os.path.join(dir_name, 'info_match.p'), 'rb'))

        # set output video
        if args.output_demo:
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            out = cv2.VideoWriter(dir_name + '_' + args.output_prefix + '.mp4',
                                  fourcc, 15.0, (2560, 720))
        ts_list = sorted(info_dict.keys())
        point = []
        #for ts in ts_list:
        #    roadinfo = parseroadinfo(info_dict[ts]['roadinfo'])
        #    if "rightroadpoint" in roadinfo:
        #        leftpoint = tuple(roadinfo['leftroadpoint'])
        #        rightpoint = tuple(roadinfo['rightroadpoint'])
        #        if not (leftpoint in point):
        #            point.append(leftpoint)
        #        if not (rightpoint in point):
        #            point.append(rightpoint)
        point = list(point)
        for ts in ts_list[:2000]:
            info = info_dict[ts]

            if args.map_type == 'flow':
                raw_frame = readFlow2(
                    os.path.join(
                        dir_name,
                        str(ts) +
                        '_' +
                        args.map_type))
            elif args.map_type == '3dbbox':
                raw_frame = cv2.imread(
                    os.path.join(
                        dir_name, str(ts) + '_final.png'), -1)
            else:
                raw_frame = cv2.imread(
                    os.path.join(
                        dir_name, str(ts) + '_' + args.map_type + '.png'), -1)
            final = None
            if args.with_final:
                final = cv2.imread(
                    os.path.join(
                        dir_name, str(ts) + '_final' + '.png'), -1)

            frame = process_frame(
                raw_frame,
                args.map_type,
                args.output_prefix,
                args.apply_color_map,
                final,
                info_dict,
                point,
                ts)

            if args.output_demo:
                frameS = cv2.resize(frame, (2560, 720))
                out.write(frameS)
            frameS = cv2.resize(frame, (2560, 720))
            cv2.imshow('frame', frameS)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if args.output_demo:
            out.release()
            cv2.destroyAllWindows()
