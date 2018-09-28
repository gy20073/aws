#!python2
# !/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Main function to collect data from GTA
"""

import argparse
import socket
import threading
import time
from multiprocessing import Manager
from queue import Queue
from math import sqrt

import cv2
import numpy as np
import png
import sys

from deepgtav.client import Client
from deepgtav.messages import Start, Stop, Dataset, Scenario
from utils.client_save_utils import BufferClient, capture_gta_to_queue
from utils.flowutils import writeFlow2
from utils.client_save_utils import UNIT

from utils.client_save_utils import save_game_info

MS = UNIT(0)        # Milliseconds

GTA_path = "C:/Program Files (x86)/Steam/steamapps/common/Grand Theft Auto V/"

# Weather Options
weathers = {"CLEAR": 5,
            "EXTRASUNNY": 2,
            "CLOUDS": 5,
            "RAIN": 1,
            "CLEARING": 1,
            "THUNDER": 1,
            "OVERCAST": 0,
            "SMOG": 0,
            "FOGGY": 0,
            "XMAS": 0,
            "SNOWLIGHT": 0,
            "BLIZZARD": 0,
            "NEUTRAL": 0,
            "SNOW": 0}
weather_list = []
for k, v in weathers.items():
    weather_list += v * [k]


def queue_reader(queue, id_tracker):
    # Read from the queue
    # id_tracker range: 0-65535, 107-0-vec, 26-2-ped, 53-4-others
    # type_dict = {107:0, 26:20000, 53:40000, 80:40000}
    type_dict = {107: 40000, 26: 20000, 53: 0, 80: 40000}
    print('==> Staring to read from queue...')
    while True:
        msg = queue.get()
        file_name, data = msg
        if file_name == 'DONE':
            break

        if 'final' in file_name:
            cv2.imwrite(file_name, data)

        elif 'flow' in file_name:
            # renderedflow = flow_to_image(data.copy())
            # cv2.imwrite(file_name+'rendered.png',renderedflow)
            writeFlow2(file_name, data)
            # writeFlow(file_name + '.flo', data)

        elif 'depth' in file_name:
            # depth_map = data
            # depth_map.setflags(write=True)
            # # we only care about 1km
            # depth_map[depth_map < 1. / 6800.] = 1. / 6800.
            # # depth_map = 1. / depth_map
            # # depth_map = depth_map / 6800. * 65535
            # depth_map = 0.05 / depth_map
            # depth_map = depth_map.astype('uint16')
            # f = open(file_name, 'wb')
            # w = png.Writer(
            #     depth_map.shape[1],
            #     depth_map.shape[0],
            #     greyscale=True,
            #     bitdepth=16)
            # w.write(f, depth_map)
            depth_map = data
            depth_map = 0.05 * 1000 / depth_map
            depth_map = depth_map.astype('uint32')
            h, w = depth_map.shape
            depth_map.dtype = 'uint8'
            depth_map = depth_map.reshape((h,w,4))
            depth_map = depth_map[:,:,:3]
            # cv2 stuff here
            cv2.imwrite(file_name, depth_map)

        elif 'id' in file_name:
            # c_id_map = np.zeros_like(data, 'uint16')
            # for uu in np.unique(data):
            #     if uu == 0:
            #         continue
            #     # add a mutex to protect the id_tracker
            #
            #     if uu not in id_tracker.keys():
            #         mutex.acquire()
            #         type_id = int(uu / 1e7)
            #         base_id = type_dict[type_id]
            #         final_id = base_id + np.random.randint(0, 20000)
            #         while final_id in id_tracker.values():
            #             final_id = base_id + np.random.randint(0, 20000)
            #         id_tracker[uu] = final_id
            #         mutex.release()
            #     c_id_map[data == uu] = id_tracker[uu]
            #
            # f = open(file_name, 'wb')
            # w = png.Writer(
            #     c_id_map.shape[1],
            #     c_id_map.shape[0],
            #     greyscale=True,
            #     bitdepth=16)
            # w.write(f, c_id_map)
            #littleendian = sys.byteorder == 'little'
            #data = data.astype('int32')
            h, w = data.shape
            data.dtype = 'uint8'
            data = data.reshape((h, w, 4))
            data = data[:,:,:-1]
            #if littleendian:
            #    data = data[:, :, 1:]
            #else:
            #    data = data[:, :, 3:]

            cv2.imwrite(file_name, data)

        else:
            raise ValueError("Invalid Filename Received")


# Store a dataset file with data coming from DeepGTAV
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=None)

    # Path Parameters
    parser.add_argument('-pa', '--path', default='G:/comp/')
    parser.add_argument(
        '-n',
        '--name',
        default='rec',
        help='Name of the record (will be augmented)')

    # Client Parameters
    parser.add_argument('-l', '--host', default='localhost',
                        help='The IP where DeepGTAV is running')
    parser.add_argument(
        '-p',
        '--port',
        default=8000,
        help='The port where DeepGTAV is running')
    parser.add_argument(
        '-cl',
        '--comp_level',
        default=0,
        help='Compression level for Target class')

    # Scenario Parameters
    parser.add_argument(
        '-po',
        '--policy',
        default=2191, #6287, #1074266286, #1074528959, #1074528828, #786603,
        type=int,
        help='mode (0-4294967296), speed (0-20), only if auto drive')
    parser.add_argument(
        '--maxspeed',
        default=60.0,  # 1074528959, #1074528828, #786603,
        type=float,
        help='mode (0-4294967296), speed (0-20), only if auto drive')

    parser.add_argument(
        '-v',
        '--vehicle',
        default='oracle',
        help='blista, voltic, packer, oracle')
    parser.add_argument(
        '-w',
        '--weather',
        default='EXTRASUNNY',
        help='CLEAR, EXTRASUNNY, CLOUDS, OVERCAST, RAIN, CLEARING, THUNDER, '
             'SMOG, FOGGY, XMAS, SNOWLIGHT, BLIZZARD, NEUTRAL, SNOW')
    parser.add_argument(
        '-t',
        '--time',
        nargs=2,
        default=[
            10,
            30],
        help='HH, MM')
    parser.add_argument('-loc', '--location', nargs=2,
                        default=[-281.8045, -1396.085],
                        help='[-274.2653, -1122.707]')
    parser.add_argument('-lxr',
                        '--loc_x_range',
                        nargs=2,
                        default=[-1000,
                                 500],
                        help='Location x range, default is urban')
    parser.add_argument('-lyr',
                        '--loc_y_range',
                        nargs=2,
                        default=[-2000,
                                 0],
                        help='Location y range, default is urban')

    # Dataset Parameters
    parser.add_argument(
        '-r',
        '--record_rate',
        default=30,
        help='Rate of recording in Hz')
    parser.add_argument(
        '-fr',
        '--frame_resolution',
        nargs=2,
        default=[
            1280,
            720],
        help='Frame resolution in width and height')
    parser.add_argument('-thr', '--throttle', default=True, help='throttle')
    parser.add_argument('-b', '--brake', default=True, help='brake')
    parser.add_argument('-s', '--steering', default=True, help='steering')
    parser.add_argument('-vs', '--vehicles', default=True, help='vehicles')
    parser.add_argument(
        '-ts',
        '--trafficSigns',
        default=False,
        help='trafficSigns')
    parser.add_argument('-ped', '--peds', default=True, help='peds')
    parser.add_argument('-rew', '--reward', default=False, help='reward')
    parser.add_argument('-dt', '--dataset_time', default=True, help='time')
    parser.add_argument('-sp', '--speed', default=True, help='speed')
    parser.add_argument('-y', '--yawRate', default=True, help='yawRate')
    parser.add_argument('-il', '--if_location', default=True, help='location')
    parser.add_argument(
        '-rf',
        '--roadinfo',
        default=True,
        help='show roadinfo')
    # Training Parameters
    parser.add_argument('-nt', '--n_trials', default=1, type=int,
                        help='Number of trials to run')
    parser.add_argument(
        '-nw',
        '--n_workers',
        default=4,
        help='Number of workers to receive messages in parallel')
    parser.add_argument(
        '-ad',
        '--auto_drive',
        default=True,
        help='Determine driving mode for Scenario')
    parser.add_argument(
        '-vl',
        '--video_len',
        default=1200,
        type=int,
        help='Length of video to record in seconds, default 0.42*3600')

    args = parser.parse_args()
    args.driving_mode = [args.policy, args.maxspeed]
    print(args.driving_mode)

    # Creates a new connection to DeepGTAV using the specified ip and port.
    # If desired, a dataset path and compression level can be set to store in
    # memory all the data received in a gziped pickle file.

    client = Client(
        ip=args.host,
        port=args.port,
        datasetPath="tmp.gz",
        compressionLevel=0)
    for i_trial in range(args.n_trials):
        # Scenario parameters: time, weather, location
        with open("highway.np", "rb") as f:
            x_list, y_list, _ = np.load(f)
        # with open("road.np", "rb") as f:
        #     x_list_, y_list_, z_list_ = np.load(f)
        # x_list, y_list, z_list = np.hstack((x_list,x_list_)), np.hstack((y_list, y_list_)), np.hstack((z_list, z_list_))
        # with open("sampled_path_points.np", "rb") as f:
        #     x_list, y_list = np.load(f)
        args.time[0] = 12#np.random.randint(8, 20)
        args.time[1] = 51#np.random.randint(0, 60)
        weather = 'EXTRASUNNY'#weather_list[np.random.randint(0, len(weather_list))]
        rand_loc_idx = np.random.randint(len(x_list))
        while not (x_list[rand_loc_idx] > args.loc_x_range[0]
                   and x_list[rand_loc_idx] < args.loc_x_range[1]
                   and y_list[rand_loc_idx] > args.loc_y_range[0]
                   and y_list[rand_loc_idx] < args.loc_y_range[1]):
            rand_loc_idx = np.random.randint(len(x_list))
        # np.random.randint(-1450, 1400) # 5000 * np.random.rand() - 2500
        args.location[0] = x_list[rand_loc_idx]
        #args.location[0] = -400
        # np.random.randint(-2700, 330) # 8000 * np.random.rand() - 2000
        args.location[1] = y_list[rand_loc_idx]
        #args.location[1] = -1000
        # Renew data path for client

        args.des = [0,0,0]
        with open("highway.np", "rb") as f:
            x_list_des, y_list_des, z_list_des = np.load(f)
        with open("road.np", "rb") as f:
            x_list_, y_list_, z_list_ = np.load(f)
        x_list_des, y_list_des, z_list_des = np.hstack((x_list_des,x_list_)), np.hstack((y_list_des, y_list_)), np.hstack((z_list_des, z_list_))
        rand_loc_idx = np.random.randint(len(x_list_des))
        while not (sqrt((x_list_des[rand_loc_idx]-args.location[0])**2+(y_list_des[rand_loc_idx]-args.location[1])**2) > 3000 and
                   sqrt((x_list_des[rand_loc_idx] - args.location[0]) ** 2 + (y_list_des[rand_loc_idx] - args.location[1]) ** 2) < 4000):
            rand_loc_idx = np.random.randint(len(x_list_des))
        args.des = [x_list_des[rand_loc_idx], y_list_des[rand_loc_idx], z_list_des[rand_loc_idx]]

        x_int = int(args.location[0])
        y_int = int(args.location[1])
        x_des = int(args.des[0])
        y_des = int(args.des[1])
        ISOTIMEFORMAT = '%m%d%H%M'
        time_str = str(time.strftime(ISOTIMEFORMAT))
        extended_name = '{}_{}_{}_{}h{}m_x{}y{}tox{}y{}'.format(
            args.name, time_str, str(
                weather.lower()), str(
                args.time[0]), str(
                args.time[1]), str(x_int), str(y_int), str(x_des), str(y_des))
        data_full_path = args.path + extended_name + '/info.gz'
        info_full_path = args.path + extended_name + '/realtimeinfo.gz'
        print("Data Path: {}".format(data_full_path))
        client.renewTarget(
            datasetPath=data_full_path,
            compressionLevel=args.comp_level)  # Rename

        # Configures the information that we want DeepGTAV to generate and
        # send to us.
        # See deepgtav/messages.py to see what options are supported
        dataset = Dataset(
            rate=args.record_rate,
            frame=args.frame_resolution,
            throttle=args.throttle,
            brake=args.brake,
            steering=args.steering,
            vehicles=args.vehicles,
            trafficSigns=args.trafficSigns,
            peds=args.peds,
            reward=args.reward,
            speed=args.speed,
            yawRate=args.yawRate,
            location=args.if_location,
            time=args.dataset_time,
            roadinfo=args.roadinfo,
            direction=args.des
        )

        if args.auto_drive:
            scenario = Scenario(
                drivingMode=args.driving_mode,
                vehicle=args.vehicle,
                weather=weather,
                time=args.time,
                location=args.location)
        else:
            scenario = Scenario(
                vehicle=args.vehicle,
                weather=weather,
                time=args.time,
                location=args.location)
        client.sendMessage(Start(dataset=dataset, scenario=scenario))
        time.sleep(5)  # wait for the scene to be ready

        # Setup multi-threading for receiving messages
        queue = Queue()
        manager = Manager()
        infosaver = save_game_info(img_dir=info_full_path)
        id_tracker = manager.dict()
        mutex = threading.Lock()

        threadpoll = []
        for i in range(args.n_workers):
            out_thread = threading.Thread(
                target=queue_reader, args=(
                    queue, id_tracker))
            out_thread.daemon = True
            threadpoll.append(out_thread)
            out_thread.start()
        c = BufferClient(('localhost', 8766))
        capture_gta_to_queue(args.path + extended_name,c,queue)
        # t = threading.Thread(
        #     target=capture_gta_to_queue, args=(
        #         args.path + extended_name, c, queue))
        # t.daemon = True
        location = args.location
        stoptime = time.time() + args.video_len
        c.fetchGameState(infosaver.saveinfotolist, 1 * MS)
        # Start listening for messages coming from DeepGTAV.
        while (sqrt((location[0]-args.des[0])**2+(location[1]-args.des[1])**2)>100) and time.time()<stoptime:
            try:
                start = time.time()
                # message is python dict; keys: timestamp, trafficSigns, speed,
                # frame
                message = client.recvMessage()
                location = message['location']
                print(message['speed'])
                if message is None:
                    stoptime = time.time()
                # The frame is a numpy array and can be displayed using OpenCV
                # or similar
                # image = frame2numpy(message['frame'],
                # (frame_resolution[0],frame_resolution[1]))
                # cv2.imshow('img',image)
                # cv2.waitKey(-1)
                stop = time.time()
                #print('total time: ' + str(stop - start) + 's')
            except KeyboardInterrupt:
                break

        # Stop DeepGTAV
        c.disconnect()
        del c
        client.sendMessage(Stop())
        client.save_result()
        for i in range(1000):
            queue.put(['DONE', None])
        infosaver.saveinfo()
        time.sleep(5)
        print("=====> Finish writing")
        # client.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # client.s.connect((client.ip,int(client.port)))
        del queue
        del infosaver
    client.close()
