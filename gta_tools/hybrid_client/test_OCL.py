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
import pickle

import cv2
import numpy as np
import png
import sys

from deepgtav.client import Client
from deepgtav.messages import Start, Stop, Dataset, Scenario, Commands
from utils.client_save_utils import BufferClient, capture_gta_to_queue_only_final
from utils.client_save_utils import save_game_info


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=None)
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

    parser.add_argument(
        '-v',
        '--vehicle',
        default='oracle',
        help='blista, voltic, packer, oracle')
    parser.add_argument(
        '-w',
        '--weather',
        default='CLEAR',
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
                        default=[-2000,
                                 1500],
                        help='Location x range, default is urban')
    parser.add_argument('-lyr',
                        '--loc_y_range',
                        nargs=2,
                        default=[-2000,
                                 1500],
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
    parser.add_argument('-nw','--n_workers',default=4,help='Number of workers to receive messages in parallel')

    args = parser.parse_args()

    # Creates a new connection to DeepGTAV using the specified ip and port.
    # If desired, a dataset path and compression level can be set to store in
    # memory all the data received in a gziped pickle file.

    client = Client(ip=args.host,port=args.port)
    with open("road.np", "rb") as f:
        x_list, y_list, z_list = np.load(f)
    args.time[0] = np.random.randint(8, 20)
    args.time[1] = np.random.randint(0, 60)
    weather = 'EXTRASUNNY'
    rand_loc_idx = np.random.randint(len(x_list))
    while not (x_list[rand_loc_idx] > args.loc_x_range[0]
               and x_list[rand_loc_idx] < args.loc_x_range[1]
               and y_list[rand_loc_idx] > args.loc_y_range[0]
               and y_list[rand_loc_idx] < args.loc_y_range[1]):
        rand_loc_idx = np.random.randint(len(x_list))
    args.location[0] = x_list[rand_loc_idx]
    args.location[1] = y_list[rand_loc_idx]
    args.des = [0,0,0]
    with open("path.np", "rb") as f:
        x_list_des, y_list_des, z_list_des = np.load(f)
    with open("road.np", "rb") as f:
        x_list_, y_list_, z_list_ = np.load(f)
    x_list_des, y_list_des, z_list_des = np.hstack((x_list_des,x_list_)), np.hstack((y_list_des, y_list_)), np.hstack((z_list_des, z_list_))
    rand_loc_idx = np.random.randint(len(x_list_des))
    while not (sqrt((x_list_des[rand_loc_idx]-args.location[0])**2+(y_list_des[rand_loc_idx]-args.location[1])**2) > 100 and
               sqrt((x_list_des[rand_loc_idx] - args.location[0]) ** 2 + (y_list_des[rand_loc_idx] - args.location[1]) ** 2) < 1000):
        rand_loc_idx = np.random.randint(len(x_list_des))
    args.des = [x_list_des[rand_loc_idx], y_list_des[rand_loc_idx], z_list_des[rand_loc_idx]]
    x_int = int(args.location[0])
    y_int = int(args.location[1])
    x_des = int(args.des[0])
    y_des = int(args.des[1])
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
    scenario = Scenario(
        vehicle=args.vehicle,
        weather=weather,
        time=args.time,
        location=args.location)
    client.sendMessage(Start(dataset=dataset, scenario=scenario))
    time.sleep(5)  # wait for the scene to be ready
    # Setup multi-threading for receiving messages
    queue = Queue(maxsize=1)
    manager = Manager()
    c = BufferClient(('localhost', 8766))
    capture_gta_to_queue_only_final(c,queue)


    while True:
        imgtime, img = queue.get()
        imgtime = int(time)
        speed, angle = model(Variable(torch.Tensor(img)))
        throttle, brake, steering = pid(speed, angle)
        client.sendMessage(
            Commands(throttle=throttle, brake=brake, steering=steering, manual=1))

    del queue
    client.close()
