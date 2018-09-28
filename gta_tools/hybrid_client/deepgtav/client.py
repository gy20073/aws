#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import numpy as np
import socket
import struct
import pickle
import gzip
import time


class Targets:
    def __init__(self, datasetPath, compressionLevel):
        self.result_list = []
        self.count = 0
        self.datasetPath = datasetPath
        self.compressionLevel = compressionLevel

        self.pickleFile = None

        self.realtime_save = False

        if datasetPath is not None and self.realtime_save:
            self.pickleFile = gzip.open(
                datasetPath, mode='ab', compresslevel=compressionLevel)

    def parse(self, frame, jsonstr):
        try:
            dct = json.loads(jsonstr)
        except json.decoder.JSONDecodeError:
            dct = None
            print("receive from deepgtav error")
        if dct is not None:
            if self.realtime_save and self.pickleFile is not None:
                pickle.dump(dct, self.pickleFile)
            else:
                self.result_list.append(dct.copy())
        return dct

    def save_result(self):
        print("=====> Saving result...")
        if self.realtime_save:
            return

        serial = False
        if serial:
            self.pickleFile = gzip.open(
                self.datasetPath,
                mode='ab',
                compresslevel=self.compressionLevel)
            for i in range(len(self.result_list)):
                print('dump ' + str(i))
                pickle.dump(self.result_list[i], self.pickleFile)
        else:
            self.pickleFile = open(self.datasetPath, "wb")
            pickle.dump(self.result_list, self.pickleFile)
        print("=====> Done " + str(len(self.result_list)))


class Client:
    def __init__(
            self,
            ip='localhost',
            port=8000,
            datasetPath=None,
            compressionLevel=0):
        print('Trying to connect to DeepGTAV')

        self.targets = Targets(datasetPath, compressionLevel)
        self.ip = ip
        self.port = port
        self.recv_frame = False

        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((ip, int(port)))
        except BaseException:
            print('ERROR: Failed to connect to DeepGTAV')
        else:
            print('Successfully connected to DeepGTAV')

    def renewTarget(self, datasetPath, compressionLevel):
        self.targets = Targets(datasetPath, compressionLevel)

    def sendMessage(self, message):
        jsonstr = message.to_json().encode('utf-8')
        try:
            self.s.sendall(len(jsonstr).to_bytes(4, byteorder='little'))
            self.s.sendall(jsonstr)
        except Exception as e:
            print('ERROR: Failed to send message. Reason:', e)
            return False
        return True

    def recvMessage(self):
        t1 = time.time()
        if self.recv_frame:
            frame = self._recvall()
        else:
            frame = 'null'
        t2 = time.time()
        # print 'time to get frame: ' + str(t2-t1)
        if not frame:
            print('ERROR: Failed to receive frame')
            return None
        t1 = time.time()
        data = self._recvall()
        t2 = time.time()
        # print 'time to get data: ' + str(t2-t1)
        if not data:
            print('ERROR: Failed to receive message')
            return None

        t1 = time.time()
        parse_result = self.targets.parse(frame, data.decode('utf-8'))
        t2 = time.time()

        return parse_result

    def _recvall(self):
        # Receive first size of message in bytes
        data = b""
        while len(data) < 4:
            packet = self.s.recv(4 - len(data))
            if not packet:
                return None
            data += packet
        size = struct.unpack('I', data)[0]

        # We now proceed to receive the full message
        data = b""
        while len(data) < size:
            packet = self.s.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data

    def save_result(self):
        self.targets.save_result()

    def close(self):
        self.s.close()
