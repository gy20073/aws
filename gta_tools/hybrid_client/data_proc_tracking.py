#!python2

import argparse
from pandas._libs import json
from tqdm import tqdm
from utils.proc_data_new import *
import os


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Provide the job name and the folder to be processed.')
    parser.add_argument(
        '-j',
        '--job',
        default='parse_data',
        help='Data processig job. parse_data/filter_for_detection_and_split/'
             'filter_for_tracking_and_split')
    parser.add_argument(
        '-p',
        '--path',
        default=None,
        help='The path to be processed.')
    parser.add_argument(
        '--skip',
        default=1,
        type=int,
        help='skip frame')
    args = parser.parse_args()

    if args.job == 'parse_data':
        # parse all the data in the folder, including find matching frame,
        # tranform to KITTI format, etc.
        base_dir = args.path
        dset = os.listdir(base_dir)
        dset_list = [d for d in dset]
        print(dset_list)
        for dset in tqdm(dset_list):
            parse_data_sub_multi_process(dset, base_dir)

    elif args.job == 'create_json':
        base_dir = args.path
        dset = os.listdir(base_dir)
        dset_list = [d for d in dset]
        print(dset_list)
        data = []
        endvid = []
        for subset in dset:
            jsonpath = os.path.join(base_dir,subset,'track.json')
            if os.path.exists(jsonpath):
                subdata = json.load(open(jsonpath,'r'))
                for idx, frame in enumerate(subdata):
                    if idx % args.skip == 0 and frame['object']!=[]:
                        data.append(frame)
                        endvid.append(False)
                    elif frame['object']!=[]:
                        print('found no veh in %d'%(frame['timestamp']))
                endvid[-1]=True
        json.dump(data,open(os.path.join(base_dir,'track.json'),'w'))
        json.dump(endvid,open(os.path.join(base_dir,'endvid.json'),'w'))
    else:
        raise NotImplementedError(
            'Please specify a valid job: parse_data/filter_for_detection_and_'
            'split/filter_for_tracking_and_split')