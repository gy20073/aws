#!python2

import argparse
from tqdm import tqdm
from utils.proc_data_utils import *


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Provide the job name and the folder to be processed.')
    parser.add_argument(
        '-j',
        '--job',
        default=None,
        help='Data processig job. parse_data/filter_for_detection_and_split/'
             'filter_for_tracking_and_split')
    parser.add_argument(
        '-p',
        '--path',
        default=None,
        help='The path to be processed.')
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

    elif args.job == 'filter_for_detection_and_split':
        # filter the data for detection and split into train/test.
        # change the detailed setting in filter_for_detection_and_split
        filter_for_detection_and_split(args.path)

    elif args.job == 'filter_for_tracking_and_split':
        # filter the data for tracking and split into train/test.
        # change the detailed setting in filter_for_tracking_and_split
        filter_for_tracking_and_split(args.path)

    else:
        raise NotImplementedError(
            'Please specify a valid job: parse_data/filter_for_detection_and_'
            'split/filter_for_tracking_and_split')
