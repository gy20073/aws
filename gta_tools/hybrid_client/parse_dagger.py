import pickle
from glob import glob
import os, sys
from tqdm import tqdm
import numpy as np
import json

def find_time_match_info(dir_name):
    info_pickle_name = os.path.join(dir_name, 'info.gz')
    print(info_pickle_name)
    assert os.path.exists(info_pickle_name)
    info_list = pickle.load(open(info_pickle_name, "rb"))
    # info_list is info.gz

    real_info_pickle_name = os.path.join(dir_name, 'realtimeinfo.gz')
    assert os.path.exists(real_info_pickle_name)
    real_info_list = pickle.load(open(real_info_pickle_name, "rb"))
    # real info list is realtimeinfo.gz

    f_list = os.listdir(dir_name)
    # f list is the image names list

    #import pdb; pdb.set_trace()

    im_ts_list = [int(fname[0:13]) for fname in f_list if 'final' in fname]
    pk_ts_list = [info['timestamp'] for info in info_list]
    real_ts_list = [real_info['time'] for real_info in real_info_list]

    print(len(im_ts_list), len(pk_ts_list), len(real_ts_list))

    match_info = dict()
    pk_ts_list = np.array(pk_ts_list)
    real_ts_list = np.array(real_ts_list)
    error_list = []
    for im_ts in im_ts_list:
        best_match = np.argmin(np.abs(pk_ts_list - im_ts))
        match_info[im_ts] = info_list[best_match]

        min_error = np.abs(pk_ts_list[best_match] - im_ts)
        error_list.append(min_error)
    real_error_list = []
    for im_ts in im_ts_list:
        best_match = np.argmin(np.abs(real_ts_list - im_ts))
        match_info[im_ts]['cam_coords'] = real_info_list[best_match]['cam_pos']
        match_info[im_ts]['cam_rotation'] = \
            real_info_list[best_match]['cam_rot']
        match_info[im_ts]['cam_field_of_view'] = \
            real_info_list[best_match]['cam_field_of_view']
        match_info[im_ts]['cam_near_clip'] = \
            real_info_list[best_match]['cam_near_clip']
        min_error = np.abs(real_ts_list[best_match] - im_ts)
        real_error_list.append(min_error)

    print('==> Average min error: {}MS'.format(np.mean(error_list)))
    print('==> Average min error: {}MS'.format(np.mean(real_error_list)))
    print('==> Max error: {}MS in frame{}'.format(
        np.amax(error_list), np.argmax(error_list)))
    pickle.dump(match_info, open(os.path.join(dir_name, 'info_match.p'), 'wb'), protocol=2)


imgpath_raw = glob(sys.argv[1] + '/*')
imgpath = []
to_del = []
for path in tqdm(imgpath_raw):
    if os.path.exists(os.path.join(path,'info.gz')) and os.path.exists(os.path.join(path,'label.pkl')):
        imgpath.append(path)
    else:
        to_del.append(path)

for path in tqdm(imgpath):
    find_time_match_info(path)
    info_dict = pickle.load(open(os.path.join(path,'info_match.p'),'rb'))
    dagger_info =  pickle.load(open(os.path.join(path,'label.pkl'),'rb'))
    if info_dict is None or dagger_info is None:
        continue
    dagger_info = zip(dagger_info[0::2], dagger_info[1::2])
    info_parsed = {}
    dagger = next(dagger_info)
    for i in sorted(info_dict):
        time = float(i)
        frame = info_dict[i]
        info_parsed[i] = {}
        info_parsed[i]['throttle'] = frame['throttle']
        info_parsed[i]['brake'] = frame['brake']
        info_parsed[i]['steering'] = frame['steering']
        info_parsed[i]['speed'] = frame['speed']
        info_parsed[i]['yawRate'] = frame['yawRate']
        info_parsed[i]['location'] = frame['location']
        info_parsed[i]['direction'] = frame['direction']
        if time>dagger[1]*1000:
            try:
                dagger = next(dagger_info)
            except:
                pass
        if dagger[0]*1000+500<time<dagger[1]*1000-500:
            info_parsed[i]['dagger'] = False
        else:
            info_parsed[i]['dagger'] = True
    json.dump(info_parsed,open(os.path.join(path,'parsedinfo.json'),'w'),indent=2)
print(to_del)