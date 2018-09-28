import pickle
from glob import glob
import os
from tqdm import tqdm
import json

def writelist2txt(thelist, filename):
    with open(filename,'w') as thefile:
        for item in thelist:
            thefile.write("%s\n" % item)
img = []
speed = []
thottle = []
brake = []
steering = []
angle = []
direction = []
rootpath = '/scratch/yang/aws_data/carla_collect/daggerseg/daggerseg/'
imgpath_raw = glob(rootpath+'*')
imgpath = []
to_del = []
for path in tqdm(imgpath_raw):
    if os.path.exists(os.path.join(path,'info_match.p')) and os.path.exists(os.path.join(path,'label.pkl')):
        imgpath.append(path)
    else:
        to_del.append(path)
train_len = int(len(imgpath)*0.9)
for path in tqdm(imgpath[:train_len]):
    try:
        parsedinfo = json.load(open(os.path.join(path,'parsedinfo.json'),'r'))
    except:
        pass
    sortedtime = sorted(parsedinfo.keys())
    for idx, t in enumerate(sortedtime):
        frame = parsedinfo[t]
        if frame['dagger']:
            continue
        img.append(os.path.join(os.path.relpath(path,rootpath),str(t)+'_final.png'))
        speed.append(frame['speed'])
        thottle.append(frame['throttle'])
        brake.append(frame['brake'])
        steering.append(frame['steering'])
        angle.append(frame['yawRate'])
        direction.append(frame['direction'])

writelist2txt(img, rootpath+'train_imgs.txt')
writelist2txt(speed, rootpath+'train_speeds.txt')
writelist2txt(angle, rootpath+'train_angles.txt')
writelist2txt(brake, rootpath+'train_brakes.txt')
writelist2txt(steering, rootpath+'train_steerings.txt')
writelist2txt(thottle, rootpath+'train_thottles.txt')
writelist2txt(direction, rootpath+'train_direction.txt')

img = []
speed = []
thottle = []
brake = []
steering = []
angle = []
direction = []
for path in tqdm(imgpath[train_len:]):
    try:
       parsedinfo = json.load(open(os.path.join(path,'parsedinfo.json'),'r'))
    except:
        pass
    sortedtime = sorted(parsedinfo.keys())
    for idx, t in enumerate(sortedtime):
        frame = parsedinfo[t]
        if frame['dagger']:
            continue
        img.append(os.path.join(os.path.relpath(path,rootpath),str(t)+'_final.png'))
        speed.append(frame['speed'])
        thottle.append(frame['throttle'])
        brake.append(frame['brake'])
        steering.append(frame['steering'])
        angle.append(frame['yawRate'])
        direction.append(frame['direction'])

writelist2txt(img, rootpath + 'val_imgs.txt')
writelist2txt(speed, rootpath + 'val_speeds.txt')
writelist2txt(angle, rootpath + 'val_angles.txt')
writelist2txt(brake, rootpath + 'val_brakes.txt')
writelist2txt(steering, rootpath + 'val_steerings.txt')
writelist2txt(thottle, rootpath + 'val_thottles.txt')
writelist2txt(direction, rootpath+'val_direction.txt')
