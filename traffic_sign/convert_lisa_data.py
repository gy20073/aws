from PIL import Image, ImageDraw, ImageFont
import os
import argparse, random
import shutil
from os import listdir

parser = argparse.ArgumentParser(description='Mark or crop each annotation.',
                                 epilog='This program will create a directory called \'annotations\' and save the output there. The directory will be created in the folder where the annotation file is located. If the directory exists already, that will be used, and any existing files will be overwritten.')
parser.add_argument('path', metavar='/data/yang/code/aws/scratch/LISA/allAnnotationsRGB.csv', type=str,
                    help='The full path to the csv-file containing the annotations.')
args = parser.parse_args()

if not os.path.isfile(args.path):
    print("The given annotation file does not exist.\nSee annotateVisually.py -h for more info.")
    exit()

csv = open(os.path.abspath(args.path), 'r')
csv.readline()  # Discard the header-line.
csv = csv.readlines()
csv.sort()

basePath = os.path.dirname(args.path)
savePath = os.path.join(basePath, 'images')
saveLabelPath = os.path.join(basePath, 'labels')
if not os.path.isdir(savePath):
    os.mkdir(savePath)
if not os.path.isdir(saveLabelPath):
    os.mkdir(saveLabelPath)

def convert(size, box):
    dw = 1./size[0]
    dh = 1./size[1]
    x = (box[0] + box[1])/2.0
    y = (box[2] + box[3])/2.0
    w = box[1] - box[0]
    h = box[3] - box[2]
    x = x*dw
    w = w*dw
    y = y*dh
    h = h*dh
    return (x,y,w,h)

unique_names = {}

def cls_name_to_id(name):
    name = name.lower()
    unique_names[name] = 1
    if "stop"==name:
        return 0
    if "yield"==name:
        return 1
    return -1

def refined_im_name(name):
    sp = name.split(".")
    suffix = sp[-1]
    sp = sp[:-1]
    return "_".join(sp) + "." + suffix

imids = {}

for line in csv:
    fields = line.split(";")
    #0 vid1/frameAnnotations-vid_cmp1.avi_annotations/yield_1323813350.avi_image2.png
    #1 yield
    #2 562, upper left X
    #3 205, upper left Y
    #4 592, lower right X
    #5 229, lower right Y
    #6 0,0
    #7 vid1/vid_cmp1.avi
    #8 8764
    #9 yield_1323813350.avi
    #10 12

    imname = fields[0].split("/")[-1]
    imname = refined_im_name(imname)
    imid = imname.split(".")[0]

    clsid = cls_name_to_id(fields[1])
    if clsid < 0:
        continue

    thisImage = os.path.join(basePath, fields[0])
    im = Image.open(thisImage)
    im.save(os.path.join(savePath, imid+".jpg"))

    std = convert(im.size, [int(fields[2]), int(fields[4]), int(fields[3]), int(fields[5])])

    with open(os.path.join(saveLabelPath, imid+".txt"), 'a') as out_file:
        out_file.write(str(clsid) + " " + " ".join([str(a) for a in std]) + '\n')

    imids[imid] = 1

train_portion = 0.8
random.seed(1)

with open(os.path.join(basePath, "train.txt"), "w") as ftrain:
    with open(os.path.join(basePath, "val.txt"), "w") as fval:
        for imid in sorted(imids.keys()):
            tpath = os.path.join(savePath, imid+".jpg")

            if random.random() < train_portion:
                ftrain.write(tpath+"\n")
            else:
                fval.write(tpath + "\n")
# shuffle the index and partition into training and validation set

print(unique_names.keys())
