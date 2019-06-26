# for each json file, parse all objects, if no traffic light, continue
# the retained TL objects, write the label to a text file, store the image path in a list
# write out the image names list

import os, json, shutil

subset="train" # train / test
label_path = "/data/yang/code/aws/scratch/bdd100k/labels/100k/" + subset
out_path = "/data/yang/code/aws/scratch/bdd100k/yolo_format/labels/" + subset
input_image_prefix = "/scratch/yang/aws_data/bdd100k/images/100k/" + subset
output_image_prefix = "/data/yang/code/aws/scratch/bdd100k/yolo_format/images/" + subset
index_file = "/data/yang/code/aws/scratch/bdd100k/yolo_format/filtered_" + subset + ".txt"

if not os.path.exists(out_path):
    os.mkdir(out_path)
if not os.path.exists(output_image_prefix):
    os.mkdir(output_image_prefix)

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

def json_to_yolo(obj):
    color = obj["attributes"]["trafficLightColor"]
    box2d = obj["box2d"]
    mapping={"red": 0,
             "green": 1,
             "yellow": 2}
    color = mapping[color]

    width = 1280.0
    height = 720.0
    # x <--> width, y <--> height
    out = convert((width, height), [box2d["x1"], box2d["x2"], box2d["y1"], box2d["y2"]])
    return color, out

def filter_stop_sign(fname):
    with open(fname, "r") as f:
        j = json.load(f)

    out = []
    for obj in j["frames"][0]["objects"]:
        # each obj is a dict
        if obj["category"] == "traffic light":
            if not(obj["attributes"]["trafficLightColor"] == "none"):
                color, box = json_to_yolo(obj)
                if box[2] > 10.0 /1280 and box[3] > 10.0 / 720:
                    # filter out the super small detections
                    out.append((color, box))
    return out

def write_label(oname, filtered):
    with open(oname, "w") as f:
        for l in filtered:
            f.write(str(l[0]) + " " + " ".join([str(a) for a in l[1]]) + '\n')

index = open(index_file, "w")

count = 0
for file in os.listdir(label_path):
    count = count+1
    if count % 1000 == 0:
        print(count)
    if file.endswith(".json"):
        filtered = filter_stop_sign(os.path.join(label_path, file))
        if len(filtered) > 0:
            imname = file.replace(".json", ".jpg")

            # save the label
            write_label(os.path.join(out_path, file.replace(".json", ".txt")), filtered)
            # copy the image
            shutil.copy(os.path.join(input_image_prefix,  imname),
                        os.path.join(output_image_prefix, imname))
            # save the image name
            index.write(os.path.join(output_image_prefix, imname) + "\n")

index.close()
