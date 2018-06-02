import os

# for each label file, check whether stop sign in it.
# if do, then create new label file with only stop sign, in label dir, and add an entry of this image in the index file

subset="train2014"
label_path = "/scratch/yang/aws_data/coco/labels_bak/" + subset
out_path = "/scratch/yang/aws_data/coco/labels/" + subset
image_prefix = "/scratch/yang/aws_data/coco/images/" + subset
index_file = "/scratch/yang/aws_data/coco/filtered_" + subset + ".txt"

if not os.path.exists(out_path):
    os.mkdir(out_path)

# 11 is stop sign
def filter_stop_sign(fname):
    with open(fname, "r") as f:
        lines = f.readlines()
    out = []
    for line in lines:
        if line.startswith("11 "):
            out.append("0 " + line[3:])
    return out

def write_label(oname, filtered):
    with open(oname, "w") as f:
        for l in filtered:
            f.write(l)

index = open(index_file, "w")

for file in os.listdir(label_path):
    if file.endswith(".txt"):
        filtered = filter_stop_sign(os.path.join(label_path, file))
        if len(filtered) > 0:
            # save the label
            write_label(os.path.join(out_path, file), filtered)
            # save the image name
            index.write(os.path.join(image_prefix, file.replace(".txt", ".jpg")) + "\n")

index.close()