import numpy as np
import os, sys
from PIL import Image

def voc_ap(rec, prec, use_07_metric=False):
    """ ap = voc_ap(rec, prec, [use_07_metric])
    Compute VOC AP given precision and recall.
    If use_07_metric is true, uses the
    VOC 07 11 point method (default:False).
    """
    if use_07_metric:
        # 11 point metric
        ap = 0.
        for t in np.arange(0., 1.1, 0.1):
            if np.sum(rec >= t) == 0:
                p = 0
            else:
                p = np.max(prec[rec >= t])
            ap = ap + p / 11.
    else:
        # correct AP calculation
        # first append sentinel values at the end
        mrec = np.concatenate(([0.], rec, [1.]))
        mpre = np.concatenate(([0.], prec, [0.]))

        # compute the precision envelope
        for i in range(mpre.size - 1, 0, -1):
            mpre[i - 1] = np.maximum(mpre[i - 1], mpre[i])

        # to calculate area under PR curve, look for points
        # where X axis (recall) changes value
        i = np.where(mrec[1:] != mrec[:-1])[0]

        # and sum (\Delta recall) * prec
        ap = np.sum((mrec[i + 1] - mrec[i]) * mpre[i + 1])
    return ap

def voc_eval(detfile, class_recs, ovthresh, use_07_metric):
    npos = 0
    for key in class_recs:
        npos += len(class_recs[key]["det"])

    # read dets
    with open(detfile, 'r') as f:
        lines = f.readlines()

    splitlines = [x.strip().split(' ') for x in lines]
    image_ids = [x[0] for x in splitlines]
    confidence = np.array([float(x[1]) for x in splitlines])
    BB = np.array([[float(z) for z in x[2:]] for x in splitlines])

    # sort by confidence
    sorted_ind = np.argsort(-confidence)
    sorted_scores = np.sort(-confidence)
    BB = BB[sorted_ind, :]
    image_ids = [image_ids[x] for x in sorted_ind]

    # go down dets and mark TPs and FPs
    nd = len(image_ids)
    tp = np.zeros(nd)
    fp = np.zeros(nd)
    for d in range(nd):
        R = class_recs[image_ids[d]]
        bb = BB[d, :].astype(float)
        ovmax = -np.inf
        BBGT = R['bbox'].astype(float)

        if BBGT.size > 0:
            # compute overlaps
            # intersection
            ixmin = np.maximum(BBGT[:, 0], bb[0])
            iymin = np.maximum(BBGT[:, 1], bb[1])
            ixmax = np.minimum(BBGT[:, 2], bb[2])
            iymax = np.minimum(BBGT[:, 3], bb[3])
            iw = np.maximum(ixmax - ixmin + 1., 0.)
            ih = np.maximum(iymax - iymin + 1., 0.)
            inters = iw * ih

            # union
            uni = ((bb[2] - bb[0] + 1.) * (bb[3] - bb[1] + 1.) +
                   (BBGT[:, 2] - BBGT[:, 0] + 1.) *
                   (BBGT[:, 3] - BBGT[:, 1] + 1.) - inters)

            overlaps = inters / uni
            ovmax = np.max(overlaps)
            jmax = np.argmax(overlaps)

        if ovmax > ovthresh:
            if not R['difficult'][jmax]:
                if not R['det'][jmax]:
                    tp[d] = 1.
                    R['det'][jmax] = 1
                else:
                    fp[d] = 1.
        else:
            fp[d] = 1.

    # compute precision recall
    fp = np.cumsum(fp)
    tp = np.cumsum(tp)
    rec = tp / float(npos)
    # avoid divide by zero in case the first detection matches a difficult
    # ground truth
    prec = tp / np.maximum(tp + fp, np.finfo(np.float64).eps)
    ap = voc_ap(rec, prec, use_07_metric)

    return rec, prec, ap

def yolo_format_to_voc(centerx, centery, bw, bh, w, h):
    xmin = max((centerx - bw / 2)*w + 1, 1)
    xmax = min((centerx + bw / 2)*w + 1, w)
    ymin = max((centery - bh / 2)*h + 1, 1)
    ymax = min((centery + bh / 2)*h + 1, h)

    return xmin, ymin, xmax, ymax

def read_yolo_gt(base, numClasses):
    out = [{} for _ in range(numClasses)]
    # out[classid, such as 0, 1][imageid, no extensions like jpg]["bbox", "difficult", "det"]

    for file in os.listdir(base):
        if file.endswith(".txt"):
            # prepare two path
            label_path = os.path.join(base, file)
            image_path = label_path.replace(".txt", ".jpg")
            image_path = image_path.replace("labels", "images")

            # get width and height
            im = Image.open(image_path)
            width, height = im.size

            this_im = [[] for _ in range(numClasses)]
            # this_im[classid][bboxid] = bbox_tuple

            # read the gt file
            with open(label_path, "r") as f:
                lines = f.readlines()
            for line in lines:
                line = line.strip()
                if len(line) > 0:
                    # each line is a detection gt
                    cls, centerx, centery, bw, bh = line.split(" ")
                    bbox = yolo_format_to_voc(float(centerx), float(centery), float(bw), float(bh), width, height)
                    cls = int(cls)
                    this_im[cls].append(bbox)

            # feed this_im into the output object
            imid = file[:-4]
            for i in range(numClasses):
                numobj = len(this_im[i])
                out[i][imid] = {"bbox": np.array(this_im[i]),
                                "difficult": [False for _ in range(numobj)],
                                "det": [False for _ in range(numobj)]}
    return out


if __name__ == "__main__":
    # config begin

    class_name_path = "/data/yang/code/aws/traffic_sign/CL.names"
    prediction_base = "/data/yang/code/aws/scratch/coco_lisa/results_coco/"
    gt_base = "/data/yang/code/aws/scratch/coco/labels/val2014/"
    # config end

    # config for original coco
    """
    with open("/home/yang/darknet/data/coco.names", "r") as f:
        classes = f.readlines()
    classes = [x.strip() for x in classes]
    prediction_base = "/home/yang/darknet/results/"
    gt_base = "/data/yang/code/aws/scratch/coco/labels/val2014/"
    # end of config
    """

    # reading config from command line
    class_name_path = sys.argv[1]
    prediction_base = sys.argv[2]
    gt_base = sys.argv[3]

    with open(class_name_path, "r") as f:
        classes = f.readlines()
    classes = [x.strip() for x in classes]

    use_07_metric = False
    all_gt = read_yolo_gt(gt_base, len(classes))

    aps = []
    for i, cls in enumerate(classes):
        filename = os.path.join(prediction_base, "comp4_det_test_"+cls+".txt")
        rec, prec, ap = voc_eval(
            filename, all_gt[i], ovthresh=0.5, use_07_metric=use_07_metric)
        print("Class=", cls,
              "AP=", "{:.3f}".format(ap),
              " Precision=", "{:.3f}".format(np.mean(prec)),
              " Recall=", "{:.3f}".format(np.mean(rec)))
        aps += [ap]

    print('Mean AP is {:.3f}'.format(np.mean(aps)))
