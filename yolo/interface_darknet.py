import cv2, os, math
import numpy as np
from pydarknet import Detector, Image
from common import resize_images
from subprocess import call


class YoloDetector:
    """
    Wrapper aroun YOLOv3 objects to perform detection
    """
    def __init__(self,
                 path_cfg,
                 path_weights,
                 path_meta,
                 GPU="0",
                 compute_method="compute_logit_list",
                 viz_method="visualize_logits",
                 batch_size=1):

        self.path_cfg = path_cfg
        self.path_weights = path_weights
        self.path_meta = path_meta

        os.environ["CUDA_VISIBLE_DEVICES"] = GPU
        self.compute_method = compute_method
        self.viz_method = viz_method

        self.batch_size=batch_size

        # has to replace the batch size
        self.path_cfg_batch = self.path_cfg + ".batch_temp" + str(batch_size)
        cmd = "sed 's/batch=1/batch="+str(batch_size)+"/g' < " + self.path_cfg + " > " + self.path_cfg_batch
        print(cmd)
        call(cmd, shell=True)

        '''
        # python3 syntax
        self.net = Detector(bytes(self.path_cfg_batch, encoding="utf-8"),
                            bytes(self.path_weights, encoding="utf-8"),
                            0,
                            bytes(self.path_meta, encoding="utf-8"))
        '''
        self.net = Detector(self.path_cfg_batch,
                            self.path_weights,
                            0,
                            self.path_meta)

        self.current_images = None


    def compute(self, image):
        return getattr(self, self.compute_method)(image)

    def visualize(self, pred, ibatch):
        return getattr(self, self.viz_method)(pred, ibatch)

    def _combine_logits(self, logits):
        # The shape of each logit is: (batch[i], n[i], classp5[i], h[i], w[i])
        # reshape them into batch*H*W*C
        max_h = -1
        max_w = -1
        transposed = []
        for l in logits:
            l = np.transpose(l, axes=(0, 3, 4, 1, 2))
            # has shape B*H*W*N*Cp5
            l = np.reshape(l, (l.shape[0], l.shape[1], l.shape[2], -1))
            max_h = max(max_h, l.shape[1])
            max_w = max(max_w, l.shape[2])
            transposed.append(l)

        resized = []
        for l in transposed:
            #out = scipy.ndimage.zoom(l, (max_h // l.shape[0], max_w // l.shape[1], 1), order=0)
            l = np.repeat(l, max_h // l.shape[1], axis=1)
            l = np.repeat(l, max_w // l.shape[2], axis=2)
            resized.append(l)
        concat = np.concatenate(resized, axis=3)
        return concat

    def _combine_logits2(self, logits):
        # The shape of each logit is: (batch[i], n[i], classp5[i], h[i], w[i])
        # reshape them into batch*H*W*C
        max_h = -1
        max_w = -1
        out = []
        for l in logits:
            l = np.reshape(l, (l.shape[0], l.shape[1]*l.shape[2], l.shape[3], l.shape[4]))
            max_h = max(max_h, l.shape[2])
            max_w = max(max_w, l.shape[3])
            l = np.transpose(l, (2, 3, 1, 0))
            # shape is H*W*cp5*B
            out.append(l)
        logits = out
        out = []
        for l in logits:
            l = np.repeat(l, max_w // l.shape[1], axis=1)
            l = np.repeat(l, max_h // l.shape[0], axis=0)
            out.append(l)
        concat = np.concatenate(out, axis=2)
        # H W cp5 B
        concat = np.transpose(concat, axes=(3, 0, 1, 2))

        return concat


    def compute_logits(self, images):
        logits = self.compute_logit_list(images)
        return self._combine_logits(logits)

    def darknet_preprocess(self, images):
        # resize the images to shorter edge be neth
        neth = netw = 416
        im_h = images.shape[1]
        im_w = images.shape[2]

        if (1.0 * netw / im_w) < (1.0 * neth / im_h):
            new_w = netw
            new_h = (im_h * netw) // im_w
        else:
            new_h = neth
            new_w = (im_w * neth) // im_h
        images = resize_images(images, [new_h, new_w])
        resized = images

        # BCHW format
        images = np.transpose(images, (0, 3, 1, 2))
        # RGB to BGR
        images = images[:, ::-1, :, :]
        # to float and to 0-1
        images = images / 255.0
        # as continuous memory
        images = np.ascontiguousarray(images, dtype=np.float32)
        dark_frames = Image(images)
        return dark_frames, resized, images

    def compute_logit_list(self, images):
        dark_frames, self.current_images, images = self.darknet_preprocess(images)
        self.net.forward(dark_frames, self.current_images.shape[0])
        if False:
            # has to keep this images pointer, otherwise it's recycled
            self.current_images = images

        logits = self.net.get_logits()

        return logits

    # Note that the visualize function is a stateful function
    # in the sense that it needs the state of self.net, and self.image

    def visualize_logits_general(self, pred, ibatch, thresh=.5, nms=.45):
        image = self.current_images[ibatch]
        detections = self.net.get_boxes(ibatch,
                                        thresh=thresh,
                                        hier_thresh=.5, # This value is not used for Yolo
                                        nms=nms)

        output = np.copy(image)
        for cat, score, bounds in detections:
            x, y, w, h = bounds
            if math.isinf(w) or math.isinf(h):
                print(bounds, "inf encountered, continue")
                raise
                continue
            cv2.rectangle(output, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 0))
            cv2.putText(output, str(cat.decode("utf-8")), (int(x), int(y)), cv2.FONT_HERSHEY_COMPLEX, 0.3, (255, 255, 0))

        return output

    def visualize_logits(self, pred, ibatch):
        return self.visualize_logits_general(pred, ibatch)

    def visualize_logits_low_thresh(self, pred, ibatch):
        return self.visualize_logits_general(pred, ibatch, thresh=0.1)

if __name__ == "__main__":
    im = cv2.imread("/scratch/yang/aws_data/mapillary/validation/images/0daE8mWxlKFT8kLBE5f12w.jpg")
    if False:
        detector = YoloDetector(path_cfg="/data/yang/code/aws/coco_original/yolov3.cfg",
                                path_weights="/data/yang/code/aws/data/yolov3.weights",
                                path_meta="/data/yang/code/aws/coco_original/coco.data")

    if False:
        detector = YoloDetector(path_cfg="/data/yang/code/aws/traffic_light/yolov3-TL.cfg.test",
                                path_weights="/scratch/yang/aws_data/bdd100k/yolo_format/backup/yolov3-TL.backup",
                                path_meta="/data/yang/code/aws/traffic_light/TL.data")

    if True:
        detector = YoloDetector(path_cfg="/data/yang/code/aws/traffic_sign/yolov3-CL.cfg.test",
                                path_weights="/scratch/yang/aws_data/coco_lisa_v2/backup/yolov3-CL.backup",
                                path_meta="/data/yang/code/aws/traffic_sign/CL.data")

    pred = detector.compute(im)
    viz = detector.visualize_logits_general(pred, thresh=0.01)
    cv2.imwrite("output.png", viz)
