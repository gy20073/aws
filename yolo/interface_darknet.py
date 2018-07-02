import cv2, os, scipy.misc, scipy.ndimage
import numpy as np
from pydarknet import Detector, Image


class YoloDetector:
    """
    Wrapper aroun YOLOv3 objects to perform detection
    """
    def __init__(self,
                 path_cfg,
                 path_weights,
                 path_meta,
                 GPU="0",
                 compute_method="compute_logits",
                 viz_method="visualize_logits"):

        self.path_cfg = path_cfg
        self.path_weights = path_weights
        self.path_meta = path_meta

        os.environ["CUDA_VISIBLE_DEVICES"] = GPU
        self.compute_method = compute_method
        self.viz_method = viz_method

        self.width = 555
        self.height = 416

        self.net = Detector(bytes(self.path_cfg, encoding="utf-8"),
                            bytes(self.path_weights, encoding="utf-8"),
                            0,
                            bytes(self.path_meta, encoding="utf-8"))

        self.current_image = None


    def compute(self, image):
        return getattr(self, self.compute_method)(image)

    def visualize(self, pred):
        return getattr(self, self.viz_method)(pred)

    def _combine_logits(self, logits):
        # The shape of each logit is: (batch[i], n[i], classp5[i], h[i], w[i])
        # reshape them into H*W*C
        max_h = -1
        max_w = -1
        transposed = []
        for l in logits:
            l = np.transpose(l, axes=(3, 4, 0, 1, 2))
            l = np.reshape(l, (l.shape[0], l.shape[1], -1))
            max_h = max(max_h, l.shape[0])
            max_w = max(max_w, l.shape[1])
            transposed.append(l)

        resized = []
        for l in transposed:
            #out = scipy.ndimage.zoom(l, (max_h // l.shape[0], max_w // l.shape[1], 1), order=0)
            l = np.repeat(l, max_h // l.shape[0], axis=0)
            l = np.repeat(l, max_w // l.shape[1], axis=1)
            resized.append(l)
        concat = np.concatenate(resized, axis=2)
        return concat

    def compute_logits(self, image):
        image = scipy.misc.imresize(image, [self.height, self.width], interp='bilinear')
        dark_frame = Image(image)
        self.net.forward(dark_frame)
        del dark_frame

        logits = self.net.get_logits()
        concat = self._combine_logits(logits)

        self.current_image = image
        return concat

    # Note that the visualize function is a stateful function
    # in the sense that it needs the state of self.net, and self.image
    def visualize_logits_general(self, pred, thresh=.5, nms=.45):
        image = self.current_image
        detections = self.net.get_boxes(thresh=thresh,
                                        hier_thresh=.5, # This value is not used for Yolo
                                        nms=nms)

        output = np.copy(image)
        for cat, score, bounds in detections:
            x, y, w, h = bounds
            cv2.rectangle(output, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 0))
            cv2.putText(output, str(cat.decode("utf-8")), (int(x), int(y)), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 0))

        return output

    def visualize_logits(self, pred):
        return self.visualize_logits_general(pred)

    def visualize_logits_low_thresh(self, pred):
        return self.visualize_logits_general(pred, thresh=0.1)

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
