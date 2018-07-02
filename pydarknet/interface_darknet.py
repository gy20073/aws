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


    def compute(self, image):
        return getattr(self, self.compute_method)(image)

    def visualize(self, pred):
        return getattr(self, self.viz_method)(pred)


    def compute_logits(self, image):
        image = scipy.misc.imresize(image, [self.height, self.width], interp='bilinear')
        dark_frame = Image(image)
        detections, logits = self.net.detect(dark_frame)
        del dark_frame

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
            out = scipy.ndimage.zoom(l, (max_h/l.shape[0], max_w/l.shape[1], 1), order=0)
            resized.append(out)
        concat = np.concatenate(resized, axis=2)

        return {"logit":concat, "detection":detections, "image": image}

    def visualize_logits(self, pred):
        image = pred["image"]
        detections = pred["detection"]
        output = np.copy(image)
        for cat, score, bounds in detections:
            x, y, w, h = bounds
            cv2.rectangle(output, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 0))
            cv2.putText(output, str(cat.decode("utf-8")), (int(x), int(y)), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 0))

        return output

if __name__ == "__main__":
    im = cv2.imread("/scratch/yang/aws_data/mapillary/validation/images/0daE8mWxlKFT8kLBE5f12w.jpg")
    detector = YoloDetector(path_cfg="/data/yang/code/aws/coco_original/yolov3.cfg",
                            path_weights="/data/yang/code/aws/data/yolov3.weights",
                            path_meta="/data/yang/code/aws/coco_original/coco.data")

    pred = detector.compute(im)
    viz = detector.visualize(pred)
    cv2.imwrite("output.png", viz)
