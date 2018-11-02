import numpy as np
import time, os, inspect, sys
sys.path.append("../")
from common import resize_images
from collections import OrderedDict

import cv2
import torch
import torchvision.models as models
import perception_intersection.marvin.io as mio

from PIL import Image, ImageDraw, ImageFont

def write_text_on_image(image, string, fontsize=10):
    image = image.copy()
    image = np.uint8(image)
    j = Image.fromarray(image)
    draw = ImageDraw.Draw(j)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", fontsize)
    draw.text((0, 0), string, (255, 0, 0), font=font)

    return np.array(j)

class Intersection:
    def __init__(self,
                 model_path,
                 mean_path,
                 GPU="0",
                 compute_method="compute_logits",
                 viz_method="visualize_logits",
                 batch_size=1):
        os.environ["CUDA_VISIBLE_DEVICES"] = GPU
        self.compute_method = compute_method
        self.viz_method = viz_method

        self._model = None
        self._resolution = [227, 227]
        self._means_marvin_tensor = mio.read_tensor(mean_path)[0].value

        self._opts = {'gpu': 0}

        self._model = models.__dict__['alexnet'](pretrained=False, num_classes=1)
        self._model.cuda(self._opts['gpu'])
        self._model.eval()

        print('>>> Loading model: {}'.format(model_path))
        checkpoint = torch.load(model_path)

        # "features.0.weight",
        # "features.0.bias",
        # "features.3.weight", "features.3.bias", "features.6.weight",
        # "features.6.bias", "features.8.weight", "features.8.bias",
        # "features.10.weight", "features.10.bias", "classifier.1.weight",
        # "classifier.1.bias", "classifier.4.weight", "classifier.4.bias",
        # "classifier.6.weight", "classifier.6.bias".
        new_state_dict = OrderedDict()
        for k, v in checkpoint['state_dict'].items():
            name = k.replace('.module', '')
            new_state_dict[name] = v

        self._model.load_state_dict(new_state_dict)
        print('>>> Done! ')

    def compute(self, image):
        return getattr(self, self.compute_method)(image)

    def visualize(self, pred, ibatch):
        return getattr(self, self.viz_method)(pred, ibatch)

    def compute_logits(self, images):
        # might be zoomed images
        #images = resize_images(images, [512*3//4, 512])
        images = resize_images(images, [400*3//4, 400])

        [N, H, W, C] = images.shape

        assert H >= self._resolution[0]
        assert W >= self._resolution[1]

        # get central crop of given resolution
        h0 = int(0.5 * (H - self._resolution[0]))
        hN = int(0.5 * (H + self._resolution[0]))
        w0 = int(0.5 * (W - self._resolution[1]))
        wN = int(0.5 * (W + self._resolution[1]))

        center_crop = images[:, h0:hN, w0:wN, :]
        self.last_images = center_crop
        center_crop = center_crop.transpose(0, 3, 1, 2)
        center_crop = (center_crop - self._means_marvin_tensor) / 127.0
        center_crop = torch.from_numpy(center_crop).type(torch.FloatTensor)
        input = center_crop.cuda(self._opts['gpu'], non_blocking=True)

        logits = self._model(input)
        logits = logits.squeeze()
        #print("logits shape", logits.size()) # size is batch*1
        #logits = torch.nn.functional.sigmoid(logits)

        return logits

    def visualize_logits(self, pred, ibatch):
        image = self.last_images[ibatch]
        this_logit = pred[ibatch]
        prob = torch.nn.functional.sigmoid(this_logit)
        value = np.squeeze(prob.cpu().detach().numpy())
        image = write_text_on_image(image, "prob: %.2f" % (value, ), 20)
        return image



def get_current_folder():
    return os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

if __name__ == "__main__":
    # TODO: rewrite the testing code

    # path = "/home/gaoyang1/data/CityScapes/leftImg8bit/val/munster/munster_000115_000019_leftImg8bit.png"
    # path = "/scratch/yang/aws_data/bdd100k/yolo_format/images/val/c8620a67-55f86ae2.jpg"

    seg = Intersection("/scratch/yang/aws_data/intersection_detection/model_74.pth",
                       "/scratch/yang/aws_data/intersection_detection/train_mean_42140.tensor",
                       GPU="0",)

    paths = ["/scratch/yang/aws_data/mapillary/validation/images/0daE8mWxlKFT8kLBE5f12w.jpg"]
    start = time.time()
    print("number of images:", len(paths))

    for path in paths:
        id = path.split("/")[-1].split(".")[0]
        ori = Image.open(path)
        img = ori

        img = np.array(img)
        img = np.expand_dims(img, 0)

        img = np.concatenate([img]*4, 0)
        for i in range(10):
            pred = seg.compute(img)
        print(pred)
        #print(np.max(pred), np.min(pred), np.mean(pred), np.median(pred))

        colored = seg.visualize(pred, 3)
        colored = Image.fromarray(colored)

        colored.save(id + "-seg.png")
        ori.save(id+"-original.jpg")

    print("elapsed time:", time.time() - start)

