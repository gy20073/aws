import numpy as np
import time, os, inspect, sys
sys.path.append("../")
from common import resize_images
from functools import partial

import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
from PIL import Image, ImagePalette

import models
from modules.bn import InPlaceABN
from modules.deeplab import DeeplabV3


class SegmentationModule(nn.Module):
    def __init__(self, body, head, head_channels, classes):
        super(SegmentationModule, self).__init__()
        self.body = body
        self.head = head
        self.cls = nn.Conv2d(head_channels, classes, 1)

    def _network(self, x):
        x_up = x

        x_up = self.body(x_up)
        x_up = self.head(x_up)
        sem_logits = self.cls(x_up)

        del x_up
        return sem_logits

    def forward(self, x):
        sem_logits = self._network(x)
        return sem_logits


class SegmenterABN:
    def __init__(self,
                 model_path,
                 GPU="0",
                 compute_method="compute_logits",
                 viz_method="visualize_logits",
                 batch_size=1,
                 output_downsample_factor=4,
                 input_downsample=1):
        # Torch stuff
        torch.cuda.set_device(int(GPU))
        cudnn.benchmark = True

        # Create model by loading a snapshot
        body, head, cls_state = load_snapshot(model_path)
        model = SegmentationModule(body, head, 256, 65)
        model.cls.load_state_dict(cls_state)
        model = model.cuda().eval()
        self.model = model
        print(model)
        # end of new code

        self.compute_method = compute_method
        self.viz_method = viz_method
        self.height = 576//input_downsample
        self.width = 768//input_downsample
        # TODO: see if we need to downsample it depending on the output size
        self.output_downsample_factor = output_downsample_factor

    def compute(self, image):
        return getattr(self, self.compute_method)(image)

    def visualize(self, pred, ibatch):
        return getattr(self, self.viz_method)(pred, ibatch)

    def compute_logits(self, image):
        # TODO pin_memory=True,
        rgb_mean = [0.41738699, 0.45732192, 0.46886091]
        rgb_std = [0.25685097, 0.26509955, 0.29067996]

        image = resize_images(image, [self.height, self.width])
        # image shape B*H*W*C
        with torch.no_grad():
            img = torch.from_numpy(image)
            img = img.cuda(non_blocking=True)
            # from B H W C to B C H W
            img = img.permute(0, 3, 1, 2)
            img = img.float()
            # from a int tensor to a float tensor
            img = img / 255.0

            # normalize it
            img.sub_(img.new(rgb_mean).view(1, -1, 1, 1))
            img.div_(img.new(rgb_std).view(1, -1, 1, 1))

            sem_logits = self.model(img)
            # this logits has size of batch*nclass*H*W

            # compute the visualization within cuda
            max_value, argmax = sem_logits.max(1)
            # the output has size of batch*H*W
            self.argmax = argmax.cpu().numpy()
            sem_logits = sem_logits.permute(0, 2, 3, 1)
            sem_logits = sem_logits.cpu().numpy()

        # out has shape batch * nclass * H * W: [1, 65, 72, 96]
        return sem_logits

    def visualize_logits(self, pred, ibatch):
        tensor = self.argmax[ibatch]
        img = Image.fromarray(tensor.astype(np.uint8), mode="P")
        img.putpalette(_PALETTE)
        img = img.convert("RGB")
        out = np.array(img)
        return out

def get_current_folder():
    return os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

def load_snapshot(snapshot_file):
    """Load a training snapshot"""
    print("--- Loading model from snapshot")

    # Create network
    norm_act = partial(InPlaceABN, activation="leaky_relu", slope=.01)
    body = models.__dict__["net_wider_resnet38_a2"](norm_act=norm_act, dilation=(1, 2, 4, 4))
    head = DeeplabV3(4096, 256, 256, norm_act=norm_act, pooling_size=(84, 84))

    # Load snapshot and recover network state
    data = torch.load(snapshot_file)
    body.load_state_dict(data["state_dict"]["body"])
    head.load_state_dict(data["state_dict"]["head"])

    return body, head, data["state_dict"]["cls"]



_PALETTE = np.array([[165, 42, 42],
                     [0, 192, 0],
                     [196, 196, 196],
                     [190, 153, 153],
                     [180, 165, 180],
                     [90, 120, 150],
                     [102, 102, 156],
                     [128, 64, 255],
                     [140, 140, 200],
                     [170, 170, 170],
                     [250, 170, 160],
                     [96, 96, 96],
                     [230, 150, 140],
                     [128, 64, 128],
                     [110, 110, 110],
                     [244, 35, 232],
                     [150, 100, 100],
                     [70, 70, 70],
                     [150, 120, 90],
                     [220, 20, 60],
                     [255, 0, 0],
                     [255, 0, 100],
                     [255, 0, 200],
                     [200, 128, 128],
                     [255, 255, 255],
                     [64, 170, 64],
                     [230, 160, 50],
                     [70, 130, 180],
                     [190, 255, 255],
                     [152, 251, 152],
                     [107, 142, 35],
                     [0, 170, 30],
                     [255, 255, 128],
                     [250, 0, 30],
                     [100, 140, 180],
                     [220, 220, 220],
                     [220, 128, 128],
                     [222, 40, 40],
                     [100, 170, 30],
                     [40, 40, 40],
                     [33, 33, 33],
                     [100, 128, 160],
                     [142, 0, 0],
                     [70, 100, 150],
                     [210, 170, 100],
                     [153, 153, 153],
                     [128, 128, 128],
                     [0, 0, 80],
                     [250, 170, 30],
                     [192, 192, 192],
                     [220, 220, 0],
                     [140, 140, 20],
                     [119, 11, 32],
                     [150, 0, 255],
                     [0, 60, 100],
                     [0, 0, 142],
                     [0, 0, 90],
                     [0, 0, 230],
                     [0, 80, 100],
                     [128, 64, 64],
                     [0, 0, 110],
                     [0, 0, 70],
                     [0, 0, 192],
                     [32, 32, 32],
                     [120, 10, 10]], dtype=np.uint8)
_PALETTE = np.concatenate([_PALETTE, np.zeros((256 - _PALETTE.shape[0], 3), dtype=np.uint8)], axis=0)
_PALETTE = ImagePalette.ImagePalette(
    palette=list(_PALETTE[:, 0]) + list(_PALETTE[:, 1]) + list(_PALETTE[:, 2]), mode="RGB")

if __name__ == "__main__":
    # TODO: rewrite the testing code

    # path = "/home/gaoyang1/data/CityScapes/leftImg8bit/val/munster/munster_000115_000019_leftImg8bit.png"
    # path = "/scratch/yang/aws_data/bdd100k/yolo_format/images/val/c8620a67-55f86ae2.jpg"

    seg = SegmenterABN(model_path="/scratch/yang/aws_data/mapillary/inplace_abn/wide_resnet38_deeplab_vistas.pth.tar",
                    GPU="0",
                    compute_method="compute_logits",
                    viz_method="visualize_logits")

    paths = ["/scratch/yang/aws_data/mapillary/validation/images/0daE8mWxlKFT8kLBE5f12w.jpg"]
    start = time.time()
    print("number of images:", len(paths))

    for path in paths:
        id = path.split("/")[-1].split(".")[0]
        ori = Image.open(path)
        img = ori

        img = np.array(img)
        img = np.expand_dims(img, 0)

        img = np.concatenate([img]*1, 0)
        for i in range(10):
            pred = seg.compute(img)
        print(np.max(pred), np.min(pred), np.mean(pred), np.median(pred))

        colored = seg.visualize(pred, 3)
        colored = Image.fromarray(colored)

        colored.save(id + "-seg.png")
        ori.save(id+"-original.jpg")

    print("elapsed time:", time.time() - start)

