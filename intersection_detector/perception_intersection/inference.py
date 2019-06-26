from collections import OrderedDict
import re
import sys
sys.path.append('.')

import cv2
import torch
import torchvision.models as models
import perception_intersection.marvin.io as mio
import pdb

class IntersectionDetector():
    def __init__(self, model_path, image_mean, resolution=(227, 227), opts=None):
        self._model = None
        self._resolution = resolution
        self._means_marvin_tensor = mio.read_tensor(image_mean)[0].value
        self._opts = opts

        # default initialization
        if self._opts is None:
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

    def __call__(self, image):
        [H, W, C] = image.shape

        assert H >= self._resolution[0]
        assert W >= self._resolution[1]

        # get central crop of given resolution
        h0 = int(0.5 * (H - self._resolution[0]))
        hN = int(0.5 * (H + self._resolution[0]))
        w0 = int(0.5 * (W - self._resolution[1]))
        wN = int(0.5 * (W + self._resolution[1]))

        center_crop = image[h0:hN, w0:wN, :]
        center_crop = center_crop.transpose(2, 0, 1)
        center_crop = (center_crop - self._means_marvin_tensor) / 127.0
        center_crop = torch.from_numpy(center_crop).type(torch.FloatTensor).unsqueeze(0)
        input = center_crop.cuda(self._opts['gpu'], non_blocking=True)

        output = self._model(input)
        logits = torch.nn.functional.sigmoid(output)

        return logits

if __name__ == '__main__':
    model_path = '/home/grossanc/Projects/Intersections/models/model_74.pth'
    image_mean_path = '/media/grossanc/5ee96718-a500-4356-988b-9f6a66773bff/inter_detection/train_mean_42140.tensor'
    resolution = (227, 227)
    opts = {'gpu': 0}
    test_image_path = '/home/grossanc/Pictures/Screenshot from 2018-10-10 13-51-49.png'

    test_image = cv2.imread(test_image_path, -1)
    test_image = cv2.cvtColor(test_image, cv2.COLOR_BGR2RGB)

    detector = IntersectionDetector(model_path, image_mean_path, resolution, opts)
    classi = detector(test_image)

    pdb.set_trace()
    pass
