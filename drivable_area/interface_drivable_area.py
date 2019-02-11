import numpy as np
import os
from common import resize_images

import torch
import torch.backends.cudnn as cudnn
from torch.autograd import Variable
from segment import DRNSeg


class DrivableArea:
    def __init__(self,
                 model_path,
                 mean=[0.2741781117493873, 0.2896292359749455, 0.28759914979064544], #[0.3930478 , 0.44596469, 0.52638272], #
                 std=[0.2454775432519874, 0.26320519893368627, 0.2730747230872623], #[0.23479487, 0.22210911, 0.24706927], #
                 GPU="0",
                 batch_size=1,
                 output_downsample_factor=4,
                 compute_method="compute_logits",
                 viz_method="visualize_logits"):
        self.batch_size = batch_size
        self.output_downsample_factor = output_downsample_factor
        self.height = 576
        self.width = 768
        self.mean = np.reshape(mean, (1, 1, 1, 3))
        self.std = np.reshape(std, (1, 1, 1, 3))
        self.compute_method = compute_method
        self.viz_method = viz_method
        os.environ['CUDA_VISIBLE_DEVICES'] = GPU

        single_model = DRNSeg("drn_d_22", 3, pretrained_model=None, pretrained=False)
        single_model.load_state_dict(torch.load(model_path))
        self.model = torch.nn.DataParallel(single_model).cuda()
        self.model.eval()
        cudnn.benchmark = True

    def compute(self, image):
        return getattr(self, self.compute_method)(image)

    def visualize(self, pred, ibatch):
        return getattr(self, self.viz_method)(pred, ibatch)

    def compute_logits(self, image):
        image = resize_images(image, [self.height, self.width])
        # normalize with mean and std
        image = (image / 255.0 - self.mean) / self.std
        image = image.astype(np.float32)
        image = np.transpose(image, [0, 3, 1, 2])
        image = torch.from_numpy(image)

        image_var = Variable(image, requires_grad=False, volatile=True)
        pred = self.model(image_var)[0]
        pred = pred[:, :, ::self.output_downsample_factor, ::self.output_downsample_factor]
        # _, pred = torch.max(final, 1)
        pred = pred.permute([0, 2, 3, 1])
        pred = pred.cpu().data.numpy()
        return pred

    def visualize_logits(self, pred, ibatch):
        argmax = np.argmax(pred, axis=3)
        out = self.visualize_index(argmax, ibatch)

        return out

    def visualize_index(self, pred, ibatch):
        pred = pred[ibatch]
        # 19*3
        # color coding from here:
        # https://github.com/mcordts/cityscapesScripts/blob/master/cityscapesscripts/helpers/labels.py
        color = np.asarray([
            [0, 0, 0],
            [217, 83, 79],
            [91, 192, 222]], dtype=np.uint8)

        shape = pred.shape

        pred = pred.ravel()
        pred = np.array([color[pred, 0], color[pred, 1], color[pred, 2]])
        pred = np.transpose(pred)
        pred = pred.reshape(shape[0], shape[1], 3)

        return pred.astype(np.uint8)