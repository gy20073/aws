import os, sys, scipy.misc
os.environ['TF_CPP_MIN_LOG_LEVEL']='0'
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from PIL import Image
from common import resize_images

# some default params: encoder=vgg, checkpoint=select one, height=256, width=512

class Depth:
    def __init__(self,
                 model_path, # the path to a pretrained model, use city2eigen
                 python_path, # the path to the monodepth codebase
                 GPU="0",
                 compute_method="compute_disparity_average",
                 viz_method="visualize_disparity_average",
                 batch_size=1):
        os.environ["CUDA_VISIBLE_DEVICES"] = GPU
        sys.path.append(python_path)
        from monodepth_model import monodepth_parameters, MonodepthModel

        # parameters that I don't plan to change, hiding from the caller
        ENCODER="vgg"
        # if we are not going to change the model, the input size is also fixed
        self.height=256
        self.width =512
        self.batch_size = batch_size
        params = monodepth_parameters(
            encoder=ENCODER,
            height=self.height,
            width=self.width,
            batch_size=2*batch_size, # since we are doing averaging by flipping
            num_threads=1,
            num_epochs=1,
            do_stereo=False,
            wrap_mode="border",
            use_deconv=False,
            alpha_image_loss=0,
            disp_gradient_loss_weight=0,
            lr_loss_weight=0,
            full_summary=False)

        left = tf.placeholder(tf.float32, [2*batch_size, self.height, self.width, 3])
        model = MonodepthModel(params, "test", left, None)

        # SESSION
        config = tf.ConfigProto(allow_soft_placement=True)
        #config.gpu_options.allow_growth = True
        # If error arises then change the following param
        config.gpu_options.per_process_gpu_memory_fraction = 0.2 + 0.005*batch_size
        sess = tf.Session(config=config)

        # SAVER
        train_saver = tf.train.Saver()

        # INIT
        sess.run(tf.global_variables_initializer())
        sess.run(tf.local_variables_initializer())

        # RESTORE
        restore_path = model_path.split(".")[0]
        train_saver.restore(sess, restore_path)

        self.sess = sess
        self.model = model
        self.left = left

        self.compute_method = compute_method
        self.viz_method = viz_method

        # some visualization functions
        self.cm_hot = plt.cm.get_cmap('plasma')
        self.norm = plt.Normalize()

    def compute(self, image):
        return getattr(self, self.compute_method)(image)

    def visualize(self, pred, ibatch):
        return getattr(self, self.viz_method)(pred, ibatch)

    def compute_disparity_average(self, images):
        # assume a RGB input image
        input_image = resize_images(images, [self.height, self.width])
        input_image = np.concatenate((input_image, input_image[:, :, ::-1, :]), axis=0)
        input_image = input_image.astype(np.float32) / 255

        disp = self.sess.run(self.model.disp_left_est[0],
                             feed_dict={self.left: input_image})
        disp_pp = self.post_process_disparity(disp.squeeze()).astype(np.float32)
        # convert w*h image to batch*w*h*1
        disp_pp = np.expand_dims(disp_pp, axis=3)
        return disp_pp

    def visualize_disparity_average(self, pred, ibatch):
        pred = pred[ibatch, :, :, :]
        pred = pred.squeeze()

        pred = self.norm(pred)
        im = self.cm_hot(pred)
        im = np.uint8(im*255)

        # convert RGBA to RGB
        im = Image.fromarray(im)
        background = Image.new("RGB", im.size, (255, 255, 255))
        background.paste(im, mask=im.split()[3])  # 3 is the alpha channel

        return np.array(background)

    def post_process_disparity(self, disp):
        batchX2, h, w = disp.shape
        assert(2*self.batch_size == batchX2)
        l_disp = disp[0:self.batch_size, :, :]
        r_disp = disp[self.batch_size:, :, ::-1]
        m_disp = 0.5 * (l_disp + r_disp)
        l, _ = np.meshgrid(np.linspace(0, 1, w), np.linspace(0, 1, h))
        # l has shape H*W
        l_mask = 1.0 - np.clip(20 * (l - 0.05), 0, 1)
        r_mask = np.fliplr(l_mask)

        l_mask = np.expand_dims(l_mask, axis=0)
        r_mask = np.expand_dims(r_mask, axis=0)
        return r_mask * l_disp + l_mask * r_disp + (1.0 - l_mask - r_mask) * m_disp
