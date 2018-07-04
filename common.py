import numpy as np
import scipy.misc, multiprocessing, time, cv2

def resize_images(images, new_size):
    if images.shape[1] != new_size[0] or images.shape[2] != new_size[1]:
        output = np.zeros((images.shape[0], new_size[0], new_size[1], images.shape[3]), dtype=np.uint8)

        for i in range(images.shape[0]):
            if True:
                cv2.resize(images[i, :, :, :],
                           dsize=(new_size[1], new_size[0]),
                           dst=output[i, :, :, :],
                           interpolation=cv2.INTER_LINEAR)
            else:
                # do not use scipy image resize, it's too slow
                output[i, :, :, :] = scipy.misc.imresize(images[i, :, :, :], new_size, interp='bilinear')
        return output
    else:
        return images


class parallel_imresize:
    # TODO: not tested, seems unecessary with cv2.resize, that's fast enough
    def __init__(self, num_worker):
        self.num_worker = num_worker
        self.pool = multiprocessing.Pool(processes=num_worker)

    @staticmethod
    def worker(inputs, outputs, range, new_size):
        for i in range:
            outputs[i, :, :, :] = scipy.misc.imresize(inputs[i, :, :, :], new_size, interp='bilinear')

    def resize(self, images, new_size):
        output = np.zeros((images.shape[0], new_size[0], new_size[1], images.shape[3]), dtype=np.uint8)
        images = multiprocessing.Array('B', images, lock=False)
        output = multiprocessing.Array('B', output, lock=False)

        assert (images.shape[0] % self.num_worker == 0)
        work_amount = images.shape[0] // self.num_worker
        res = []
        for i in range(self.num_worker):
            out = self.pool.apply_async(self.worker,
                                        (images, output, range(work_amount * i, work_amount * (i + 1)), new_size))
            res.append(out)
        for r in res:
            r.wait()
        return output


if __name__ == "__main__":
    '''
    # test the parallel image resizing mechanism
    images = np.ones((32, 2000, 2000, 3), dtype=np.uint8)
    new_size = (320, 320)
    start = time.time()
    for i in range(10):
        output=resize_images(images, new_size)
    print(output)
    print(time.time() - start)
    '''
    images = np.ones((32, 256, 512, 3), dtype=np.uint8)
    print(images.size/1024/1024)
    start = time.time()
    for i in range(100):
        t = images.astype(np.float32) * (1.0/255)
    print(time.time() - start)
