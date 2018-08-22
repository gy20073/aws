import h5py, cv2, sys, pickle, math, numpy
import numpy as np

sys.path.append("/data/yang/code/aws")
from all_perceptions import Perceptions

class SegPerformance():
    def __init__(self):
        self.p = Perceptions(det_COCO=False,
                        det_TL=False,
                        det_TS=False,
                        seg=True,
                        depth=False,
                        batch_size=1,  # batch_size could also be a dict
                        gpu_assignment=[0],
                        compute_methods={},
                        viz_methods={},
                        num_replicates={},
                        path_config="path_jormungandr")
        self.accu = np.zeros((6, 2), dtype=np.int64)

    def _compute_inter_union(self, gt, pred, nclasses):
        # return a nclass * 2 matrix: [iclass][inter, union]
        out = np.zeros((nclasses, 2), dtype=np.int64)
        pred = pred[gt != 0] - 1
        gt = gt[gt != 0] - 1
        for i in range(nclasses):
            out[i, 0] = np.sum(np.logical_and(pred == i, gt == i))
            out[i, 1] = np.sum(np.logical_or(pred == i, gt == i))
        return out

    def reset(self):
        self.accu = np.zeros((6, 2), dtype=np.int64)

    def one_pair(self, image, seg):
        '''
        0	None
        1	Buildings
        2	Fences
        3	Other
        4	Pedestrians
        5	Poles
        6	RoadLines
        7	Roads
        8	Sidewalks
        9	Vegetation
        10	Vehicles
        11	Walls
        12	TrafficSigns

        # 0          1            2           3               4               5      6
        {'Ignored', 'Movable', 'Navigable', 'NoneNavigable', 'StaticLayout', 'Sky', 'Lane'}
        '''

        image_batch = np.expand_dims(image, 0)
        out = self.p.compute(image_batch)
        out = np.squeeze(out['seg'])
        out = np.argmax(out, axis=2) + 1  # 0 does not exists, since it is the ignore class underhood
        out = cv2.resize(out, (out.shape[1] * 4, out.shape[0] * 4), interpolation=cv2.INTER_NEAREST)

        mapping = np.array([0, 4, 3, 0, 1, 4, 6, 2, 3, 4, 1, 4, 4])
        seg_cvt = mapping[seg]

        self.accu += self._compute_inter_union(seg_cvt, out, 6)

    def summarize(self):
        names = ['Movable', 'Navigable', 'NoneNavigable', 'StaticLayout', 'Sky', 'Lane']
        out = {}
        for i in range(len(names)):
            out[names[i]] = 1.0 * self.accu[i, 0] / self.accu[i, 1]
        return out

class DepthPerformance():
    def __init__(self):
        width = 768
        fov = 90.0
        self.focal = width / (2.0 * math.tan(fov * math.pi / 360.0))
        self.baseline = 0.54

        self.p = Perceptions(det_COCO=False,
                             det_TL=False,
                             det_TS=False,
                             seg=False,
                             depth=True,
                             batch_size=1,  # batch_size could also be a dict
                             gpu_assignment=[0],
                             compute_methods={},
                             viz_methods={},
                             num_replicates={},
                             path_config="path_jormungandr")

        self.reset()

    def _compute_errors(self, gt, pred):
        thresh = np.maximum((gt / pred), (pred / gt))
        a1 = (thresh < 1.25).mean()
        a2 = (thresh < 1.25 ** 2).mean()
        a3 = (thresh < 1.25 ** 3).mean()

        rmse = (gt - pred) ** 2
        rmse = np.sqrt(rmse.mean())

        rmse_log = (np.log(gt) - np.log(pred)) ** 2
        rmse_log = np.sqrt(rmse_log.mean())

        abs_rel = np.mean(np.abs(gt - pred) / gt)

        sq_rel = np.mean(((gt - pred) ** 2) / gt)

        return abs_rel, sq_rel, rmse, rmse_log, a1, a2, a3

    def reset(self):
        self.accu = []
        self.count = 0

    def one_pair(self, image, depth):
        image_batch = np.expand_dims(image, 0)
        out = self.p.compute(image_batch)
        out = np.squeeze(out['depth'])
        out = cv2.resize(out, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR)

        out = self.baseline * self.focal / (out + 0.0000001)
        out = out / 2000.0  # some magic number that I made up

        errors = self._compute_errors(depth + 1e-5, out + 1e-5)
        self.accu.append(errors)
        self.count += 1

    def summarize(self):
        names = ['abs_rel', 'sq_rel', 'rmse', 'rmse_log', 'a1', 'a2', 'a3']
        errors = np.stack(self.accu, axis=0)
        errors = np.mean(errors, 0)
        out = {}
        for i in range(len(names)):
            out[names[i]] = errors[i]
        return out

modality = "depth"  # "seg"

perf = {"depth": DepthPerformance, "seg": None}[modality]()
results = {}
for town in ["Town01", "Town02"]:
    results[town] = {}
    for weather in range(1, 14):
        perf.reset()
        path = "/scratch/yang/aws_data/carla_collect/" + town + "_allsensor/default_RotationPitch=0_WeatherId=" + str(
            weather).zfill(2) + "/data_00000.h5"
        f = h5py.File(path, "r")
        for i in range(200):
            # print(i)
            image = f['CameraMiddle'][i]
            image = cv2.imdecode(image, 1)
            image = image[:, :, ::-1]

            if modality == "seg":
                seg = f['SegMiddle'][i]
                mode_image = cv2.imdecode(seg, 0)
            elif modality == "depth":
                mode_image = f['DepthMiddle'][i]
                mode_image = cv2.imdecode(mode_image, 1)

                def to_depth(array):
                    # input is bgr
                    array = array.astype(numpy.float32)
                    # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
                    normalized_depth = numpy.dot(array[:, :, :], [65536.0, 256.0, 1.0])
                    normalized_depth /= 16777215.0 / 1000.0  # (256.0 * 256.0 * 256.0 - 1.0)
                    return normalized_depth

                mode_image = to_depth(mode_image)
            else:
                raise

            perf.one_pair(image, mode_image)

        f.close()
        print(town, "weather", weather, perf.summarize())
        results[town][weather] = perf.summarize()
        with open(modality + ".pkl", "w") as ppp:
            pickle.dump(results, ppp)

print("done")