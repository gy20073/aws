import numpy as np
import copy, math, time, cv2
from multiprocessing import Process, Pipe
from common import resize_images

class Perceptions:
    @staticmethod
    def worker(initializer, params, conn):
        print("begin initialization")
        instance = initializer(**params)
        print("initialization finished")
        while True:
            # there should be a communication protocol
            cmd, data = conn.recv()
            # have the multiple combinations to avoid extra communications
            if cmd == "compute":
                print("start compute on ", initializer, time.time())
                logit = instance.compute(data)
                conn.send(logit)
                print("end compute on ", initializer, time.time())
            elif cmd == "visualize":
                # pred, ibatch
                viz = instance.visualize(data[0], data[1])
                conn.send(viz)
            elif cmd == "visualize_low_thresh":
                # pred, ibatch
                viz = instance.visualize_logits_low_thresh(data[0], data[1])
                conn.send(viz)
            else:
                print("wrong command")

    def __init__(self,
                 det_COCO=True,
                 det_TL=True,
                 det_TS=True,
                 seg=True,
                 depth=True,
                 batch_size=1,
                 gpu_assignment=[0,1],
                 compute_methods={},
                 viz_methods={},
                 path_config="path_jormungandr"):
        '''
        :param gpu_assignment: could be a list, where use round robin method, or could be a dict, that directly assign
        :param compute_methods: could change the default compute methods, with a dict
        '''
        getattr(self, path_config)()

        modalities = {"det_COCO": det_COCO,
                      "det_TL": det_TL,
                      "det_TS": det_TS,
                      "seg": seg,
                      "depth": depth}

        if isinstance(gpu_assignment, list):
            out = {}
            i = 0
            for mode in sorted(modalities.keys()):
                if modalities[mode]:
                    out[mode] = gpu_assignment[i % len(gpu_assignment)]
                    i += 1
            gpu_assignment = out
        for key in gpu_assignment:
            gpu_assignment[key] = str(gpu_assignment[key])

        from LinkNet.interface_segmentation import Segmenter
        from monodepth.interface_depth import Depth
        from yolo.interface_darknet import YoloDetector
        interfaces = {"det_COCO": YoloDetector,
                      "det_TL": YoloDetector,
                      "det_TS": YoloDetector,
                      "seg": Segmenter,
                      "depth": Depth}

        self.instances = {}

        for mode in sorted(modalities.keys()):
            if modalities[mode]:
                # select which GPU to use
                initializer = interfaces[mode]

                gpu = gpu_assignment[mode]
                print("Model ", mode, " is using GPU ", gpu)
                params = copy.deepcopy(self.paths[mode])
                params.update({"GPU": gpu})
                params.update({"batch_size": batch_size})
                if mode in compute_methods:
                    params.update({"compute_method": compute_methods[mode]})
                if mode in viz_methods:
                    params.update({"viz_method": viz_methods[mode]})

                parent_conn, child_conn = Pipe()
                p = Process(target=self.worker, args=(initializer, params, child_conn))
                p.start()

                self.instances[mode] = parent_conn

    def path_jormungandr(self):
        self.paths = {}
        self.paths["det_COCO"] = {"path_cfg": "/data/yang/code/aws/coco_original/yolov3.cfg",
                                  "path_weights": "/data/yang/code/aws/data/yolov3.weights",
                                  "path_meta": "/data/yang/code/aws/coco_original/coco.data"}
        self.paths["det_TL"] = {"path_cfg": "/data/yang/code/aws/traffic_light/yolov3-TL.cfg.test",
                                "path_weights": "/scratch/yang/aws_data/bdd100k/yolo_format/backup/yolov3-TL.backup",
                                "path_meta": "/data/yang/code/aws/traffic_light/TL.data"}
        self.paths["det_TS"] = {"path_cfg": "/data/yang/code/aws/traffic_sign/yolov3-CL.cfg.test",
                                "path_weights": "/scratch/yang/aws_data/coco_lisa_v2/backup/yolov3-CL.backup",
                                "path_meta": "/data/yang/code/aws/traffic_sign/CL.data"}
        self.paths["seg"] = {"model_path": "/scratch/yang/aws_data/mapillary/linknet_output2/model-last.net",
                             "mean_path": "/scratch/yang/aws_data/mapillary/cache/576_768/stat.t7"}
        self.paths["depth"] = {"model_path": "/home/yang/monodepth/models/model_city2eigen/model_city2eigen",
                               "python_path": "/home/yang/monodepth"}

    @staticmethod
    def merge_images(viz_dict, new_size):
        nimage = len(viz_dict)
        sqrt_n = math.ceil(math.sqrt(nimage))
        sqrt_n = int(sqrt_n)
        # compute how many rows
        nrow = int(math.ceil(nimage * 1.0 / sqrt_n))

        output = np.zeros((new_size[0]*nrow, new_size[1]*sqrt_n, 3), dtype=np.uint8)

        for i, key in enumerate(sorted(viz_dict.keys())):
            image = viz_dict[key]
            image = cv2.resize(image,
                       dsize=(new_size[1], new_size[0]),
                       interpolation=cv2.INTER_LINEAR)

            irow = i // sqrt_n
            icol = i % sqrt_n
            output[irow*new_size[0]:(irow+1)*new_size[0],
                   icol*new_size[1]:(icol+1)*new_size[1], :] = image
        return output

    def compute(self, images, intermediate_size=(576, 768)):
        # depth 256*512, seg: 576*768, yolo 312*416
        if intermediate_size is not None:
            images = resize_images(images, intermediate_size)

        self.images = images

        for mode in self.instances.keys():
            conn = self.instances[mode]
            conn.send(("compute", images))

        out_logits = {}

        for mode in self.instances.keys():
            conn = self.instances[mode]
            out_logits[mode] = conn.recv()

        return out_logits

    def visualize(self, logits_dict, ibatch, subplot_size=(312, 416)):
        #return np.zeros((4,4, 3), dtype=np.uint8)

        out_viz = {"0_original": self.images[ibatch]}
        for mode in self.instances.keys():
            conn = self.instances[mode]
            conn.send(("visualize", (logits_dict[mode], ibatch)))
            #if "det" in mode:
            #    conn.send(("visualize_low_thresh", (logits_dict[mode], ibatch)))

        for mode in self.instances.keys():
            conn = self.instances[mode]
            out_viz[mode] = conn.recv()
            #if "det" in mode:
            #    out_viz[mode+"_lowThres"] = conn.recv()

        for mode in self.instances.keys():
            conn = self.instances[mode]
            if "det" in mode:
                conn.send(("visualize_low_thresh", (logits_dict[mode], ibatch)))

        for mode in self.instances.keys():
            conn = self.instances[mode]
            if "det" in mode:
                out_viz[mode+"_lowThres"] = conn.recv()

        return self.merge_images(out_viz, subplot_size)
