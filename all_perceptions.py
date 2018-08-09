import numpy as np
import copy, math, time, cv2, threading, Queue
from multiprocessing import Process, Pipe
from common import resize_images
from collections import defaultdict

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
                start = time.time()
                #print("start compute on ", initializer, start)
                logit = instance.compute(data)
                conn.send(logit)
                endtime = time.time()
                #print("end compute on ", initializer, endtime, "using:", (endtime-start)*1000, " ms")
            elif cmd == "visualize":
                # pred, ibatch
                viz = instance.visualize(data[0], data[1])
                conn.send(viz)
            elif cmd == "visualize_low_thresh":
                # pred, ibatch
                viz = instance.visualize_logits_low_thresh(data[0], data[1])
                conn.send(viz)
            elif cmd == "general":
                func = getattr(instance, data[0])
                output = func(**data[1])
                conn.send(output)
            else:
                print("wrong command")

    def rep_name(self, mode, i_replicate):
        if i_replicate == 0:
            return mode
        else:
            return mode + "_" + str(i_replicate)

    def __init__(self,
                 det_COCO=True,
                 det_TL=True,
                 det_TS=True,
                 seg=True,
                 depth=True,
                 batch_size=1, # batch_size could also be a dict
                 gpu_assignment=[0,1],
                 compute_methods={},
                 viz_methods={},
                 num_replicates={},
                 path_config="path_jormungandr"):
        '''
        :param gpu_assignment: could be a list, where use round robin method, or could be a dict, that directly assign
        :param compute_methods: could change the default compute methods, with a dict
        '''
        num_replicates = defaultdict(lambda: 1, num_replicates)
        self.num_replicates = num_replicates
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
                    for irep in range(num_replicates[mode]):
                        mode_name_i = self.rep_name(mode, irep)
                        out[mode_name_i] = gpu_assignment[i % len(gpu_assignment)]
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
        self._batch_size = {}
        self.all_modes = {}

        for mode in sorted(modalities.keys()):
            if modalities[mode]:
                self.all_modes[mode] = True
                for irep in range(num_replicates[mode]):
                    mode_name_i = self.rep_name(mode, irep)

                    # select which GPU to use
                    initializer = interfaces[mode]

                    gpu = gpu_assignment[mode_name_i]
                    print("Model ", mode, " replicate ", irep, " is using GPU ", gpu)
                    params = copy.deepcopy(self.paths[mode])
                    params.update({"GPU": gpu})
                    if isinstance(batch_size, dict):
                        params.update({"batch_size": batch_size[mode]})
                        self._batch_size[mode] = batch_size[mode]
                    else:
                        params.update({"batch_size": batch_size})
                        self._batch_size[mode] = batch_size

                    if mode in compute_methods:
                        params.update({"compute_method": compute_methods[mode]})
                    if mode in viz_methods:
                        params.update({"viz_method": viz_methods[mode]})
                    if mode == "det_COCO":
                        params.update({"prune_coco": True})

                    parent_conn, child_conn = Pipe()
                    p = Process(target=self.worker, args=(initializer, params, child_conn))
                    p.start()

                    if "det" in mode:
                        print("sleeping to stable create det models")
                        time.sleep(1)

                    self.instances[mode_name_i] = parent_conn

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

    def path_docker(self):
        self.paths = {}
        codebase = "/root/aws/"
        self.paths["det_COCO"] = {"path_cfg": codebase+"coco_original/yolov3.cfg",
                                  "path_weights": "/root/models/COCO-yolov3.weights",
                                  "path_meta": codebase+"coco_original/coco.data.docker"}
        self.paths["det_TL"] = {"path_cfg": codebase+"traffic_light/yolov3-TL.cfg.test",
                                "path_weights": "/root/models/TrafficLight-yolov3-TL.backup",
                                "path_meta": codebase+"traffic_light/TL.data.docker"}
        self.paths["det_TS"] = {"path_cfg": codebase+"traffic_sign/yolov3-CL.cfg.test",
                                "path_weights": "/root/models/TrafficSign-yolov3-CL.backup",
                                "path_meta": codebase+"traffic_sign/CL.data.docker"}
        self.paths["seg"] = {"model_path": "/root/models/Segmentation-LinkNet-model-152.net",
                             "mean_path": "/root/models/Segmentation_LinkNet_576_768.stat.t7"}
        self.paths["depth"] = {"model_path": "/root/models/model_city2eigen",
                               "python_path": "/root/monodepth"}

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
            assert(self.num_replicates[mode] == 1)
            conn = self.instances[mode]
            conn.send(("compute", images))

        out_logits = {}

        for mode in self.instances.keys():
            conn = self.instances[mode]
            out_logits[mode] = conn.recv()

        return out_logits

    def compute_async(self, input_queue):
        # start multiple threads for each mode
        self.batch_id = 0 # the priority number
        t = threading.Thread(target=self._thread_input_replicater, args=(input_queue,))
        t.start()

        self.output_queue_mode = {}
        for mode in self.all_modes.keys():
            self.output_queue_mode[mode] = []
            for irep in range(self.num_replicates[mode]):
                self.output_queue_mode[mode].append(Queue.Queue(3))
                t = threading.Thread(target=self._thread_compute, args=(mode, irep,))
                t.start()

        # merge the replicate output to a queue
        self.output_replicate_merged = {}
        self.next_ids = {}
        for mode in self.all_modes.keys():
            t = threading.Thread(target=self._thread_replicate_merger, args=(mode,))
            t.start()

        output_queue = Queue.Queue(5)
        t = threading.Thread(target=self._thread_output_merger, args=(output_queue,))
        t.start()

        return output_queue

    def _thread_input_replicater(self, input_queue):
        # setup each of the input queues
        self.input_queue_replicate = {}
        for mode in self.all_modes.keys():
            self.input_queue_replicate[mode] = Queue.Queue(5)
        while True:
            data = input_queue.get()
            self.images = data
            for mode in self.input_queue_replicate.keys():
                self.input_queue_replicate[mode].put((self.batch_id, data))
            self.batch_id += 1

    def _thread_compute(self, mode, irep):
        # setup the output queue
        output_queue = self.output_queue_mode[mode][irep]
        batch_size = self._batch_size[mode]
        conn = self.instances[self.rep_name(mode, irep)]
        while True:
            id, images = self.input_queue_replicate[mode].get()
            assert(images.shape[0] % batch_size == 0)
            res = []
            for i in range(images.shape[0] // batch_size):
                conn.send(("compute", images[i*batch_size : (i+1)*batch_size]))
                res.append(conn.recv())
            out = np.concatenate(res, 0)
            output_queue.put((id, out))

    def _thread_replicate_merger(self, mode):
        num_rep = self.num_replicates[mode]
        self.output_replicate_merged[mode] = Queue.Queue(5)
        current = [None for _ in range(num_rep)]
        self.next_ids[mode] = 0

        history = list(range(num_rep))
        while True:
            search_order = [i for i in range(num_rep) if current[i] is not None]
            second_order = []
            for i in history:
                if current[i] is None and i not in second_order:
                    second_order.append(i)
            third_order = list(set(range(num_rep)) - set(search_order) - set(second_order))

            #ntrails = 0
            for i in search_order + second_order + third_order:
                if current[i] is None:
                    current[i] = self.output_queue_mode[mode][i].get()
                    #ntrails += 1

                if current[i][0] == self.next_ids[mode]:
                    self.next_ids[mode] += 1
                    self.output_replicate_merged[mode].put(current[i])
                    current[i] = None

                    history.append(i)
                    history = history[1:]

                    #print(mode, "used ", ntrails, " get")
                    break

    def _thread_output_merger(self, output_queue):
        while True:
            res = {}
            ids = []
            for mode in self.output_replicate_merged.keys():
                if self.output_replicate_merged[mode].empty():
                    #print("waiting for mode: ", mode)
                    pass
                id, res[mode] = self.output_replicate_merged[mode].get()
                ids.append(id)
            assert(all(np.array(ids)==ids[0]))
            output_queue.put(res)


    def visualize_det_class(self, mode, logits_dict, ibatch, cid):
        conn = self.instances[mode]
        conn.send(("general", ("visualize_class_heatmap",
                              {"pred": logits_dict[mode],
                               "ibatch": ibatch,
                               "classid": cid})))
        return conn.recv()

    def visualize(self, logits_dict, ibatch, subplot_size=(312, 416)):
        out_viz = {"0_original": self.images[ibatch]}
        for mode in self.all_modes.keys():
            if "det" in mode:
                # ignores the high threshold detection boxes
                continue
            conn = self.instances[mode]
            conn.send(("visualize", (logits_dict[mode], ibatch)))

        for mode in self.all_modes.keys():
            if "det" in mode:
                continue
            conn = self.instances[mode]
            out_viz[mode] = conn.recv()

        # low threshold detection boxes
        for mode in self.all_modes.keys():
            conn = self.instances[mode]
            if "det" in mode:
                conn.send(("visualize_low_thresh", (logits_dict[mode], ibatch)))

        for mode in self.all_modes.keys():
            conn = self.instances[mode]
            if "det" in mode:
                out_viz[mode+"_lowThres"] = conn.recv()

        if "det_COCO" in self.instances.keys():
            # class 2 is car
            out_viz["det_COCO_zcar"] = self.visualize_det_class("det_COCO", logits_dict, ibatch, 2)

        if "det_TL" in self.instances.keys():
            out_viz["det_TL_zgreen"] = self.visualize_det_class("det_TL", logits_dict, ibatch, 1)

        if "det_TS" in self.instances.keys():
            out_viz["det_TS_zstop"] = self.visualize_det_class("det_TS", logits_dict, ibatch, 0)


        return self.merge_images(out_viz, subplot_size)