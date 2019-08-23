import numpy as np
import copy, math, time, cv2, threading, Queue, os
from multiprocessing import Process, Pipe
from multiprocessing import Queue as mQueue
from common import resize_images
from collections import defaultdict
from scipy.ndimage.interpolation import zoom

class Perceptions:
    @staticmethod
    def worker(initializer, params, conn):
        print("begin initialization")
        if initializer == "seg_abn":
            os.environ["CUDA_VISIBLE_DEVICES"] = str(params["GPU"])
            params["GPU"] = 0
            from inplace_abn.interface_abn import SegmenterABN
            initializer = SegmenterABN
        elif initializer == "0intersection":
            from intersection_detector.interface_intersection import Intersection
            initializer = Intersection
        elif initializer == 'seg':
            from LinkNet.interface_segmentation import Segmenter
            initializer = Segmenter
        elif initializer == "drivable_area":
            from drivable_area.interface_drivable_area import DrivableArea
            initializer = DrivableArea

        instance = initializer(**params)
        print(initializer, "initialization finished")
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
                 seg=True, add_lane_color=False,
                 depth=True,
                 seg_abn=False,
                 intersection=False,
                 drivable_area=False,
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
                      "depth": depth,
                      "seg_abn": seg_abn,
                      "0intersection": intersection,
                      "drivable_area": drivable_area}

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

        if seg:
            Segmenter = "seg"
        else:
            Segmenter = None
        from monodepth.interface_depth import Depth
        from yolo.interface_darknet import YoloDetector

        if seg_abn:
            SegmenterABN = "seg_abn"
        else:
            SegmenterABN = None
        if intersection:
            Intersection = "0intersection"
        else:
            Intersection = None
        if drivable_area:
            DrivableArea = "drivable_area"
        else:
            DrivableArea = None

        interfaces = {"det_COCO": YoloDetector,
                      "det_TL": YoloDetector,
                      "det_TS": YoloDetector,
                      "seg": Segmenter,
                      "depth": Depth,
                      "seg_abn": SegmenterABN,
                      "0intersection": Intersection,
                      "drivable_area": DrivableArea}


        self.instances = {}
        self._batch_size = {}
        self.all_modes = {}
        self.channel_id = 0
        self.processes = {}

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
                    if mode == "seg" and add_lane_color:
                        params.update({"attach_lane_color": True})

                    parent_conn, child_conn = Pipe()
                    p = Process(target=self.worker, args=(initializer, params, child_conn))
                    p.start()

                    self.processes[mode_name_i] = p

                    if "det" in mode or "abn" in mode or "intersection" in mode:
                        print("sleeping to stable create det models", mode)
                        time.sleep(2)

                    self.instances[mode_name_i] = parent_conn

    def path_jormungandr(self):
        self.path_docker(codebase="/home/yang/code/aws/",
                         model_base="/home/yang/data/aws_data/models/",
                         monodepth_python_path="/shared/yang/software/monodepth")

    def path_jormungandr_newseg(self):
        self.path_jormungandr()

        model_base = "/home/yang/data/aws_data/models/"
        self.paths["seg"] = {"model_path": model_base + "seg_v2.net",
                             "mean_path": model_base + "seg_stat_v2.t7"}

    def path_docker(self,
                    codebase="/root/aws/",
                    monodepth_python_path="/root/monodepth",
                    model_base = "/root/models/"):
        self.paths = {}

        if "root" in codebase:
            suffix = ".docker"
        else:
            suffix = ""

        self.paths["det_COCO"] = {"path_cfg": codebase+"coco_original/yolov3.cfg",
                                  "path_weights": model_base + "COCO-yolov3.weights",
                                  "path_meta": codebase+"coco_original/coco.data" + suffix}
        self.paths["det_TL"] = {"path_cfg": codebase+"traffic_light/yolov3-TL.cfg.test",
                                "path_weights": model_base + "TrafficLight-yolov3-TL.backup",
                                "path_meta": codebase+"traffic_light/TL.data" + suffix}
        self.paths["det_TS"] = {"path_cfg": codebase+"traffic_sign/yolov3-CL.cfg.test",
                                "path_weights": model_base + "TrafficSign-yolov3-CL.backup",
                                "path_meta": codebase+"traffic_sign/CL.data" + suffix}
        self.paths["seg"] = {"model_path": model_base + "Segmentation-LinkNet-model-152.net",
                             "mean_path": model_base + "Segmentation_LinkNet_576_768.stat.t7"}
        self.paths["depth"] = {"model_path": model_base + "model_city2eigen",
                               "python_path": monodepth_python_path}
        self.paths["seg_abn"] = {"model_path": model_base + "abn_wideres38.pth.tar",}
        self.paths["0intersection"] = {
            "model_path": "TODO",
            "mean_path": "TODO"}
        self.paths["drivable_area"] = {
            "model_path": model_base + "drivable_python2.pth"}

    def path_docker_newseg(self):
        self.path_docker()
        codebase = "/root/aws/"
        self.paths["seg"] = {"model_path": "/root/models/seg_v2.net",
                             "mean_path": "/root/models/seg_stat_v2.t7"}

    def merge_images(self, viz_dict, new_size):
        nimage = len(viz_dict)
        sqrt_n = math.ceil(math.sqrt(nimage))
        sqrt_n = int(sqrt_n)
        # compute how many rows
        nrow = int(math.ceil(nimage * 1.0 / sqrt_n))

        output = np.zeros((new_size[0]*nrow, new_size[1]*sqrt_n, 3), dtype=np.uint8)
        self.viz_nrow = nrow
        self.viz_ncol = sqrt_n

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

    def get_viz_nrow_ncol(self):
        return self.viz_nrow, self.viz_ncol

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

    def compute_async_process(self, input_queue):
        raise DeprecationWarning()

        assert(isinstance(input_queue, type(mQueue())))
        output_queue = mQueue(5)
        p=Process(target=self.compute_async_impl, args=(input_queue, output_queue))
        p.start()
        return output_queue

    def compute_async_thread(self, input_queue):
        raise

        if not isinstance(input_queue, Queue.Queue):
            print("warning, not using Queue.Queue for the thread interface")
        output_queue = Queue.Queue(5)
        return self.compute_async_impl(input_queue, output_queue, block=False)

    def compute_async_thread_channel(self, input_queue):
        this_channel_id = self.channel_id
        self.channel_id += 1
        print("compute async thread channel ", this_channel_id)

        # feed this input_queue to the common input queue
        if this_channel_id == 0:
            # if this is the first call, then setup the main computing pipeline
            self.common_input_queue = Queue.Queue(5)
            self.common_output_queue = Queue.Queue(5)
            self.compute_async_impl(self.common_input_queue, self.common_output_queue, block=False)
            self.list_input_queue = []
            self.list_output_queue = []

            t = threading.Thread(target=self._thread_channel_output, args=(self.common_output_queue, ))
            t.start()

        self.list_output_queue.append(Queue.Queue(5))

        t = threading.Thread(target=self._thread_channel_input, args=(input_queue, self.common_input_queue, this_channel_id,))
        t.start()

        return self.list_output_queue[-1]

    def _thread_channel_input(self, input_queue, common_input, channel_id):
        while True:
            data = input_queue.get()
            common_input.put((channel_id, data))

    def _thread_channel_output(self, common_output_queue):
        while True:
            id, data = common_output_queue.get()
            self.list_output_queue[id].put(data)

    def compute_async_impl(self, input_queue, output_queue, block=True):
        # start multiple threads for each mode
        self.batch_id = 0 # the priority number
        self.input_queue_replicate = {}
        for mode in self.all_modes.keys():
            self.input_queue_replicate[mode] = Queue.Queue(5)
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

        t = threading.Thread(target=self._thread_output_merger, args=(output_queue,))
        t.start()
        if block:
            t.join() # this will never returns

        return output_queue

    def _thread_input_replicater(self, input_queue):
        # setup each of the input queues
        while True:
            data = input_queue.get()
            self.images = data[1]
            for mode in self.input_queue_replicate.keys():
                self.input_queue_replicate[mode].put((self.batch_id, data))
            self.batch_id += 1

    def _thread_compute(self, mode, irep):
        # setup the output queue
        output_queue = self.output_queue_mode[mode][irep]
        batch_size = self._batch_size[mode]
        conn = self.instances[self.rep_name(mode, irep)]
        while True:
            id, payload = self.input_queue_replicate[mode].get()
            ichannel, images = payload
            assert(images.shape[0] % batch_size == 0)
            res = []
            for i in range(images.shape[0] // batch_size):
                conn.send(("compute", images[i*batch_size : (i+1)*batch_size]))
                res.append(conn.recv())
            out = np.concatenate(res, 0)
            output_queue.put((id, (ichannel,out)))

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

    def _space2depth(self, input, downscale_factor):
        B, H, W, C = input.shape
        assert (H % downscale_factor == 0)
        assert (W % downscale_factor == 0)

        input = np.reshape(input, (B, H//downscale_factor, downscale_factor,
                                      W//downscale_factor, downscale_factor, C))
        input = np.transpose(input, (0, 1, 3, 5, 2, 4))
        # to shape: B H/Down, W/Down, C, Down_h, Down_w
        input = np.reshape(input, (B, H//downscale_factor, W//downscale_factor, -1))
        return input

    def _merge_logits_all_perception(self, logits_dict):
        res = []
        det_sz = (39, 52)
        #print(sorted(logits_dict.keys()))
        for key in sorted(logits_dict.keys()):
            if key == "seg":
                factor = 3
                size = (det_sz[0]*factor, det_sz[1]*factor)
                resized = resize_images(logits_dict[key], size, interpolation=cv2.INTER_NEAREST)
                resized *= 0.1
                resized = self._space2depth(resized, factor)
                res.append(resized)
            elif key == "depth":
                factor = 5
                size = (det_sz[0]*factor, det_sz[1]*factor)
                resized = resize_images(logits_dict[key], size, interpolation=cv2.INTER_LINEAR)
                resized *= 50
                resized = self._space2depth(resized, factor)
                res.append(resized)
            elif "det" in key:
                dB, dH, dW, dC = logits_dict[key].shape
                # compute the effective height
                eH = int(1.0 * det_sz[0] / det_sz[1] * dW)
                assert(eH == det_sz[0] and dW == det_sz[1])
                # compute the upper margine
                H_start = (dH - eH) // 2
                # crop the useful part
                cropped = logits_dict[key][:, H_start:(H_start + eH), :, :]

                # multiply the amplify factor
                num_classes = dC // 9 - 5
                # we amplify the objectness score by 10
                factor = [1.0] * 4 + [10.0] + [1.0] * num_classes
                factor = np.array(factor * 9)
                factor = np.reshape(factor, newshape=(1, 1, 1, -1))
                cropped = cropped * factor

                res.append(cropped)
            elif key == "seg_abn":
                factor = 2
                size = (det_sz[0] * factor, det_sz[1] * factor)
                resized = resize_images(logits_dict[key], size, interpolation=cv2.INTER_NEAREST)
                resized *= 0.1
                resized = self._space2depth(resized, factor)
                res.append(resized)
            elif key == "0intersection":
                factor = 1
                size = (det_sz[0] * factor, det_sz[1] * factor)
                # replicate the image to the size
                expanded = np.reshape(logits_dict[key], (-1, 1, 1, 1))
                resized = np.tile(expanded, (1, size[0], size[1], 1))

                res.append(resized)
            elif key == "drivable_area":
                factor = 3
                size = (det_sz[0] * factor, det_sz[1] * factor)
                resized = resize_images(logits_dict[key], size, interpolation=cv2.INTER_NEAREST)
                resized *= 0.1
                resized = self._space2depth(resized, factor)
                res.append(resized)
        concat = np.concatenate(res, axis=3)

        return concat


    def _thread_output_merger(self, output_queue):
        while True:
            res = {}
            ids = []
            for mode in self.output_replicate_merged.keys():
                if self.output_replicate_merged[mode].empty():
                    #print("waiting for mode: ", mode)
                    pass
                id, payload = self.output_replicate_merged[mode].get()
                ichannel, res[mode] = payload
                ids.append(id)
            assert(all(np.array(ids)==ids[0]))
            # this cost 0.28 seconds, which is 120Hz, will not be the bottleneck, however it will add some delay
            res = self._merge_logits_all_perception(res)
            output_queue.put((ichannel, res))


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

    def destroy(self):
        for key in self.processes:
            print("destroying process ", key)
            p = self.processes[key]
            p.terminate()

