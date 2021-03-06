import os, cv2, Queue, time
from multiprocessing import Queue as mQueue
from multiprocessing import Process
import numpy as np


def loop_over_video(path, func, temp_down_factor=10, batch_size=1):
    # from a video, use cv2 to read each frame

    # reading from a video
    cap = cv2.VideoCapture(path)

    i = 0
    batch_frames = []
    video_init = False
    print("cap opening", cap.isOpened())
    while (cap.isOpened()):
        ret, frame = cap.read()
        if not ret:
            print("breaking")
            break

        if i % temp_down_factor:
            i += 1
            continue
        print(i)
        frame=frame[:,:,::-1] # bgr to rgb
        batch_frames.append(frame)
        if len(batch_frames) == batch_size:
            # frame is the one
            print("calling loop function...")
            frame_seq = func(batch_frames)
            print("calling loop function finished")
            if not video_init:
                fourcc = cv2.VideoWriter_fourcc(*'DIVX')
                video = cv2.VideoWriter('output.avi', fourcc, 30 // temp_down_factor,
                                        (frame_seq[0].shape[1], frame_seq[0].shape[0]))
                print("in test_video.loop_over_video, loop function output size:", frame_seq[0].shape)
                video_init = True
            for frame in frame_seq:
                frame = frame[:, :, ::-1] # rgb to bgr
                video.write(frame)
            batch_frames = []
        i += 1

    cap.release()
    video.release()


def segment(img, seg):
    img = cv2.resize(img, (768, 576))
    img = np.array(img)
    pred = seg.segment(img)
    color = seg.colorize(pred)
    return color


def vis_all(x, wrapper):
    batch_size = len(x)
    x = np.stack(x, axis=0)

    # center zoom
    #x = x[:,x.shape[1]//4:x.shape[1]*3//4, x.shape[2] // 4:x.shape[2] * 3 // 4, :]

    print("before compute")
    pred = wrapper.compute(x)
    print("after compute")

    viz_output = []
    for i in range(batch_size):
        viz_output.append(wrapper.visualize(pred, i))
    return viz_output


count = 0
x_acc = np.array([0.0, 0.0, 0.0])
x2_acc = np.array([0.0, 0.0, 0.0])
def compute_mean_std(x):
    global count, x_acc, x2_acc
    x = np.stack(x, axis=0)
    print(x.shape, x.size)
    count += x.size // 3
    x_acc += np.sum(x, axis=(0,1,2))
    x2_acc += np.sum(1.0*x*x, axis=(0,1,2))

    return np.zeros((x.shape[0], 1,1,3))


def vis_all_half(x, wrapper):
    batch_size = len(x)
    x = np.stack(x, axis=0)

    # center zoom
    #x = x[:,x.shape[1]//4:x.shape[1]*3//4, x.shape[2] // 4:x.shape[2] * 3 // 4, :]

    print("before compute")
    pred = wrapper.compute(x)
    print("after compute")
    viz_output = []
    for i in range(batch_size):
        vz = wrapper.visualize(pred, i)
        vz = vz[:, vz.shape[1]//2:, :]
        viz_output.append(vz)
    return viz_output

def vis_all_half_96(x, wrapper):
    batch_size = len(x)
    x = np.stack(x, axis=0)

    # center zoom
    #x = x[:,x.shape[1]//4:x.shape[1]*3//4, x.shape[2] // 4:x.shape[2] * 3 // 4, :]

    print("before compute")
    pred = wrapper.compute(x)
    print("after compute")
    viz_output = []
    for i in range(batch_size):
        vz = wrapper.visualize(pred, i)
        vz = vz[:, vz.shape[1]//2:, :]
        vz = cv2.resize(vz, (96, 72))
        viz_output.append(vz)
    return viz_output

def vis_all_half_zoom(x, wrapper):
    batch_size = len(x)
    x=[item[item.shape[0]//4:item.shape[0]*3//4, item.shape[1]//4:item.shape[1]*3//4, :] for item in x]
    x = np.stack(x, axis=0)

    # center zoom
    #x = x[:,x.shape[1]//4:x.shape[1]*3//4, x.shape[2] // 4:x.shape[2] * 3 // 4, :]

    print("before compute")
    pred = wrapper.compute(x)
    print("after compute")
    viz_output = []
    for i in range(batch_size):
        vz = wrapper.visualize(pred, i)
        vz = vz[:, vz.shape[1]//2:, :]
        viz_output.append(vz)
    return viz_output

def vis_async(x, input_queue):
    batch_size = len(x)
    x = np.stack(x, axis=0)
    input_queue.put(x)
    viz_output = []
    for i in range(batch_size):
        viz_output.append(np.zeros((10,10,3), dtype=np.uint8))
    return viz_output

if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    video_path = "/scratch/yang/aws_data/mkz/video_lowres.mkv"
    #video_path = "/scratch/yang/aws_data/carla_haoran/merged_lowqual.mp4"
    batch_size = 36

    if False:
        # segmentation interface
        from LinkNet.interface_segmentation import Segmenter

        seg = Segmenter(model_path="/scratch/yang/aws_data/mapillary/linknet_output2/model-last.net",
                        mean_path="/scratch/yang/aws_data/mapillary/cache/576_768/stat.t7",
                        GPU="1",
                        batch_size=batch_size,
                        compute_method="compute_argmax",
                        viz_method="visualize_argmax",
                        output_downsample_factor=4)

        loop_over_video(video_path,
                        lambda x: vis_all(x, seg),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if False:
        # downsample
        loop_over_video(video_path,
                        lambda x: [x[0][::1, ::1, ]],
                        temp_down_factor=1)

    if False:
        # depth prediction
        from monodepth.interface_depth import Depth

        depth_estimator = Depth("/home/yang/monodepth/models/model_city2eigen/model_city2eigen",
                                "/home/yang/monodepth",
                                GPU="1",
                                batch_size=batch_size)
        print("initialization finished")

        loop_over_video(video_path,
                        lambda x: vis_all(x, depth_estimator),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if False:
        from yolo.interface_darknet import YoloDetector

        detector = YoloDetector(path_cfg="/data/yang/code/aws/coco_original/yolov3.cfg",
                                path_weights="/data/yang/code/aws/data/yolov3.weights",
                                path_meta="/data/yang/code/aws/coco_original/coco.data",
                                GPU="2",
                                batch_size=batch_size,
                                compute_method="compute_logits",
                                prune_coco=True)

        loop_over_video(video_path,
                        lambda x: vis_all(x, detector),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if False:
        from all_perceptions import Perceptions

        perceptions = Perceptions(det_COCO=False,
                                  det_TL=False,
                                  det_TS=False,
                                  seg=True,
                                  depth=True,
                                  batch_size=batch_size,
                                  gpu_assignment=[0, 2, 4],
                                  compute_methods={},
                                  viz_methods={},
                                  path_config="path_jormungandr")

        loop_over_video(video_path,
                        lambda x: vis_all(x, perceptions),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if False:
        # test within the docker
        video_path = "/scratch/yang/aws_data/carla_collect/gta/gta_batch1/train/gta_00035.h5.mp4"
        batch_size = 4

        from all_perceptions import Perceptions

        perceptions = Perceptions(det_COCO=True,
                                  det_TL=True,
                                  det_TS=True,
                                  seg=True,
                                  depth=True,
                                  batch_size=batch_size,
                                  gpu_assignment=[0,1,2,3],
                                  compute_methods={},
                                  viz_methods={},
                                  path_config="path_jormungandr_newseg")

        loop_over_video(video_path,
                        lambda x: vis_all(x, perceptions),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if False:
        from all_perceptions import Perceptions

        if True:
            perceptions = Perceptions(det_COCO=True,
                                      det_TL=True,
                                      det_TS=False,
                                      seg=True,
                                      depth=True,
                                      batch_size={"det_COCO":3, "det_TL":3, "det_TS":-1, "seg":4, "depth":4},
                                      gpu_assignment=[4, 5],
                                      compute_methods={},
                                      viz_methods={},
                                      num_replicates={"det_COCO":3, "det_TL":3, "det_TS":-1, "seg":2, "depth":2},
                                      path_config="path_jormungandr")
        else:
            perceptions = Perceptions(det_COCO=False,
                                      det_TL=False,
                                      det_TS=False,
                                      seg=False,
                                      depth=True,
                                      batch_size={"det_COCO": 3, "det_TL": 3, "det_TS": -1, "seg": 4, "depth": 4},
                                      gpu_assignment=[0, 1, 4, 5],
                                      compute_methods={},
                                      viz_methods={},
                                      num_replicates={"det_COCO": 6, "det_TL": 6, "det_TS": -1, "seg": 4, "depth": 1},
                                      path_config="path_jormungandr")

        input_queue = Queue.Queue(10000)
        #input_queue = mQueue(10000)
        output_queue = perceptions.compute_async_thread_channel(input_queue)

        loop_over_video(video_path,
                        lambda x: vis_async(x, input_queue),
                        temp_down_factor=1,
                        batch_size=batch_size)


        num_ignore = 50
        counter = 0
        while True:
            counter += 1
            if counter == num_ignore:
                total_start = time.time()
            start = time.time()
            pred = output_queue.get()

            #import pdb
            #pdb.set_trace()

            duration = time.time() - start
            print("get one output, at speed ", batch_size / duration, " Hz")
            if counter >=num_ignore:
                print("     accumulated speed is:", batch_size*(counter-num_ignore+1) / (time.time()-total_start), " Hz")

    if False:
        from all_perceptions import Perceptions

        # this does not work
        p = Process(target=Perceptions.__init__, kwargs={"det_COCO":True,
                                  "det_TL":True,
                                  "det_TS":False,
                                  "seg":True,
                                  "depth":True,
                                  "batch_size":{"det_COCO": 3, "det_TL": 3, "det_TS": -1, "seg": 4, "depth": 4},
                                  "gpu_assignment":[0, 1, 4, 5],
                                  "compute_methods":{},
                                  "viz_methods":{},
                                  "num_replicates":{"det_COCO": 6, "det_TL": 6, "det_TS": -1, "seg": 4, "depth": 4},
                                  "path_config":"path_jormungandr"})
        p.start()

    # Testing the performance of the segmentation
    if True:
        # test within the docker
        #video_path = "/scratch/yang/aws_data/mkz/mkz_large_fov/output_0.avi"
        video_path = "/data1/yang/aws_data/iphone/iphone5_undis.mp4"
        #video_path = "/shared/yang/data1/aws_data/bdd100k/yolo_format/images/val/video/video.mp4-cache-thres=0.5/seq.mp4"
        video_path = "/shared/yang/data1/aws_data/mkz/video_lowres.mkv"
        # TODO: test the carla video
        batch_size = 8

        from all_perceptions import Perceptions

        perceptions = Perceptions(det_COCO=False,
                                  det_TL=True,
                                  det_TS=False,
                                  seg=False,
                                  depth=False,
                                  seg_abn=False,
                                  intersection=False,
                                  drivable_area=False,
                                  batch_size=batch_size,
                                  gpu_assignment=[0],
                                  compute_methods={},
                                  viz_methods={},
                                  path_config="path_jormungandr_newseg") #""path_jormungandr_newseg")
        #time.sleep(5)

        loop_over_video(video_path,
                        lambda x: vis_all(x, perceptions),
                        temp_down_factor=1,
                        batch_size=batch_size)
        # done

    if False:
        # segmentation interface
        from drivable_area.interface_drivable_area import DrivableArea
        batch_size = 4
        seg = DrivableArea(model_path="/data/yang_cache/aws_data/drn_d_22_drivable-e5d3dc9c.pth",
                        GPU="3",
                        mean=[0.3930478 , 0.44596469, 0.52638272],
                        std=[0.23479487, 0.22210911, 0.24706927],
                        batch_size=batch_size,
                        output_downsample_factor=4)

        video_path = "/scratch/yang/aws_data/mkz/mkz_large_fov/data_00000.h5.mp4"
        loop_over_video(video_path,
                        lambda x: vis_all(x, seg),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if False:
        # compute mean and std
        video_path = "/scratch/yang/aws_data/mkz/video_lowres.mkv"
        #video_path = "/scratch/yang/aws_data/mkz/mkz_large_fov/data_00000.h5.mp4"
        #video_path = "/scratch/yang/aws_data/mkz/output_0_768.avi"

        batch_size = 16
        loop_over_video(video_path,
                        lambda x: compute_mean_std(x),
                        temp_down_factor=1,
                        batch_size=batch_size)
        mean = x_acc/count
        print("mean is ", mean/255.0)
        print("std is ", np.sqrt(x2_acc / count - mean*mean)/ 255.0)
