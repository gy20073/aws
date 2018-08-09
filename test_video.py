import os, cv2, Queue, time
import numpy as np


def loop_over_video(path, func, temp_down_factor=10, batch_size=1):
    # from a video, use cv2 to read each frame

    # reading from a video
    cap = cv2.VideoCapture(path)

    i = 0
    batch_frames = []
    video_init = False
    while (cap.isOpened()):
        ret, frame = cap.read()
        if not ret:
            break

        if i % temp_down_factor:
            i += 1
            continue
        print(i)
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
    print("before compute")
    pred = wrapper.compute(x)
    print("after compute")
    viz_output = []
    for i in range(batch_size):
        viz_output.append(wrapper.visualize(pred, i))
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
        video_path = "/root/video_lowres.mkv"

        from all_perceptions import Perceptions

        perceptions = Perceptions(det_COCO=True,
                                  det_TL=True,
                                  det_TS=True,
                                  seg=False,
                                  depth=True,
                                  batch_size=batch_size,
                                  gpu_assignment=[0, 1],
                                  compute_methods={},
                                  viz_methods={},
                                  path_config="path_docker")

        loop_over_video(video_path,
                        lambda x: vis_all(x, perceptions),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if True:
        from all_perceptions import Perceptions

        perceptions = Perceptions(det_COCO=True,
                                  det_TL=True,
                                  det_TS=False,
                                  seg=True,
                                  depth=True,
                                  batch_size={"det_COCO":3, "det_TL":3, "det_TS":-1, "seg":4, "depth":4},
                                  gpu_assignment=[0, 1, 4, 5],
                                  compute_methods={},
                                  viz_methods={},
                                  num_replicates={"det_COCO":6, "det_TL":6, "det_TS":-1, "seg":4, "depth":4},
                                  path_config="path_jormungandr")

        input_queue = Queue.Queue(10000)
        output_queue = perceptions.compute_async(input_queue)

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
            duration = time.time() - start
            print("get one output, at speed ", batch_size / duration, " Hz")
            if counter >=num_ignore:
                print("     accumulated speed is:", batch_size*(counter-num_ignore+1) / (time.time()-total_start), " Hz")
