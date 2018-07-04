import os, cv2
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
                video = cv2.VideoWriter('output.avi', fourcc, 30//temp_down_factor,
                                        (frame_seq[0].shape[1], frame_seq[0].shape[0]))
                print("in test_video.loop_over_video, loop function output size:",frame_seq[0].shape)
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
    batch_size=len(x)
    x = np.stack(x, axis=0)
    print("before compute")
    pred = wrapper.compute(x)
    print("after compute")
    viz_output=[]
    for i in range(batch_size):
        viz_output.append(wrapper.visualize(pred, i))
    return viz_output

if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    video_path = "/scratch/yang/aws_data/mkz/video_highqual.mp4"
    batch_size=8

    if False:
        # segmentation interface
        from LinkNet.interface_segmentation import Segmenter

        seg = Segmenter(model_path="/scratch/yang/aws_data/mapillary/linknet_output2/model-last.net",
                        mean_path="/scratch/yang/aws_data/mapillary/cache/576_768/stat.t7",
                        GPU="1",
                        batch_size=batch_size,
                        compute_method="compute_argmax",
                        viz_method="visualize_argmax")

        loop_over_video(video_path,
                        lambda x: vis_all(x, seg),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if False:
        # downsample
        loop_over_video(video_path, lambda x: x[::4,::4,])

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
                                compute_method="compute_logit_list")

        loop_over_video(video_path,
                        lambda x: vis_all(x, detector),
                        temp_down_factor=1,
                        batch_size=batch_size)

    if True:
        batch_size = 4
        from all_perceptions import Perceptions
        perceptions = Perceptions(det_COCO=True,
                                 det_TL=True,
                                 det_TS=True,
                                 seg=True,
                                 depth=True,
                                 batch_size=batch_size,
                                 gpu_assignment=[1, 2],
                                 compute_methods={},
                                 viz_methods={},
                                 path_config="path_jormungandr")

        loop_over_video(video_path,
                        lambda x: vis_all(x, perceptions),
                        temp_down_factor=1,
                        batch_size=batch_size)
