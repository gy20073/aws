import os, cv2
import numpy as np

def loop_over_video(path, func, temp_down_factor=10):
    # from a video, use cv2 to read each frame

    # reading from a video
    cap = cv2.VideoCapture(path)

    i = 0
    while (cap.isOpened()):
        ret, frame = cap.read()
        if not ret:
            break

        if i % temp_down_factor:
            i += 1
            continue
        print(i)
        # frame is the one
        frame = func(frame)
        if i==0:
            fourcc = cv2.VideoWriter_fourcc(*'DIVX')
            video = cv2.VideoWriter('output.avi', fourcc, 30//temp_down_factor,
                                    (frame.shape[1], frame.shape[0]))
            print(frame.shape)

        video.write(frame)
        i += 1

    cap.release()
    video.release()


def segment(img, seg):
    img = cv2.resize(img, (768, 576))
    img = np.array(img)
    pred = seg.segment(img)
    color = seg.colorize(pred)
    return color

if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    video_path = "/scratch/yang/aws_data/mkz/video_highqual.mp4"

    if False:
        # segmentation interface
        from LinkNet.interface import Segmenter
        segmentor=Segmenter()
        segmentor.init()

        loop_over_video(video_path, lambda x: segment(x, segmentor))

    if False:
        # downsample
        loop_over_video(video_path, lambda x: x[::4,::4,])

    if True:
        # depth prediction
        from monodepth.interface_depth import Depth
        depth_estimator = Depth("/home/yang/monodepth/models/model_city2eigen/model_city2eigen",
                                "/home/yang/monodepth",
                                GPU="1")
        print("initialization finished")

        loop_over_video(video_path,
                        lambda x: depth_estimator.visualize(depth_estimator.compute(x)),
                        temp_down_factor=30)
