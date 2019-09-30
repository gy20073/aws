import os, cv2, Queue, time
from multiprocessing import Queue as mQueue
from multiprocessing import Process
import numpy as np

FPS = 5

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
            frame_seq1, frame_seq2 = func(batch_frames)
            print("calling loop function finished")
            if not video_init:
                fourcc = cv2.VideoWriter_fourcc(*'DIVX')
                video1 = cv2.VideoWriter(path+".full.avi", fourcc, FPS // temp_down_factor,
                                        (frame_seq1[0].shape[1], frame_seq1[0].shape[0]))
                video2 = cv2.VideoWriter(path + ".noviz.avi", fourcc, FPS // temp_down_factor,
                                         (frame_seq2[0].shape[1], frame_seq2[0].shape[0]))
                video_init = True
            for frame in frame_seq1:
                frame = frame[:, :, ::-1] # rgb to bgr
                video1.write(frame)
            for frame in frame_seq2:
                frame = frame[:, :, ::-1]  # rgb to bgr
                video2.write(frame)
            batch_frames = []
        i += 1

    cap.release()
    video1.release()
    video2.release()

def cut(x):
    out_full, out_noviz = [], []
    for item in x:
        # the video + seg
        H, W, C = item.shape
        full = item[:, W//4:W//2, :]
        out_full.append(full)

        # the video only
        noviz = full[:H//2, :, :]
        out_noviz.append(noviz)
    return out_full, out_noviz

if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    video_path = "/home/yang/data/aws_data/CIL_modular_data/_benchmarks_results/mm45_v4_SqnoiseShoulder_rfsv6_withTL_fixTL_14_YangExp3cam_Town01/_images/episode_14_3_132.27.mp4"

    loop_over_video(video_path,
                    cut,
                    temp_down_factor=1,
                    batch_size=1)
