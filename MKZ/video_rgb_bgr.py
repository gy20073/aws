import os, cv2, Queue, time
from multiprocessing import Queue as mQueue
from multiprocessing import Process
import numpy as np


def loop_over_video(path, func, output_path, temp_down_factor=1, batch_size=1):
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
                video = cv2.VideoWriter(output_path, fourcc, 30 // temp_down_factor,
                                        (frame_seq[0].shape[1], frame_seq[0].shape[0]))
                print("in test_video.loop_over_video, loop function output size:", frame_seq[0].shape)
                video_init = True
            for frame in frame_seq:
                video.write(frame)
            batch_frames = []
        i += 1

    cap.release()
    video.release()

def invert_channel(batch_frames):
    out = []
    for frame in batch_frames:
        out.append(frame[:,:,::-1])
    return out

if __name__ == "__main__":
    path = "/scratch/yang/aws_data/mkz/mkz2/original_images.avi"
    output_path = "/scratch/yang/aws_data/mkz/mkz2/inverted.avi"
    loop_over_video(path, invert_channel, output_path)
