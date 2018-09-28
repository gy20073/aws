import numpy as np
import pickle
import os
import cv2


def find_time_match_info(dir_name):
    info_pickle_name = os.path.join(dir_name, 'info.gz')
    print(info_pickle_name)
    assert os.path.exists(info_pickle_name)
    info_list = pickle.load(open(info_pickle_name, "rb"))
    real_info_pickle_name = os.path.join(dir_name, 'realtimeinfo.gz')
    assert os.path.exists(real_info_pickle_name)
    real_info_list = pickle.load(open(real_info_pickle_name, "rb"))
    f_list = os.listdir(dir_name)

    im_ts_list = [int(fname[0:13]) for fname in f_list if 'final' in fname]
    pk_ts_list = [info['timestamp'] for info in info_list]
    real_ts_list = [real_info['time'] for real_info in real_info_list]

    print(len(im_ts_list), len(pk_ts_list), len(real_ts_list))

    match_info = dict()
    pk_ts_list = np.array(pk_ts_list)
    real_ts_list = np.array(real_ts_list)
    error_list = []
    for im_ts in im_ts_list:
        best_match = np.argmin(np.abs(pk_ts_list - im_ts))
        match_info[im_ts] = info_list[best_match]

        min_error = np.abs(pk_ts_list[best_match] - im_ts)
        error_list.append(min_error)
    real_error_list = []
    for im_ts in im_ts_list:
        best_match = np.argmin(np.abs(real_ts_list - im_ts))
        match_info[im_ts]['cam_coords'] = real_info_list[best_match]['cam_pos']
        match_info[im_ts]['cam_rotation'] = \
            real_info_list[best_match]['cam_rot']
        match_info[im_ts]['cam_field_of_view'] = \
            real_info_list[best_match]['cam_field_of_view']
        match_info[im_ts]['cam_near_clip'] = \
            real_info_list[best_match]['cam_near_clip']
        min_error = np.abs(real_ts_list[best_match] - im_ts)
        real_error_list.append(min_error)

    print('==> Average min error: {}MS'.format(np.mean(error_list)))
    print('==> Average min error: {}MS'.format(np.mean(real_error_list)))
    print('==> Max error: {}MS in frame{}'.format(
        np.amax(error_list), np.argmax(error_list)))
    pickle.dump(match_info, open(os.path.join(dir_name, 'info_match.p'), 'wb'), protocol=2)


def show_depth_map(dir_name):
    f_list = os.listdir(dir_name)
    depth_list = [fname for fname in f_list if 'depth' in fname]

    for f_depth in depth_list:
        depth_map = pickle.load(open(os.path.join(dir_name, f_depth), 'rb'))
        # print('Max depth: {} Min depth: {}'.format(np.amax(depth_map),
        # np.amin(depth_map)))
        cv2.imshow('depth', depth_map * 50)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def show_final_image(dir_name):
    f_list = os.listdir(dir_name)
    depth_list = [fname for fname in f_list if 'final' in fname]

    for f_depth in depth_list:
        depth_map = cv2.imread(os.path.join(dir_name, f_depth))
        # print('Max depth: {} Min depth: {}'.format(np.amax(depth_map),
        # np.amin(depth_map)))
        cv2.imshow('id', depth_map)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def show_id_map(dir_name):
    f_list = os.listdir(dir_name)
    depth_list = [fname for fname in f_list if 'id' in fname]

    for f_depth in depth_list:
        depth_map = pickle.load(open(os.path.join(dir_name, f_depth), 'rb'))
        # print('Max depth: {} Min depth: {}'.format(np.amax(depth_map),
        # np.amin(depth_map)))
        cv2.imshow('id', depth_map / np.amax(depth_map))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def show_seg_map(dir_name):
    f_list = os.listdir(dir_name)
    depth_list = [fname for fname in f_list if 'segment' in fname]

    for f_depth in depth_list:
        depth_map = pickle.load(open(os.path.join(dir_name, f_depth), 'rb'))
        # print('Max depth: {} Min depth: {}'.format(np.amax(depth_map),
        #  np.amin(depth_map)))
        cv2.imshow('id', depth_map / np.amax(depth_map))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def show_id_bbox(dir_name, extension='id', output_video=False):
    if output_video:
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        out = cv2.VideoWriter(dir_name + '.mp4', fourcc, 24.0, (1920, 1080))
    f_list = os.listdir(dir_name)
    id_list = [fname for fname in f_list if extension in fname]

    for f_id in id_list:
        id_map = pickle.load(open(os.path.join(dir_name, f_id), 'rb'))

        # own_id = id_map[1079, 960]
        frame = cv2.imread(
            os.path.join(
                dir_name,
                f_id.replace(
                    extension,
                    'final').replace(
                    '.p',
                    '.png')))
        print('unique IDs: {}'.format(np.unique(id_map)))
        # print(connected_components(id_map))
        u_id = np.unique(id_map)
        for uu in u_id:
            if uu == 0 or int(uu / 1e7) not in [26, 107]:
                continue
            y_list, x_list = np.where(id_map == uu)
            bbox = [
                np.amin(x_list),
                np.amin(y_list),
                np.amax(x_list),
                np.amax(y_list)]  # x1, y1, x2, y2
            x1, y1, x2, y2 = bbox
            # cv2.putText(frame,str(uu),(x1,y1), cv2.FONT_HERSHEY_DUPLEX,
            # 0.3,(255,255,255),1,cv2.LINE_AA)
            # if x2 - x1 < 15 or y2 - y1 <15:
            # 	continue
            if int(uu / 1e7) == 26:  # pedestriab
                color = (0, 0, 255)
            else:  # vehicle
                color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        cv2.imshow('depth', frame)
        if output_video:
            out.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    if output_video:
        out.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    dset_dir = 'D:/raw_ims/biased_10212128_clear_12h27m_x864y1634'
    # if not os.path.exists(os.path.join(dset_dir, 'info_match.p')):
    # 	find_time_match_info(dset_dir)
    # show_id_map(dset_dir)
    # find_time_match_info('H:/raw_ims')

