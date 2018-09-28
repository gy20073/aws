import json
import math
import matplotlib.pyplot as plt
import numpy as np
import cv2
from glob import glob
import os
arrid_color_dict = dict()
from tqdm import tqdm
def rotate(a, t):
    d = np.zeros(3, 'double')
    d[0] = (np.cos(t[2]) * (np.cos(t[1]) * a[0] + np.sin(t[1]) * (
        np.sin(t[0]) * a[1] + np.cos(t[0]) * a[2])) -
        (np.sin(t[2]) * (np.cos(t[0]) * a[1] - np.sin(t[0]) * a[2])))
    d[1] = (np.sin(t[2]) * (np.cos(t[1]) * a[0] + np.sin(t[1]) * (
        np.sin(t[0]) * a[1] + np.cos(t[0]) * a[2])) +
        (np.cos(t[2]) * (np.cos(t[0]) * a[1] - np.sin(t[0]) * a[2])))
    d[2] = -np.sin(t[1]) * a[0] + np.cos(t[1]) * (
        np.sin(t[0]) * a[1] + np.cos(t[0]) * a[2])
    return d
def vec_cos(a, b):
    prod = a.dot(b)
    prod = prod * 1. / np.linalg.norm(a) / np.linalg.norm(b)
    return prod
def screen_x_to_view_plane(x, cam_near_clip, cam_field_of_view):
    near_clip_height = 2 * cam_near_clip * \
        np.tan(cam_field_of_view / 2. * (np.pi / 180.))
    near_clip_width = near_clip_height * 640. / 352.
    viewPlaneX = x * near_clip_width
    return viewPlaneX
def get_rotation_y(cam_info, xxx, yyy, zzz):
    cam_coords, cam_rotation, cam_near_clip, cam_field_of_view = cam_info
    cam_dir, cam_up, cam_east = get_cam_dir_vecs(cam_rotation)
    p_forward = np.array([xxx[1], yyy[1], zzz[1]], 'double')
    p_backward = np.array([xxx[0], yyy[0], zzz[0]], 'double')
    incam_forward_vec1 = get_kitti_format_camera_coords(
        p_forward, cam_coords, cam_rotation, cam_near_clip)
    incam_forward_vec2 = get_kitti_format_camera_coords(
        p_backward, cam_coords, cam_rotation, cam_near_clip)
    incam_forward_vec = incam_forward_vec1 - incam_forward_vec2
    cos_x = vec_cos(incam_forward_vec[[0, 2]], np.array([1, 0]))
    cos_z = vec_cos(incam_forward_vec[[0, 2]], np.array([0, 1]))

    rot = np.arccos(cos_x)  # 0 - pi
    if cos_z > 0:
        rot = - rot
    return rot

def get_cam_dir_vecs(cam_rotation):
    WORLD_NORTH = np.array([0.0, 1.0, 0.0], 'double')
    WORLD_UP = np.array([0.0, 0.0, 1.0], 'double')
    WORLD_EAST = np.array([1.0, 0.0, 0.0], 'double')
    theta = (np.pi / 180.0) * cam_rotation
    cam_dir = rotate(WORLD_NORTH, theta)
    cam_up = rotate(WORLD_UP, theta)
    cam_east = rotate(WORLD_EAST, theta)

    return cam_dir, cam_up, cam_east
def get_kitti_format_camera_coords(
        vertex,
        cam_coords,
        cam_rotation,
        cam_near_clip):
    cam_dir, cam_up, cam_east = get_cam_dir_vecs(cam_rotation)
    camera_to_target = vertex - cam_coords
    camera_to_target_unit_vector = camera_to_target * \
        (1. / np.linalg.norm(camera_to_target))

    z = np.linalg.norm(camera_to_target) * \
        cam_dir.dot(camera_to_target_unit_vector)
    y = - np.linalg.norm(camera_to_target) * \
        cam_up.dot(camera_to_target_unit_vector)
    x = np.linalg.norm(camera_to_target) * \
        cam_east.dot(camera_to_target_unit_vector)

    return np.array([x, y, z])

def get_random_color():
    color = [np.random.rand() for _ in range(3)]
    return tuple(color)

def draw_bev_ins(frame_info, line_width=1, imsize=64, upsample=8, cam_coords=None, cam_rotation=None):
    mask_r = np.zeros((imsize*upsample, imsize*upsample), dtype=np.uint8)
    mask_g = np.zeros((imsize*upsample, imsize*upsample), dtype=np.uint8)
    mask_b = np.zeros((imsize*upsample, imsize*upsample), dtype=np.uint8)
    coordlist = []
    color = []
    inslist = frame_info['object']+frame_info['ped_data']
    cam_near_clip = 0.15
    cam_field_of_view = 60.0
    if cam_coords is None:
        cam_coords = np.array(frame_info['pose']['position'])
        cam_rotation = np.array(frame_info['pose']['rotation'])
    cam_info = [cam_coords, cam_rotation,
                cam_near_clip, cam_field_of_view]
    for bbox_info in inslist:
        h, w, z = bbox_info['kitti']['dimensions']
        xxx = bbox_info['xxx']
        yyy = bbox_info['yyy']
        zzz = bbox_info['zzz']
        center_3d = [
            (xxx[2] + xxx[6]) / 2.,
            (yyy[2] + yyy[6]) / 2.,
            (zzz[2] + zzz[6]) / 2.
        ]
        rot_y = get_rotation_y(cam_info, xxx, yyy, zzz)
        trans = get_kitti_format_camera_coords(np.array(center_3d), cam_coords, cam_rotation, cam_near_clip)
        vec_l = [z * np.cos(rot_y), -z * np.sin(rot_y)]
        vec_w = [-w * np.cos(rot_y - np.pi / 2),
                 w * np.sin(rot_y - np.pi / 2)]
        vec_l = np.array(vec_l)
        vec_w = np.array(vec_w)
        center = np.array([trans[0], trans[2]])
        arrid = bbox_info['tracking_id']
        if arrid not in arrid_color_dict.keys():
            arrid_color_dict[arrid] = get_random_color()
        current_color = arrid_color_dict[arrid]
        p1 = center + 0.5 * vec_l - 0.5 * vec_w
        p2 = center + 0.5 * vec_l + 0.5 * vec_w
        p3 = center - 0.5 * vec_l + 0.5 * vec_w
        p4 = center - 0.5 * vec_l - 0.5 * vec_w
        if 0<center[1]<imsize*2 and -imsize/2<center[0]<imsize/2:
            coordlist.append([[p1[0]+imsize/2,p1[1]],[p2[0]+imsize/2,p2[1]],[p3[0]+imsize/2,p3[1]],[p4[0]+imsize/2,p4[1]]])
            color.append(bbox_info['tracking_id'])
    inslen = len(color)
    color = np.array(color, dtype=np.uint32)
    color.dtype = np.uint8
    color = color.reshape(inslen,4)[:,:-1]
    for idx, i in enumerate(coordlist):
        cv2.fillConvexPoly(mask_r, (np.array(i)*upsample).astype('int'), int(color[idx][0]))
        cv2.fillConvexPoly(mask_g, (np.array(i)*upsample).astype('int'), int(color[idx][1]))
        cv2.fillConvexPoly(mask_b, (np.array(i)*upsample).astype('int'), int(color[idx][2]))
    return np.flipud(np.concatenate((mask_r[:,:,None], mask_g[:,:,None], mask_b[:,:,None]), axis=2))

if __name__ == '__main__':
    tracklist = sorted(glob('/scratch/dqwang/daggerR6/*/track.json'))
    ignorelist = []
    f = open('/scratch/dqwang/daggerR6/bevlist.txt')
    #f.writelines(tracklist)
    for trackjson in tqdm(tracklist):
        print(trackjson)
        root = trackjson.replace('/track.json','')
        trackinfo = json.load(open(trackjson))
        for idx,i in enumerate(trackinfo):
            for j in range(0,6):
                if idx+j>len(trackinfo)-1:
                    idxx = len(trackinfo)-1
                else:
                    idxx = idx + j
                filename = str(trackinfo[idxx]['timestamp'])+'_bev_%d.png'%(j)
                tryid = idxx
                if 'object' not in i and 'ped_data' not in i:
                    cv2.imwrite(os.path.join(root,filename), np.zeros(shape=(512,512,3), dtype=np.uint8))
                    ignorelist.append(os.path.join(root,filename))
                    continue
                if 'pose' not in trackinfo[tryid]:
                    cv2.imwrite(os.path.join(root,filename), np.zeros(shape=(512,512,3), dtype=np.uint8))
                    ignorelist.append(os.path.join(root,filename))
                    continue
                mask = draw_bev_ins(i, cam_coords=np.array(trackinfo[tryid]['pose']['position'])
                            , cam_rotation=np.array(trackinfo[tryid]['pose']['rotation']))
                cv2.imwrite(os.path.join(root,filename), mask)
    pickle.dump(ignorelist,open('ignore.pkl','wb'))
