import cv2
import numpy as np

########################
# CAMERA RELATED UTILS #
########################

# make sure to convert every input into a double vector


def get_2d_from_3d(
        vertex,
        cam_coords,
        cam_rotation,
        cam_near_clip,
        cam_field_of_view):
    WORLD_NORTH = np.array([0.0, 1.0, 0.0], 'double')
    WORLD_UP = np.array([0.0, 0.0, 1.0], 'double')
    WORLD_EAST = np.array([1.0, 0.0, 0.0], 'double')
    theta = (np.pi / 180.0) * cam_rotation
    cam_dir = rotate(WORLD_NORTH, theta)
    clip_plane_center = cam_coords + cam_near_clip * cam_dir
    camera_center = -cam_near_clip * cam_dir
    near_clip_height = 2 * cam_near_clip * \
        np.tan(cam_field_of_view / 2. * (np.pi / 180.))
    near_clip_width = near_clip_height * 1920. / 1080.

    cam_up = rotate(WORLD_UP, theta)
    cam_east = rotate(WORLD_EAST, theta)
    near_clip_to_target = vertex - clip_plane_center

    camera_to_target = near_clip_to_target - camera_center

    camera_to_target_unit_vector = camera_to_target * \
        (1. / np.linalg.norm(camera_to_target))

    view_plane_dist = cam_near_clip / cam_dir.dot(camera_to_target_unit_vector)

    up3d = rotate(WORLD_UP, cam_rotation)
    right3d = rotate(WORLD_EAST, cam_rotation)
    forward3d = rotate(WORLD_NORTH, cam_rotation)
    new_origin = clip_plane_center + \
        (near_clip_height / 2.) * cam_up - (near_clip_width / 2.) * cam_east

    view_plane_point = (view_plane_dist
                        * camera_to_target_unit_vector) + camera_center
    view_plane_point = (view_plane_point + clip_plane_center) - new_origin
    viewPlaneX = view_plane_point.dot(cam_east)
    viewPlaneZ = view_plane_point.dot(cam_up)
    screenX = viewPlaneX / near_clip_width
    screenY = -viewPlaneZ / near_clip_height
    # screenX and screenY between (0, 1)
    ret = np.array([screenX, screenY], 'double')

    # print near_clip_height, near_clip_width, screenX, screenY
    if view_plane_dist > 0:
        return ret
    else:
        return None


def screen_x_to_view_plane(x, cam_near_clip, cam_field_of_view):
    # x in (0, 1)
    near_clip_height = 2 * cam_near_clip * \
        np.tan(cam_field_of_view / 2. * (np.pi / 180.))
    near_clip_width = near_clip_height * 1920. / 1080.

    viewPlaneX = x * near_clip_width

    return viewPlaneX

def generate_id_map(map_path):
    id_map = cv2.imread(map_path, -1)
    h, w, _ = id_map.shape
    id_map = np.concatenate((id_map, np.zeros((h, w, 1), dtype=np.uint8)), axis=2)
    id_map.dtype = np.uint32
    return id_map

def get_depth(
        vertex,
        cam_coords,
        cam_rotation,
        cam_near_clip,
        cam_field_of_view):
    WORLD_NORTH = np.array([0.0, 1.0, 0.0], 'double')
    WORLD_UP = np.array([0.0, 0.0, 1.0], 'double')
    WORLD_EAST = np.array([1.0, 0.0, 0.0], 'double')
    theta = (np.pi / 180.0) * cam_rotation
    cam_dir = rotate(WORLD_NORTH, theta)
    clip_plane_center = cam_coords + cam_near_clip * cam_dir
    camera_center = -cam_near_clip * cam_dir

    near_clip_to_target = vertex - clip_plane_center

    camera_to_target = near_clip_to_target - camera_center
    camera_to_target_unit_vector = camera_to_target * \
        (1. / np.linalg.norm(camera_to_target))

    depth = np.linalg.norm(camera_to_target) * \
        cam_dir.dot(camera_to_target_unit_vector)
    depth = depth - cam_near_clip

    return depth


def get_kitti_format_camera_coords(
        vertex,
        cam_coords,
        cam_rotation,
        cam_near_clip):
    cam_dir, cam_up, cam_east = get_cam_dir_vecs(cam_rotation)

    clip_plane_center = cam_coords + cam_near_clip * cam_dir

    camera_center = -cam_near_clip * cam_dir

    near_clip_to_target = vertex - clip_plane_center

    camera_to_target = near_clip_to_target - camera_center
    camera_to_target_unit_vector = camera_to_target * \
        (1. / np.linalg.norm(camera_to_target))

    z = np.linalg.norm(camera_to_target) * \
        cam_dir.dot(camera_to_target_unit_vector)
    y = - np.linalg.norm(camera_to_target) * \
        cam_up.dot(camera_to_target_unit_vector)
    x = np.linalg.norm(camera_to_target) * \
        cam_east.dot(camera_to_target_unit_vector)

    return np.array([x, y, z])


def get_cam_dir_vecs(cam_rotation):
    WORLD_NORTH = np.array([0.0, 1.0, 0.0], 'double')
    WORLD_UP = np.array([0.0, 0.0, 1.0], 'double')
    WORLD_EAST = np.array([1.0, 0.0, 0.0], 'double')
    theta = (np.pi / 180.0) * cam_rotation
    cam_dir = rotate(WORLD_NORTH, theta)
    cam_up = rotate(WORLD_UP, theta)
    cam_east = rotate(WORLD_EAST, theta)

    return cam_dir, cam_up, cam_east


def is_before_clip_plane(
        vertex,
        cam_coords,
        cam_rotation,
        cam_near_clip,
        cam_field_of_view):
    WORLD_NORTH = np.array([0.0, 1.0, 0.0], 'double')
    WORLD_UP = np.array([0.0, 0.0, 1.0], 'double')
    WORLD_EAST = np.array([1.0, 0.0, 0.0], 'double')
    theta = (np.pi / 180.0) * cam_rotation
    cam_dir = rotate(WORLD_NORTH, theta)
    clip_plane_center = cam_coords + cam_near_clip * cam_dir
    camera_center = -cam_near_clip * cam_dir
    near_clip_height = 2 * cam_near_clip * \
        np.tan(cam_field_of_view / 2. * (np.pi / 180.))
    near_clip_width = near_clip_height * 1920. / 1080.

    cam_up = rotate(WORLD_UP, theta)
    cam_east = rotate(WORLD_EAST, theta)
    near_clip_to_target = vertex - clip_plane_center

    camera_to_target = near_clip_to_target - camera_center

    camera_to_target_unit_vector = camera_to_target * \
        (1. / np.linalg.norm(camera_to_target))

    if cam_dir.dot(camera_to_target_unit_vector) > 0:
        return True
    else:
        return False


def get_clip_center_and_dir(cam_coords, cam_rotation, cam_near_clip):
    WORLD_NORTH = np.array([0.0, 1.0, 0.0], 'double')
    WORLD_UP = np.array([0.0, 0.0, 1.0], 'double')
    WORLD_EAST = np.array([1.0, 0.0, 0.0], 'double')
    theta = (np.pi / 180.0) * cam_rotation
    cam_dir = rotate(WORLD_NORTH, theta)
    clip_plane_center = cam_coords + cam_near_clip * cam_dir
    return clip_plane_center, cam_dir


# note that in numpy np.cos and np.sin use -pi-pi
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


# get the intersection point of two 3D points and a plane
def get_intersect_point(center_pt, cam_dir, vertex1, vertex2):
    c1 = center_pt[0]
    c2 = center_pt[1]
    c3 = center_pt[2]
    a1 = cam_dir[0]
    a2 = cam_dir[1]
    a3 = cam_dir[2]
    x1 = vertex1[0]
    y1 = vertex1[1]
    z1 = vertex1[2]
    x2 = vertex2[0]
    y2 = vertex2[1]
    z2 = vertex2[2]

    k_up = a1 * (x1 - c1) + a2 * (y1 - c2) + a3 * (z1 - c3)
    k_down = a1 * (x1 - x2) + a2 * (y1 - y2) + a3 * (z1 - z2)
    k = k_up / k_down
    inter_point = (1 - k) * vertex1 + k * vertex2

    return inter_point


#########################
# DATASET RELATED UTILS #
#########################

def get_labels_from_data(
        cam_coords,
        cam_rotation,
        cam_near_clip,
        cam_field_of_view,
        xxx,
        yyy,
        zzz):
    obj_center = np.zeros(3, 'double')
    obj_center[0] = 0.5 * (xxx[2] + xxx[6])
    obj_center[1] = 0.5 * (yyy[2] + yyy[6])
    obj_center[2] = 0.5 * (zzz[2] + zzz[6])

    cam_dir, cam_up, cam_east = get_cam_dir_vecs(cam_rotation)
    clip_center, _ = get_clip_center_and_dir(
        cam_coords, cam_rotation, cam_near_clip)

    obj2clip = obj_center - clip_center
    part_up = obj2clip.dot(cam_up)
    part_forward = obj2clip.dot(cam_dir)
    part_east = obj2clip.dot(cam_east)
    dist = np.linalg.norm(obj2clip)

    all_before = True
    for i in xrange(8):
        pp = np.array([xxx[i], yyy[i], zzz[i]], 'double')
        beforeb = is_before_clip_plane(
            pp,
            cam_coords,
            cam_rotation,
            cam_near_clip,
            cam_field_of_view)


def is_inside(x, y):
    return (x >= 0 and x <= 1 and y >= 0 and y <= 1)


def get_cut_edge(x1, y1, x2, y2):
    # (x1, y1) inside while (x2, y2) outside
    dx = x2 - x1
    dy = y2 - y1
    ratio_pool = []
    if x2 < 0:
        ratio = (x1 - 0) / (x1 - x2)
        ratio_pool.append(ratio)
    if x2 > 1:
        ratio = (1 - x1) / (x2 - x1)
        ratio_pool.append(ratio)
    if y2 < 0:
        ratio = (y1 - 0) / (y1 - y2)
        ratio_pool.append(ratio)
    if y2 > 1:
        ratio = (1 - y1) / (y2 - y1)
        ratio_pool.append(ratio)
    actual_ratio = min(ratio_pool)
    return x1 + actual_ratio * dx, y1 + actual_ratio * dy


def get_min_max_x_y_from_line(x1, y1, x2, y2):
    if is_inside(x1, y1) and is_inside(x2, y2):
        return min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
    if (not is_inside(x1, y1)) and (not is_inside(x2, y2)):
        return None, None, None, None
    if is_inside(x1, y1) and not is_inside(x2, y2):
        x2, y2 = get_cut_edge(x1, y1, x2, y2)
        return min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
    if is_inside(x2, y2) and not is_inside(x1, y1):
        x1, y1 = get_cut_edge(x2, y2, x1, y1)
        return min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)


def get_before_coord_inside_list(
        xxx,
        yyy,
        zzz,
        cam_coords,
        cam_rotation,
        cam_near_clip,
        cam_field_of_view):
    is_before_list = []
    is_inside_list = []
    coord_list = []

    for i in xrange(8):
        pp = np.array([xxx[i], yyy[i], zzz[i]], 'double')
        beforeb = is_before_clip_plane(
            pp,
            cam_coords,
            cam_rotation,
            cam_near_clip,
            cam_field_of_view)
        is_before_list.append(beforeb)
        if beforeb:
            cp = get_2d_from_3d(
                pp,
                cam_coords,
                cam_rotation,
                cam_near_clip,
                cam_field_of_view)
            coord_list.append(cp)
            if is_inside(cp[0], cp[1]):
                is_inside_list.append(True)
            else:
                is_inside_list.append(False)
        else:
            coord_list.append(None)
            is_inside_list.append(False)
    return is_before_list, coord_list, is_inside_list


def get_border_consistent_2d_bbox(
        xxx,
        yyy,
        zzz,
        is_before_list,
        coord_list,
        cam_coords,
        cam_rotation,
        cam_near_clip,
        cam_field_of_view):
    vertex_idx = [[0, 1],
                  [0, 3],
                  [1, 2],
                  [2, 3],
                  [4, 5],
                  [4, 7],
                  [5, 6],
                  [6, 7],
                  [0, 6],
                  [1, 7],
                  [2, 4],
                  [3, 5]]

    if False not in is_before_list:  # all before the camera coord
        x_list = []
        y_list = []

        for i in xrange(len(vertex_idx)):
            idx = vertex_idx[i]
            idx1 = idx[0]
            idx2 = idx[1]

            x_min, x_max, y_min, y_max = get_min_max_x_y_from_line(
                coord_list[idx1][0], coord_list[idx1][1], coord_list[idx2][0],
                coord_list[idx2][1])
            if x_min is not None:
                x_list.append(x_min)
                x_list.append(x_max)
                y_list.append(y_min)
                y_list.append(y_max)

        if len(x_list) == 0 or len(y_list) == 0:
            return None, None, None, None

        x1, y1 = int(min(x_list) * 1920), int(min(y_list) * 1080)
        x2, y2 = int(max(x_list) * 1920), int(max(y_list) * 1080)

        return x1, y1, x2, y2

    return None, None, None, None


def draw3dbbox(frame, bboxpoint, color=(0, 255, 0)):
    cv2.line(frame, (int(bboxpoint[0][0]), int(bboxpoint[1][0])),
             (int(bboxpoint[0][1]), int(bboxpoint[1][1])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][0]), int(bboxpoint[1][0])),
             (int(bboxpoint[0][3]), int(bboxpoint[1][3])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][1]), int(bboxpoint[1][1])),
             (int(bboxpoint[0][2]), int(bboxpoint[1][2])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][2]), int(bboxpoint[1][2])),
             (int(bboxpoint[0][3]), int(bboxpoint[1][3])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][4]), int(bboxpoint[1][4])),
             (int(bboxpoint[0][5]), int(bboxpoint[1][5])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][4]), int(bboxpoint[1][4])),
             (int(bboxpoint[0][7]), int(bboxpoint[1][7])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][5]), int(bboxpoint[1][5])),
             (int(bboxpoint[0][6]), int(bboxpoint[1][6])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][6]), int(bboxpoint[1][6])),
             (int(bboxpoint[0][7]), int(bboxpoint[1][7])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][0]), int(bboxpoint[1][0])),
             (int(bboxpoint[0][6]), int(bboxpoint[1][6])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][1]), int(bboxpoint[1][1])),
             (int(bboxpoint[0][7]), int(bboxpoint[1][7])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][2]), int(bboxpoint[1][2])),
             (int(bboxpoint[0][4]), int(bboxpoint[1][4])), thickness=3,
             color=color)
    cv2.line(frame, (int(bboxpoint[0][3]), int(bboxpoint[1][3])),
             (int(bboxpoint[0][5]), int(bboxpoint[1][5])), thickness=3,
             color=color)

def get_angle_in_2pi(unit_vec):
    theta = np.arccos(unit_vec[0])
    if unit_vec[1] > 0:
        return theta
    else:
        return 2 * np.pi - theta


######################
# MATH RELATED UTILS #
######################

def vec_cos(a, b):
    prod = a.dot(b)
    prod = prod * 1. / np.linalg.norm(a) / np.linalg.norm(b)
    return prod


def compute_bbox_ratio(bbox2, bbox):
    # bbox2 is inside bbox
    s = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
    s2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
    return s2 * 1. / s


def compute_iou(boxA, boxB):
    if boxA[0] > boxB[2] or boxB[0] > boxA[2] or boxA[1] > boxB[3] \
            or boxB[1] > boxA[3]:
        return 0
    # determine the (x, y)-coordinates of the intersection rectangle
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    # compute the area of intersection rectangle
    interArea = (xB - xA + 1) * (yB - yA + 1)

    # compute the area of both the prediction and ground-truth
    # rectangles
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = interArea / float(boxAArea + boxBArea - interArea)

    # return the intersection over union value
    return iou


def re_project_3d(ori_cam, new_cam, picture_coord, depth):
    assert len(picture_coord) == 2

    cam_dir, cam_up, cam_east = get_cam_dir_vecs(ori_cam['rotation'])
    x, y = picture_coord
    east = depth * x * 1. / cfg.GTA.FOCAL_LENGTH
    center = ori_cam['coord'] + east * cam_east + depth * cam_dir

    cam_dir, cam_up, cam_east = get_cam_dir_vecs(new_cam['rotation'])
    relative_coord = center - new_cam['coord']
    new_depth = cam_dir.dot(relative_coord)
    new_east = cam_east.dot(relative_coord)
    new_up = cam_up.dot(relative_coord)

    size_ratio = depth * 1. / new_depth  # bigger when closer
    new_x = cfg.GTA.FOCAL_LENGTH * new_east * 1. / new_depth
    new_y = cfg.GTA.FOCAL_LENGTH * new_up * 1. / new_depth
    new_y = y + new_y  # since we assume the older y is 0

    return [new_x, new_y], size_ratio
