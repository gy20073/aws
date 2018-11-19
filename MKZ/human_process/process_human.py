'''
import matplotlib
matplotlib.use('GtkAgg')
% matplotlib inline
import matplotlib.pyplot as plt
'''

import rosbag, sys, os, cv2, IPython, math
os.environ['CARLA_VERSION']='0.9.X'
sys.path.append("../../CIL_modular/drive_interfaces/carla/")
sys.path.append('../../CIL_modular/drive_interfaces/carla/carla_client')
from carla_recorder import Recorder
import numpy as np
from operator import itemgetter
from collections import namedtuple

def read_topic(bag, topic, item_transformer=lambda x: x):
    output = []
    for item in bag.read_messages(topics=topic):
        time = item.timestamp.to_nsec()
        content = item_transformer(item.message)
        output.append((time, content))
    return output

def image_converter(x):
    arr = x.data
    arr = np.fromstring(arr, dtype=np.uint8)
    arr = cv2.imdecode(arr, 1)
    # now the channel is flipped, so actually in RGB order
    # since we expect RGB order in the next step, we keep it the same

    # arr = arr[:,:,::-1]
    # arr = cv2.imencode(".jpg", arr)[1]
    # arr = arr.tostring()
    return arr

class NN():
    def __init__(self, seq):
        self.seq = seq
        self.cur = 0

    def find_time(self, time):
        while self.cur < len(self.seq) - 1 and self.seq[self.cur][0] < time:
            self.cur += 1
        # now the self.cur is larger than time
        if self.cur == 0:
            return 0
        else:
            time_1 = self.seq[self.cur - 1][0]
            time_now = self.seq[self.cur][0]
            if time - time_1 < time_now - time:
                return self.cur - 1
            else:
                return self.cur

# TODO: fill this with last bag
def btns_to_seq(btns, default_action="down"):
    changes = []
    for key in btns:
        for item in btns[key]:
            if item[1]:
                changes.append((item[0], key))
    changes = sorted(changes, key=itemgetter(0))
    print("changes", changes)
    if len(changes) == 0:
        changes=[(0, default_action)]
        print("no changes detected")
    # build a standard list using the up time stamps
    directions = []
    i_c = 0
    for i in range(len(btns[default_action])):
        time = btns[default_action][i][0]
        while i_c < len(changes) - 1 and changes[i_c+1][0] < time:
            i_c += 1
        directions.append((time, changes[i_c][1]))
    return directions

def quaternion_to_yaw(msg):
    q = msg.orientation
    yaw = math.atan2(2 * (q.x * q.w + q.y * q.z), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return yaw

def main(ss, recorder, pos):
    best_key = "left"
    for key in ss:
        if len(ss[key]) < len(ss[best_key]):
            best_key = key
    print("the best key is ", best_key)

    ss_nn = {}
    for key in ss:
        ss_nn[key] = NN(ss[key])

    last_i_yaw = 0

    #yaw_rate = ss["yaw_rate"]
    speed_ms = ss["speed"]
    yaw_xsens = ss["yaw_xsens"]

    speed_ms_NN2 = NN(ss["speed"])

    for i in range(len(ss[best_key])):
        # we are going to issue one for each of this key
        sensors = {best_key: ss[best_key][i][1]}
        this_time = ss[best_key][i][0]
        indexes = {}
        for key in ss:
            if key != best_key:
                this_i = ss_nn[key].find_time(this_time)
                indexes[key] = this_i
                sensors[key] = ss[key][this_i][1]

        # we finished the sensor dict
        # now we move on to the yaw rate integration (to yaw) and combined with speed to get the positions
        this_i_yaw = indexes["yaw_xsens"]

        for j in range(last_i_yaw, this_i_yaw):
            k = speed_ms_NN2.find_time(yaw_xsens[j][0])
            yaw_now = yaw_xsens[j][1]
            pos[0] += np.cos(yaw_now) * speed_ms[k][1] * (yaw_xsens[j + 1][0] - yaw_xsens[j][0]) * 1.0e-9
            pos[1] += np.sin(yaw_now) * speed_ms[k][1] * (yaw_xsens[j + 1][0] - yaw_xsens[j][0]) * 1.0e-9
        # done
        last_i_yaw = this_i_yaw
        sensors["pos"] = pos
        sensors["yaw"] = yaw_now

        # now we need to store them into the carla recorder

        second_level = namedtuple('second_level', ['forward_speed', 'transform',
                                                   'collision_other', 'collision_pedestrians', 'collision_vehicles'])
        transform = namedtuple('transform', ['location', 'orientation'])
        loc = namedtuple('loc', ['x', 'y'])
        ori = namedtuple('ori', ['x', 'y', 'z'])
        Meas = namedtuple('Meas', ['player_measurements', 'game_timestamp'])

        measurements = Meas(
            second_level(sensors["speed"],
                         transform(loc(sensors["pos"][0],
                                       sensors["pos"][1]),
                                   ori(0.0, 0.0,  np.rad2deg(sensors["yaw"]))),
                         0.0, 0.0, 0.0),
            this_time * 1e-6 - 1542218274550.0)
        if sensors["noise"] == "inc":
            action = lambda x: x
            action.steer = sensors["steer"]
            action.throttle = sensors["throttle"]
            action.brake = sensors["brake"]
            action.hand_brake = 0.0
            action.reverse = 0.0

            mapping = {"up": 5.0, "down": 2.0, "left": 3.0, "right": 4.0}

            recorder.record(measurements,
                            {'CameraLeft': sensors["left"],
                             'CameraMiddle': sensors["middle"],
                             'CameraRight': sensors["right"]},
                            action,
                            action,
                            mapping[sensors["direction"]],
                            None
                            )
        else:
            pass
            #print("it is noise")
    return pos

def read_a_bag(bag_path, last_direction, last_noise):
    print("start reading bag")
    bag = rosbag.Bag(bag_path, 'r')

    im = read_topic(bag, "/compressed0", image_converter)
    imL = read_topic(bag, "/compressed1", image_converter)
    imR = read_topic(bag, "/compressed2", image_converter)

    # this yaw is not used, but I will leave it here
    yaw_xsens = read_topic(bag, "/xsens/imu/data", quaternion_to_yaw)

    yaw_rate = read_topic(bag, "/vehicle/twist", lambda msg: msg.twist.angular.z)
    speed_ms = read_topic(bag, "/vehicle/twist", lambda msg: msg.twist.linear.x)

    # this require special handling for the vehicle control
    steer = read_topic(bag, '/vehicle/steering_report', lambda msg: msg.steering_wheel_angle / 8.2)
    # range 0.15 to 0.5
    brake = read_topic(bag, '/vehicle/brake_report',
                       lambda msg: msg.pedal_input)  # , lambda msg: msg.torque_input / 3412.0)
    # range 0.15 to 0.8
    throttle = read_topic(bag, '/vehicle/throttle_report', lambda msg: msg.pedal_input)

    # populate the button inputs
    # direction
    btns = {}
    for key in ["up", "down", "left", "right"]:
        this = read_topic(bag, '/vehicle/misc_1_report', lambda msg: getattr(msg, "btn_ld_" + key))
        btns[key] = this

    directions = btns_to_seq(btns, last_direction)

    btns = {}
    # decrease: noise start, increase, noise end
    for key in ["inc", "dec"]:
        this = read_topic(bag, '/vehicle/misc_1_report', lambda msg: getattr(msg, "btn_cc_gap_" + key))
        btns[key] = this

    have_noise = btns_to_seq(btns, last_noise)

    # find the image key with least number of images
    ss = {"left": imL, "middle": im, "right": imR,
          "steer": steer, "brake": brake, "throttle": throttle,
          "speed": speed_ms, "yaw_rate": yaw_rate, "direction": directions, "noise": have_noise,
          "yaw_xsens": yaw_xsens}
    print("reading rosbag done")
    return ss


if __name__ == "__main__":
    base = "/scratch/yang/aws_data/human_driving/2018-11-14_09-37-53/"
    output_path = "/scratch/yang/aws_data/human_driving/converted/"
    debug_num_bags = range(4, 31)
    # end of paramters

    recorder = Recorder(output_path, [768, 576], image_cut=[0, 10000000])

    last_direction = "down"
    last_noise = "dec"
    pos = [0.0, 0.0]

    for i in debug_num_bags:
        bag_path = base + "bag_"+str(i)+".bag"
        ss = read_a_bag(bag_path, last_direction, last_noise)
        last_direction = ss["direction"][-1][1]
        last_noise = ss["noise"][-1][1]

        pos = main(ss, recorder, pos)

    recorder.close()