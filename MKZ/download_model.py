import os, copy, sys
from subprocess import call
import sys

if __name__ == "__main__":
    exp_id_list = sys.argv[1] #"mm45_v4_SqnoiseShoulder_exptownv3_notown0102_mergefollowstraight"

    sp = exp_id_list.split(",")
    for id in sp:
        exp_id = id.strip()

        remote_path = "/scratch/yang/aws_data/CIL_modular_data/models/" + exp_id
        local_path = "/root/mount/home/bdd/intel/data/CIL_modular_data/models/" + exp_id
        if not os.path.exists(local_path):
            os.mkdir(local_path, 0777)

        scp_cmd = ["scp", "-i", "/root/mount/home/bdd/yang/yang_private", "yang@jormungandr.ist.berkeley.edu:"]

        scp_now = copy.deepcopy(scp_cmd)
        scp_now[-1] += remote_path + "/checkpoint"

        scp_now.append(local_path)

        call(" ".join(scp_now), shell=True)

        with open(local_path+"/checkpoint", "r") as f:
            line = f.readline()

        model = line.split("\"")[-2]
        print(model)


        scp_now = copy.deepcopy(scp_cmd)
        scp_now[-1] += remote_path + "/" + model + "*"

        scp_now.append(local_path)

        call(" ".join(scp_now), shell=True)

