timestamp() {
  date +"%Y-%m-%d_%H-%M-%S"
}

ts=$(timestamp)

source ../catkin_ws_docker/devel/setup.bash

output_base="/root/mount/home/bdd/intel/data/"$ts
mkdir $output_base
bash ./record_rosbag_and_video.bash $output_base &
