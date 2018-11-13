timestamp() {
  date +"%Y-%m-%d_%H-%M-%S"
}

ts=$(timestamp)

output_base="/root/mount/home/bdd/intel/data/"$ts
mkdir $output_base
bash ./record_rosbag_and_video.bash $output_base &
