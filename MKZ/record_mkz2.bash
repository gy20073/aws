timestamp() {
  date +"%Y-%m-%d_%H-%M-%S"
}

ts=$(timestamp)

source ../catkin_ws_docker/devel/setup.bash

OUTPUT_PREFIX="/root/mount/media/bdd/Samsung_T5/recordings/"$ts
mkdir $OUTPUT_PREFIX

roslaunch mkz_intel record_visualization_and_raw.launch \
    viz_output_path:=$OUTPUT_PREFIX"/video"
