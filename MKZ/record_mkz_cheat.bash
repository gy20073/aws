# when finish recording, press k and enter, to stop the recording, wait for 2 seconds, and press control+C

timestamp() {
  date +"%Y-%m-%d_%H-%M-%S"
}

ts=$(timestamp)

source ../catkin_ws_docker/devel/setup.bash

OUTPUT_PREFIX="/root/mount/media/bdd/Samsung_T5/recordings/"$ts
mkdir $OUTPUT_PREFIX

roslaunch mkz_intel record_visualization_and_raw_cheat.launch \
    viz_output_path:=$OUTPUT_PREFIX"/video"
