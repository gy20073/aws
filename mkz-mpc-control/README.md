# MKZ MPC Control

## Steps to run the simulation:
1. ./setup.sh
2. catkin_make
3. rosrun path_follower mpc
4. roslaunch path_follower path_follower_MPC_simu.launch
(To change test track, change the track name in plotter.py and toy_planner.py.
Currently, there are two options "Tra_1" and "Tra_2")