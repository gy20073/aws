# MKZ MPC Control

## Steps to run the simulation:
1. ./setup.sh
2. catkin_make
3. rosrun path_follower MPC
4. roslaunch path_follower path_follower_waypoints_simu_smooth.launch
(To change test track, change the track name in plotter.py and toy_planner_waypoints.py.
Currently, there are three options "Tra_1", "Tra_2", and "Tra_3")

## Experiment:
roslaunch path_follower path_follower_waypoints_smooth.launch