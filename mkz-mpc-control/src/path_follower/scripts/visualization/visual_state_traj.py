import rosbag
import matplotlib.pyplot as plt
import numpy as np
from math import floor, cos, sin
from scipy.interpolate import UnivariateSpline

bag = rosbag.Bag('experiment_1026/waypoints_intersection_left_1_new.bag')
init = False
p_k = 0
states = []
T = []

fig = plt.figure()
ax1 = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2)

for topic, msg, t in bag.read_messages(topics='/waypoints_state_received'):
    
    if not init:
        t_init = t.to_sec()
        t_current = 0
        init = True
    else:
        t_current = t.to_sec() - t_init  

    p_k = (p_k + 1) % 20
    if p_k != 0:
        continue  

    msg = msg.state
    state = np.zeros(6)
    state[0] = msg.vx
    state[1] = msg.vy
    state[2] = msg.X + cos(msg.psi) * 1.65
    state[3] = msg.Y + sin(msg.psi) * 1.65
    state[4] = msg.psi
    state[5] = msg.wz
    states.append(state)
    T.append(t_current)

    ax1.plot([state[2], state[2] + cos(state[4])], [state[3], state[3] + sin(state[4])], 'r-')
    ax2.plot([state[2], state[2] + cos(state[4])], [state[3], state[3] + sin(state[4])], 'r-')

p_k = 0

for topic, msg, t in bag.read_messages(topics='/waypoints_state_received'):

    p_k = (p_k + 1) % 20
    if p_k != 0:
        continue

    msg = msg.traj
    num_steps_received = len(msg.point)
    points = np.zeros((6, num_steps_received))
    for i in range(num_steps_received):
        points[0, i] = msg.point[i].t
        points[1, i] = msg.point[i].x
        points[2, i] = msg.point[i].y
        points[3, i] = msg.point[i].v
        points[4, i] = msg.point[i].theta
        points[5, i] = msg.point[i].kappa
    ax1.plot(points[1, :], points[2, :], 'b-')

p_k = 0

for topic, msg, t in bag.read_messages(topics='/waypoints_state_received'):

    p_k = (p_k + 1) % 20
    if p_k != 0:
        continue

    X_ref = msg.state.X + cos(msg.state.psi) * 1.65
    Y_ref = msg.state.Y + sin(msg.state.psi) * 1.65
    psi_ref = msg.state.psi 
    msg = msg.waypoints
    num_steps_received = len(msg.points) - 1
    points = np.zeros((2, num_steps_received))
    for i in range(num_steps_received):
        points[0, i] = msg.points[i].x * cos(psi_ref) - msg.points[i].y * sin(psi_ref) + X_ref
        points[1, i] = msg.points[i].x * sin(psi_ref) + msg.points[i].y * cos(psi_ref) + Y_ref
    ax2.plot(points[0, :], points[1, :], 'g-')

plt.show()
plt.close(fig)
bag.close()
