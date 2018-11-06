import rosbag
import matplotlib.pyplot as plt
import numpy as np
from math import floor, cos, sin
from scipy.interpolate import UnivariateSpline

bag = rosbag.Bag('experiment_1026/scurve4.bag')
init = False
p_k = 0
states = []
T = []
for topic, msg, t in bag.read_messages(topics='/state_estimate'):
    
    if not init:
        t_init = t.to_sec()
        t_current = 0
        init = True
    else:
        t_current = t.to_sec() - t_init
    print(t_current)
    
    state = np.zeros(6)
    state[0] = msg.vx
    state[1] = msg.vy
    state[2] = msg.X + cos(msg.psi) * 1.65
    state[3] = msg.Y + sin(msg.psi) * 1.65
    state[4] = msg.psi
    state[5] = msg.wz
    states.append(state)
    T.append(t_current)

states = np.array(states)
fig = plt.figure()
ax1 = fig.add_subplot(2,2,1)
ax1.plot(states[:, 2], states[:, 3])
ax1.set_title('xy')
ax2 = fig.add_subplot(2,2,2)
ax2.plot(T, states[:, 0])
ax2.set_title('vx')
ax3 = fig.add_subplot(2,2,3)
ax3.plot(T, states[:, 1])
ax3.set_title('vy')
ax4 = fig.add_subplot(2,2,4)
ax4.plot(T, states[:, 4], '*')
ax4.set_title('psi')
plt.show()
plt.close(fig)

bag.close()
