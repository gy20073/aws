import rosbag
import matplotlib.pyplot as plt
import numpy as np
from math import floor, cos, sin
from scipy.interpolate import UnivariateSpline

dt = 0.05
bag = rosbag.Bag('experiment_1026/waypoints_intersection_left_1.bag')
init = False
p_k = 0

xy_prev = np.zeros((2, 3))

for topic, msg, t in bag.read_messages(topics='/waypoints_state_received'):
    
    if not init:
        t_init = t.to_sec()
        t_current = 0
        init = True
    else:
        t_current = t.to_sec() - t_init
    
    state = msg.state
    xy_prev[:, :-1] = xy_prev[:, 1:]
    xy_prev[0, -1] = state.X
    xy_prev[1, -1] = state.Y

    p_k = (p_k + 1) % 10
    if p_k != 9:
        continue

    print(t_current)
    waypoints = msg.waypoints
    state = msg.state
    num_steps_received = len(waypoints.points)-1
    dt_received = waypoints.dt
    horizon = dt_received * num_steps_received

    # dump last point adjust weights control derivatives
    w = np.ones(num_steps_received+2) * 0.1
    w[0:2] *= 10
    w[-1] *= 5
    points = np.zeros((2, num_steps_received+2))
    points[0, 0] -= state.vx * dt_received
    for i in range(num_steps_received):
        points[0, i+2] = waypoints.points[i].x 
        points[1, i+2] = waypoints.points[i].y
    t_received = np.linspace(-dt_received, horizon, num_steps_received+2)
    num_points = int(floor(horizon / dt + 1)) + 10
    t = np.linspace(0, (num_points - 1) * dt, num_points)
    w_psi = np.ones(t.shape[0]) * 0.1
    w_psi[0] = w_psi[0] * 10
    spl_x = UnivariateSpline(t_received, points[0, :], k=3, w=w)
    spl_y = UnivariateSpline(t_received, points[1, :], k=3, w=w)
    spl_x_dot = spl_x.derivative()
    spl_y_dot = spl_y.derivative()
    spl_x_val = spl_x(t)
    spl_y_val = spl_y(t)
    spl_x_dot_val= spl_x_dot(t)
    spl_y_dot_val = spl_y_dot(t)
    spl_v_val = np.sqrt(spl_x_dot_val**2 + spl_y_dot_val**2)
    spl_psi_val = np.arctan2(spl_y_dot_val, spl_x_dot_val)
    spl_psi_val[0] = state.psi
    spl_psi_fn = UnivariateSpline(t, spl_psi_val, k=3, w=w_psi)
    spl_psi_val = spl_psi_fn(t)
    spl_yr_fn = spl_psi_fn.derivative()
    spl_yr_val = spl_yr_fn(t)

    w = np.ones(num_steps_received+3) * 0.1
    w[0:3] *= 20
    w[-1] *= 5
    w[int(num_steps_received/2)+2] *= 2
    points = np.zeros((2, num_steps_received+3))
    points[:, :3] = xy_prev
    for i in range(num_steps_received):
        points[0, i+3] = waypoints.points[i].x * cos(state.psi) - waypoints.points[i].y * sin(state.psi) + state.X
        points[1, i+3] = waypoints.points[i].x * sin(state.psi) + waypoints.points[i].y * cos(state.psi) + state.Y
    t_received = np.linspace(-dt_received*2, horizon, num_steps_received+3)
    num_points = int(floor(horizon / dt + 1)) + 10
    t = np.linspace(0, (num_points - 1) * dt, num_points)
    spl_x_new = UnivariateSpline(t_received, points[0, :], k=3, w=w)
    spl_y_new = UnivariateSpline(t_received, points[1, :], k=3, w=w)
    spl_x_dot_new = spl_x_new.derivative()
    spl_y_dot_new = spl_y_new.derivative()
    spl_x_val_new = spl_x_new(t)
    spl_y_val_new = spl_y_new(t)
    spl_x_dot_val_new= spl_x_dot_new(t)
    spl_y_dot_val_new = spl_y_dot_new(t)
    spl_v_val_new = np.sqrt(spl_x_dot_val_new**2 + spl_y_dot_val_new**2)
    spl_psi_val_new = np.arctan2(spl_y_dot_val_new, spl_x_dot_val_new)
    spl_psi_val_new[0] = state.psi
    spl_psi_fn_new = UnivariateSpline(t, spl_psi_val_new, k=3)
    spl_psi_val_new = spl_psi_fn_new(t)
    spl_yr_fn_new = spl_psi_fn_new.derivative()
    spl_yr_val_new = spl_yr_fn_new(t)

    fig = plt.figure(figsize=(20, 12))
    ax1 = fig.add_subplot(2,2,1)    
    ax1.plot(t, spl_v_val, '.-', label='prev')
    ax1.plot(t, spl_v_val_new, '.-', label='new')
    ax1.set_title('velocity')
    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(t, spl_yr_val, '.-', label='prev')
    ax2.plot(t, spl_yr_val_new, '.-', label='new')
    ax2.set_title('yaw rate')
    ax3 = fig.add_subplot(2,2,3)
    ax3.plot(t, spl_psi_val + state.psi, '.-', label='prev')
    ax3.plot(t, spl_psi_val_new, '.-', label='new')
    ax3.set_title('psi')
    ax4 = fig.add_subplot(2,2,4)
    ax4.plot(spl_x_val * cos(state.psi) - spl_y_val * sin(state.psi) + state.X, spl_x_val * sin(state.psi) + spl_y_val * cos(state.psi) + state.Y, '.-', label='prev')
    ax4.plot(spl_x_val_new, spl_y_val_new, '.-', label='new')
    ax4.plot(points[0, :], points[1, :], 's-', label='origin')
    ax4.plot(points[0, :3], points[1, :3], 'rs-')
    ax4.legend()
    ax4.set_title('xy coordinates')
    fig.savefig('figures/'+str(round(t_current, 2))+'.png')
    #plt.show()
    plt.close(fig)

bag.close()