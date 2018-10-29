#!/usr/bin/env python
import rospy
import scipy.io
import os, rospkg
from path_follower.msg import state_Dynamic, Trajectory2D, TrajectoryPoint2D
import matplotlib.pyplot as plt
from math import cos, sin

axis_range = 20
ini_flag = False
ini_flag2 = False
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
X2 = 0
X3 = 0
Y2 = 0
Y3 = 0

def callback(data):
    global axis_range, ax, ini_flag, ini_flag2, X2, X3
    if ini_flag2:
        if ini_flag:
            while len(ax.lines)>1:     
                ax.lines.pop(1) 
        ini_flag = True
        ax.plot(X2, Y2, color='blue', marker='.', markersize = 3, linewidth=2)
        ax.plot([data.X, data.X + 2.85 * cos(data.psi)], [data.Y, data.Y + 2.85 * sin(data.psi)], color = 'red', marker = '*', markersize = 8)
        ax.axis([data.X - axis_range, data.X + axis_range, data.Y - axis_range, data.Y + axis_range])
        plt.draw()

def clearmomery():
    print "clear done"
    plt.clf()
    plt.close('all')

def ref_traj_callback(data) :
    global ini_flag2, X2, Y2
    X2tem = []
    Y2tem = []
    ini_flag2 = True
    for index in range(0, len(data.point), 50):
        X2tem.append(data.point[index].x)
        Y2tem.append(data.point[index].y)
    X2 = X2tem
    Y2 = Y2tem

def plotter():
    global ref_x, ref_y, ax
    # initialize node
    rospy.init_node('plotter', anonymous=True)
    #rospack = rospkg.RosPack()
    #reference = scipy.io.loadmat(os.path.join(rospack.get_path("path_follower"), "scripts", "waypoint_loader", "Tra_4.mat"))['Tra_4']
    #ref_x = reference[0][:]
    #ref_y = reference[1][:]
    #ax.plot(ref_x, ref_y)

    rospy.Subscriber('state_estimate', state_Dynamic, callback, queue_size=1)
    rospy.Subscriber('final_trajectory', Trajectory2D, ref_traj_callback, queue_size=1)
    plt.show()
    rospy.spin()
    rospy.on_shutdown(clearmomery)

if __name__ == '__main__':
    plotter()
