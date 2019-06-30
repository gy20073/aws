import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat
bag = rosbag.Bag('2018-10-13-16-20-05.bag')
X = []
Y = []
time = []
initialize = False

for topic, msg, t in bag.read_messages(topics='/state_estimate'):
    if initialize == False:
       t_initial = t.to_sec()
       initialize = True
    X.append(msg.X)
    Y.append(msg.Y)
    time.append(t.to_sec()-t_initial)

X = np.array(X)
Y = np.array(Y)
time = np.array(time)
Tra_4 = np.vstack((X,Y,time))
plt.plot(Tra_4[0, :], Tra_4[1, :])
plt.show()
savemat('Tra_4', {'Tra_4':Tra_4})
bag.close()
