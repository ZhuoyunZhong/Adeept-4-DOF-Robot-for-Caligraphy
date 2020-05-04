#!/usr/bin/env python

import sys
from os import path
from os.path import dirname, abspath
try:
    caligraphy_file = dirname(dirname(dirname(abspath(__file__))))
    sys.path.append(caligraphy_file+"/adeept_command/src")
except IndexError:
    pass

from helper import acquire_coordinates

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib import cm


# Plot trajectory
def plot_trajectory():
    plt.style.use('ggplot')
    fig = plt.figure()
    ax = fig.add_subplot(111)
    traj_x = []
    traj_y = []
    traj_z = []

    def animate(i):
        x, y, z, psi, theta, phi = acquire_coordinates()
        if z < 0.008:
            traj_x.append(-y)
            traj_y.append(x)
            traj_z.append(z)

        plt.cla()
        ax.scatter(traj_x[:], traj_y[:], cmap=cm.get_cmap("BuPu"))
    
    ani = FuncAnimation(fig, animate, interval=200)
    plt.show()

if __name__ == "__main__":
    plot_trajectory()
