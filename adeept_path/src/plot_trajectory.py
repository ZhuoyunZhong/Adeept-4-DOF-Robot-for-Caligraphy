#!/usr/bin/env python

import sys
from os import path
from os.path import dirname, abspath
try:
    caligraphy_file = dirname(dirname(dirname(abspath(__file__))))
    sys.path.append(caligraphy_file)
except IndexError:
    pass
from adeept_command.src.helper import acquire_coordinates

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib import cm


# Plot trajectory
def plot_trajectory():
    plt.style.use('ggplot')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    traj_x = []
    traj_y = []
    traj_z = []

    def animate(i):
        x, y, z, psi, theta, phi = acquire_coordinates()
        traj_x.append(x)
        traj_y.append(y)
        traj_z.append(z)

        plt.cla()
        ax.scatter(traj_x[-200:], traj_y[-200:], traj_z[-200:], cmap=cm.get_cmap("RdGy"))
    
    ani = FuncAnimation(fig, animate, interval=200)
    plt.show()

if __name__ == "__main__":
    plot_trajectory()