#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import time
import sys
from os import path
from os.path import dirname, abspath

import rospy
import numpy as np

import rosservice

from alphabet_trajectory import get_string_trajectory
from adeept_path.srv import DrawSentence, DrawSentenceResponse
try:
    caligraphy_file = dirname(dirname(dirname(abspath(__file__))))
    sys.path.append(caligraphy_file)
    sys.path.append(caligraphy_file+"/adeept_command/src")
except IndexError:
    pass
from helper import np2ma, acquire_coordinates
from adeept_command.srv import SetCartesianPos


# Call back
def handle_draw_sentence(req):
    # Get waypoint from trajectory function
    x, y, z, psi, theta, phi = acquire_coordinates()
    waypoints = get_string_trajectory(req.sentence, start_pos=[x,y,z])

    if False:
        # Print the planned trajectory
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(waypoints[:,0], waypoints[:,1], waypoints[:,2], c='r')
        # print(waypoints)
        plt.show()

    for waypoint in waypoints:
        # Using position controller to move the robot
        rospy.wait_for_service('set_cartesian_pos_ref')
        try:
            set_pos = rospy.ServiceProxy('set_cartesian_pos_ref', SetCartesianPos)
            success = set_pos(waypoint[0], waypoint[1], waypoint[2])
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Check if the robot has arrived the target
        # Stay at least 0.2 seconds for each waypoint
        prev_time = time.time()
        while 1:
            x, y, z, psi, theta, phi = acquire_coordinates()
            if abs(waypoint[0]-x)<0.02 and abs(waypoint[1]-y)<0.02 and \
               abs(waypoint[2]-z)<0.02 and time.time() - prev_time > 0.2:
                prev_time = time.time()
                break

    return DrawSentenceResponse(True)


# Alphabe services
def draw_sentence():
    rospy.init_node('draw_sentence_node')

    s1 = rospy.Service('draw_sentence', DrawSentence, handle_draw_sentence)

    rospy.spin()


if __name__ == "__main__":
    draw_sentence()
