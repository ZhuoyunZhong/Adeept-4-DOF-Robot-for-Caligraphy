#!/usr/bin/env python

import time
import sys
from os import path
from os.path import dirname, abspath

import rospy
import numpy as np

import rosservice

from adeept_path.srv import DrawAlphabet, DrawAlphabetResponse
from alphabet_trajectory import get_alphabet_trajectory
try:
    caligraphy_file = dirname(dirname(dirname(abspath(__file__))))
    sys.path.append(caligraphy_file)
except IndexError:
    pass
from adeept_command.src.helper import np2ma, acquire_coordinates
from adeept_command.srv import SetCartesianPos


# Call back
def handle_draw_alphabet(req):
    alphabet = req.alphabet
    
    # Get waypoint from trajectory function
    waypoints = get_alphabet_trajectory(alphabet)
    for waypoint in waypoints:
        # Using position controller to move the robot
        rospy.wait_for_service('set_cartesian_pos_ref')
        try:
            set_pos = rospy.ServiceProxy('set_cartesian_pos_ref', SetCartesianPos)
            success = set_pos(waypoint[0], waypoint[1], waypoint[2])
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Check if the robot has arrived the target
        while 1:
            x, y, z, psi, theta, phi = acquire_coordinates()
            if abs(waypoint[0]-x)<0.01 and abs(waypoint[1]-y)<0.01 and \
               abs(waypoint[2]-z)<0.01:
                break

    return DrawAlphabetResponse(True)

# Alphabe services
def draw_alphabet():
    rospy.init_node('draw_alphabet_node')

    s1 = rospy.Service('draw_alphabet', DrawAlphabet, handle_draw_alphabet)

    rospy.spin()


if __name__ == "__main__":
    draw_alphabet()
