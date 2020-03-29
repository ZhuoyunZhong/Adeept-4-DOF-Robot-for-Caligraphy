#!/usr/bin/env python 

from adeept_command.srv import AdeeptHomoMatrix, AdeeptHomoMatrixResponse
from numpy import array, sin, cos
from helper import np2ma
import sys
import rospy
from math import pi


# Compute Transform Matrix A
def homogeneous_A(a1, alph1, d1, theta1):
    A = array([[cos(theta1), (sin(theta1)*-1) * cos(alph1), sin(theta1) * sin(alph1), a1 * cos(theta1)],
               [sin(theta1), cos(theta1) * cos(alph1), (cos(theta1)*-1) * sin(alph1), a1 * sin(theta1)],
               [0, sin(alph1), cos(alph1), d1],
               [0, 0, 0, 1]])
    return A


def handle_homogeneous_matrix(req):    
    # Robot dimensions
    l1 = 0.1
    l2 = 0.065
    l3 = 0.055
    l4 = 0.0665
    # pen length adjustment
    pen = 0.013 + 0

    ##_---------------------- DH Parameters----------------------------------------

    a1 = 0
    alph1 = pi/2
    d1 = l1
    theta1 = req.q1

    a2 = l2
    alph2 = pi
    d2 = 0
    theta2 = req.q2+pi/2

    a3 = 0
    alph3 = pi/2
    d3 = 0
    theta3 = req.q3+pi

    a4 = 0
    alph4 = 0
    d4 = l3
    theta4 = req.q4

    a5 = pen
    alph5 = 0
    d5 = l4
    theta5 = 0

    A1 = np2ma(homogeneous_A(a1, alph1, d1, theta1))
    A2 = np2ma(homogeneous_A(a2, alph2, d2, theta2))
    A3 = np2ma(homogeneous_A(a3, alph3, d3, theta3))
    A4 = np2ma(homogeneous_A(a4, alph4, d4, theta4))
    A5 = np2ma(homogeneous_A(a5, alph5, d5, theta5))
    return AdeeptHomoMatrixResponse(A1, A2, A3, A4, A5)



def get_homogeneous():
    rospy.init_node('homogeneous_matrix_server')
    s = rospy.Service('homogeneous_matrix', AdeeptHomoMatrix, handle_homogeneous_matrix)

    rospy.spin()


if __name__ == "__main__":
    get_homogeneous()
