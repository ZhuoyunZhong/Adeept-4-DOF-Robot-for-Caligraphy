#!/usr/bin/env python

from adeept_command.srv import AdeeptKinFK, AdeeptKinFKResponse, AdeeptHomoMatrix
from adeept_command.srv import AdeeptKinIK, AdeeptKinIKResponse
from math import atan2, sqrt, pow, sin, cos, acos, atan2
import numpy
from helper import ma2np

import rospy


def handle_inverse_kinematics(req):
    # Robot dimension
    a1 = 0.55
    a2 = 0.425
    a3 = 0.345
    a4 = 0.11

    # Input Coordinate
    x = req.x
    y = req.y
    z = req.z
    psi = req.psi
	
    # Perform IK
    try:
        # calculate joint 3
        q3 = a1 - a4 - z
        if not (0 <= q3 <= 0.3):
            print "IK failed, coordinate z is not reachable."
            return AdeeptKinIKResponse(False, 0, 0, 0)
        # calculate joint 2
        q2 = acos(round(-(a2**2 + a3**2 - x**2 - y**2) / (2*a2*a3),3))
        q2_1 = q2
        q2_2 = -q2
        # calculate joint 1
        nor_1 = pow((a3*cos(q2_1)+a2), 2) + pow(a3*sin(q2_1) , 2)
        nor_2 = pow((a3*cos(q2_2)+a2), 2) + pow(a3*sin(q2_2) , 2)
        c1_1 = (x*(a3*cos(q2_1) + a2) + y*a3*sin(q2_1)) / nor_1
        c1_2 = (x*(a3*cos(q2_2) + a2) + y*a3*sin(q2_2)) / nor_2
        s1_1 = (y*(a3*cos(q2_1) + a2) - x*a3*sin(q2_1)) / nor_1
        s1_2 = (y*(a3*cos(q2_2) + a2) - x*a3*sin(q2_2)) / nor_2
        q1_1 = atan2(s1_1, c1_1)
        q1_2 = atan2(s1_2, c1_2)
        
        # Checking which q1, q2 pair is correct
        # Second set of solution
        calculated_psi = q2_2 + q1_2
        if abs(calculated_psi-psi) < 0.01:
            q1 = q1_2
            q2 = q2_2
        else:
            q1 = q1_1
            q2 = q2_1
    except ValueError, e:
        print "IK failed, the coordinate x, y provided may be invalid: %s"%e
        return AdeeptKinIKResponse(False, 0, 0, 0)

    return AdeeptKinIKResponse(True, q1, q2, q3)


def handle_forward_kinematics(req):
    # Get Homogeneous Matrix
    rospy.wait_for_service('homogeneous_matrix')
    try:
        get_homo_matrix = rospy.ServiceProxy('homogeneous_matrix', AdeeptHomoMatrix)
        homo_matrix = get_homo_matrix(req.q1, req.q2, req.q3)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    A1 = ma2np(homo_matrix.A1)
    A2 = ma2np(homo_matrix.A2)
    A3 = ma2np(homo_matrix.A3)

    transform = numpy.matmul(numpy.matmul(A1, A2), A3)

    x = transform[0, 3]
    y = transform[1, 3]
    z = transform[2, 3]
    roll = atan2(transform[2, 1], transform[2, 2])
    pitch = atan2((-1 * transform[2, 0]), sqrt(pow(transform[2, 1], 2) + pow(transform[2, 2], 2)))
    yaw = atan2(transform[1, 0], transform[0, 0])

    return AdeeptKinFKResponse(x, y, z, roll, pitch, yaw)


def adeept_kin_server():
    rospy.init_node('kin_server')
    s = rospy.Service('for_kin', AdeeptKinFK, handle_forward_kinematics)
    s = rospy.Service('inv_kin', AdeeptKinIK, handle_inverse_kinematics)

    rospy.spin()


if __name__ == "__main__":
    adeept_kin_server()
