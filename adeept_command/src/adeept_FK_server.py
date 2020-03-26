#!/usr/bin/env python

from adeept_command.srv import AdeeptKinFK, AdeeptKinFKResponse, AdeeptHomoMatrix
from math import atan2, sqrt, pow
import numpy
from helper import ma2np

import rospy


def handle_forward_kinematics(req):
    # Get Homogeneous Matrix
    rospy.wait_for_service('homogeneous_matrix')
    try:
        get_homo_matrix = rospy.ServiceProxy('homogeneous_matrix', AdeeptHomoMatrix)
        homo_matrix = get_homo_matrix(req.q1, req.q2, req.q3, req.q4)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    A1 = ma2np(homo_matrix.A1)
    A2 = ma2np(homo_matrix.A2)
    A3 = ma2np(homo_matrix.A3)
    A4 = ma2np(homo_matrix.A4)
    A5 = ma2np(homo_matrix.A5)

    transform = numpy.matmul(numpy.matmul(numpy.matmul(numpy.matmul(A1, A2), A3), A4), A5)

    transform = numpy.matmul(numpy.matmul(numpy.matmul(numpy.matmul(A1, A2), A3), A4), A5)
    x = transform[0, 3]
    y = transform[1, 3]
    z = transform[2, 3]
    yaw = atan2(round(transform[2, 1],4), round(transform[2, 2],4))
    pitch = atan2(round((-1 * transform[2, 0]),4), round(sqrt(pow(transform[2, 1], 2) + pow(transform[2, 2], 2)),4))
    roll = atan2(round(transform[1, 0],4), round(transform[0, 0],4))

    return AdeeptKinFKResponse(x, y, z, roll, pitch, yaw)


def adeept_FK_server():
    rospy.init_node('for_kin_server')
    s = rospy.Service('for_kin', AdeeptKinFK, handle_forward_kinematics)

    rospy.spin()


if __name__ == "__main__":
    adeept_FK_server()
