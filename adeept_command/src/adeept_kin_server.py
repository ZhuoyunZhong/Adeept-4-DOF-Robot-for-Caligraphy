#!/usr/bin/env python

from adeept_command.srv import AdeeptHomoMatrix
from adeept_command.srv import AdeeptKinFK, AdeeptKinFKResponse
from adeept_command.srv import AdeeptKinIK, AdeeptKinIKResponse
from math import atan2, sqrt, pow, sin, cos, acos, atan2, pi
import numpy
from helper import ma2np

import rospy


def handle_inverse_kinematics(req):
    # Robot dimension
    pen = 0.013 + 0
    link3d = 0.065
    link4d = 0.1215
    # Input Coordinate
    x = req.x
    y = req.y
    z = req.z
    phi = req.phi
    theta = req.theta
    psi = req.psi

    # TODO
    # Ask for service but does not give a specific orientation
    if phi == 0 and theta == 0 and psi == 0:
        # Remain horizontal
        phi = atan2(y, x)
        theta = pi/2 
        psi = 0
    
    cphi = cos(phi)
    sphi = sin(phi)
    ctheta = cos(theta)
    stheta = sin(theta)
    cpsi = cos(psi)
    spsi = sin(psi)
    R_04 = numpy.array([[cphi*ctheta, -sphi*cpsi+cphi*stheta*spsi, sphi*spsi+cphi*stheta*cpsi],
                        [sphi*ctheta, cphi*cpsi+sphi*stheta*spsi, -cphi*spsi+sphi*stheta*cpsi],
                        [-stheta, ctheta*spsi, ctheta*cpsi]])

    try:
        # Calculate position of projection onto axis of rotation of joint 4: 
        # displacement of 13 + additional pen length in final x direction
        wrist_x = round(x - pen*(cphi*ctheta), 4)
        wrist_y = round(y - pen*(sphi*ctheta), 4)
        wrist_z = round(z + pen*stheta, 4)

        # May need to adjust this for quadrant based on joint limits
        q1 = round(atan2(wrist_y, wrist_x), 3)

        q3d = (wrist_x**2 + wrist_y**2 + (wrist_z-0.1)**2 - link3d**2 - link4d**2) / \
              (2 * link3d * link4d)
        temp3_1 = atan2(sqrt(1 - q3d**2), q3d)
        temp3_2 = atan2(-sqrt(1 - q3d**2), q3d)
        q3_1 = round(- pi/2 - temp3_1, 4)
        q3_2 = round(- pi/2 - temp3_2, 4)

        q2a = atan2(wrist_z-0.1, sqrt(wrist_x**2+wrist_y**2))
        q2_1b = atan2(link4d*sin(temp3_1), (link3d + link4d*cos(temp3_1)))
        q2_2b = atan2(link4d*sin(temp3_2), (link3d + link4d*cos(temp3_2)))
        temp2_1 = q2a - q2_1b
        temp2_2 = q2a - q2_2b
        q2_1 = -round(pi/2-temp2_1, 4)
        q2_2 = -round(pi/2-temp2_2, 4)

	    # Find candidate rotation matrices R_03
        try:
            get_homo_matrix = rospy.ServiceProxy('homogeneous_matrix', AdeeptHomoMatrix)
            homo_matrix_1 = get_homo_matrix(q1, q2_1, q3_1, 0)
            homo_matrix_2 = get_homo_matrix(q1, q2_2, q3_2, 0)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        A1_1 = ma2np(homo_matrix_1.A1)
        A2_1 = ma2np(homo_matrix_1.A2)
        A3_1 = ma2np(homo_matrix_1.A3)
        R_03_1 = numpy.matmul(numpy.matmul(A1_1, A2_1), A3_1)[0:3, 0:3]

        A1_2 = ma2np(homo_matrix_2.A1)
        A2_2 = ma2np(homo_matrix_2.A2)
        A3_2 = ma2np(homo_matrix_2.A3)
        R_03_2 = numpy.matmul(numpy.matmul(A1_2, A2_2), A3_2)[0:3, 0:3]

        A4 = ma2np(homo_matrix_2.A4)[0:3, 0:3]

        # Determine which solution matches the input pose
        R_34_1 = numpy.matmul(numpy.transpose(R_03_1), R_04)
        R_34_2 = numpy.matmul(numpy.transpose(R_03_2), R_04)

        if abs(R_34_1[0, 2]) < 0.03 and abs(R_34_1[1, 2]) < 0.03:
            q4 = round(atan2(R_34_1[1, 0], R_34_1[0, 0]), 4)
            q3 = q3_1
            q2 = q2_1
        elif abs(R_34_2[0, 2]) < 0.03 and abs(R_34_2[1, 2]) < 0.03:
            q4 = round(atan2(R_34_2[1, 0], R_34_2[0, 0]), 4)
            q3 = q3_2
            q2 = q2_2
        else:
            print("IK failed, something went wrong.")
            return AdeeptKinIKResponse(False, 0, 0, 0, 0)
        print(q1, q2, q3, q4)
    except ValueError, e:
        print "IK failed, the coordinate x, y provided may be invalid: %s"%e
        return AdeeptKinIKResponse(False, 0, 0, 0, 0)
    print "trying to return values"
    return AdeeptKinIKResponse(True, q1, q2, q3, q4)


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
    x = transform[0, 3]
    y = transform[1, 3]
    z = transform[2, 3]
    yaw = atan2(round(transform[2, 1],4), round(transform[2, 2],4))
    pitch = atan2(round((-1 * transform[2, 0]),4), round(sqrt(pow(transform[2, 1], 2) + \
            pow(transform[2, 2], 2)),4))
    roll = atan2(round(transform[1, 0],4), round(transform[0, 0],4))

    return AdeeptKinFKResponse(x, y, z, roll, pitch, yaw)


def adeept_kin_server():
    rospy.init_node('kin_server')
    s = rospy.Service('for_kin', AdeeptKinFK, handle_forward_kinematics)
    s = rospy.Service('inv_kin', AdeeptKinIK, handle_inverse_kinematics)

    rospy.spin()


if __name__ == "__main__":
    adeept_kin_server()
