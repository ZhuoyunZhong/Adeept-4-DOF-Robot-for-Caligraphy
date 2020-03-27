#!/usr/bin/env python
import rospy
import numpy
from math import sin, cos, acos, atan2, pi, sqrt
from adeept_command.srv import AdeeptKinIK, AdeeptKinIKResponse


#Assuming that input is x,y,z,phi,theta,psi and output is q1,q2,q3,q4

def rotation_mat(alph1, theta1):
    R = numpy.array([[cos(theta1), -sin(theta1) * cos(alph1), sin(theta1) * sin(alph1)],
                     [sin(theta1), cos(theta1) * cos(alph1), -cos(theta1) * sin(alph1)],
                     [0, sin(alph1), cos(alph1)]])
    return R

def handle_IK(req):
    # Robot dimension
    pen = 13
    link3d = 65
    link4d = 121.5
    # Input Coordinate
    x = req.x
    y = req.y
    z = req.z
    phi = req.phi
    theta = req.theta
    psi = req.psi

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
        # Calculate position of projection onto axis of rotation of joint 4: displacement of 13 + additional pen length in final x direction

	wrist_x = x-pen*(cphi*ctheta)
  	wrist_y = y-pen*(sphi*ctheta)
	wrist_z = z+pen*stheta

        # May need to adjust this for quadrant based on joint limits

        q1 = round(atan2(wrist_y, wrist_x),4)

        q3d = (wrist_x ** 2 + wrist_y ** 2 + (wrist_z - 100) ** 2 - link3d ** 2 - link4d ** 2) / (2 * link3d * link4d)
        temp3_1 = atan2(sqrt(1-q3d**2), q3d)
        temp3_2 = atan2(-sqrt(1-q3d**2),q3d)
        q3_1 = round(-pi/2 - temp3_1, 4)
        q3_2 = round(-pi/2 - temp3_2, 4)

        q2a = atan2(wrist_z-100,sqrt(wrist_x**2+wrist_y**2))
        q2_1b = atan2(link4d*sin(temp3_1), (link3d + link4d*cos(temp3_1)))
        q2_2b = atan2(link4d*sin(temp3_2), (link3d + link4d*cos(temp3_2)))
        temp2_1 = q2a-q2_1b
        temp2_2 = q2a-q2_2b
        q2_1 = -round(pi/2-temp2_1, 4)
        q2_2 = -round(pi/2-temp2_2, 4)


	# Find candidate rotation matrices R_03
        R1 = numpy.round(rotation_mat(pi / 2, q1), 4)

        R2_1 = numpy.round(rotation_mat(pi, q2_1+pi/2), 4)
        R2_2 = numpy.round(rotation_mat(pi, q2_2+pi/2), 4)

        R3_1 = numpy.round(rotation_mat(pi / 2, q3_1+pi), 4)
        R3_2 = numpy.round(rotation_mat(pi / 2, q3_2+pi), 4)

        R_03_1 = numpy.matmul(numpy.matmul(R1, R2_1), R3_1)
        R_03_2 = numpy.matmul(numpy.matmul(R1, R2_2), R3_2)

        # Determine which solution matches the input pose
        R_34_1 = numpy.matmul(numpy.transpose(R_03_1), R_04)
        R_34_2 = numpy.matmul(numpy.transpose(R_03_2), R_04)
        if abs(R_34_1[0, 2]) < 0.01 and abs(R_34_1[1, 2]) < 0.01:
            q4 = round(atan2(R_34_1[1, 0], R_34_1[0, 0]), 4)
            q3 = q3_1
            q2 = q2_1
        elif abs(R_34_2[0, 2]) < 0.01 and abs(R_34_2[1, 2]) < 0.01:
            q4 = round(atan2(R_34_2[1, 0], R_34_2[0, 0]), 4)
            q3 = q3_2
            q2 = q2_2
        else:
            print("IK failed, something went wrong.")
            return AdeeptKinIKResponse(False, 0, 0, 0, 0)

    except ValueError, e:
        print "IK failed, the coordinate x, y provided may be invalid: %s"%e
        return AdeeptKinIKResponse(False, 0, 0, 0, 0)
    print "trying to return values"
    return AdeeptKinIKResponse(True, q1, q2, q3, q4)


def adeept_IK_server():
    rospy.init_node('inv_kin_server')
    s = rospy.Service('inv_kin', AdeeptKinIK, handle_IK)
    
    rospy.spin()


if __name__ == "__main__":
    adeept_IK_server()
