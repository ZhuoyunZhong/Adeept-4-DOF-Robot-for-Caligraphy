#!/usr/bin/env python

from adeept_command.srv import AdeeptVelFK, AdeeptVelFKResponse, AdeeptHomoMatrix, AdeeptHomoMatrixResponse, AdeeptVelIK, AdeeptVelIKResponse
import rospy
import numpy
import rosservice
from helper import ma2np
from math import pi, atan2, sqrt

def homogeneous_A(a1, alph1, d1, theta1):
    A = numpy.array([[numpy.cos(theta1), (numpy.sin(theta1)*-1) * numpy.cos(alph1), numpy.sin(theta1) * numpy.sin(alph1), a1 * numpy.cos(theta1)],
                      [numpy.sin(theta1), numpy.cos(theta1) * numpy.cos(alph1), (numpy.cos(theta1)*-1) * numpy.sin(alph1), a1 * numpy.sin(theta1)],
                      [0, numpy.sin(alph1), numpy.cos(alph1), d1],
                      [0, 0, 0, 1]])
    return A

def get_Jacobian(q1,q2,q3,q4):
    pen = 0

    #Now to find A1 A2 A3..
    a1 = 0
    alph1 = pi / 2
    d1 = 0.100
    theta1 = q1

    a2 = 0.065
    alph2 = pi
    d2 = 0
    theta2 = q2 + pi / 2

    a3 = 0
    alph3 = pi / 2
    d3 = 0
    theta3 = q3 + pi

    a4 = 0
    alph4 = 0
    d4 = 0.055
    theta4 = q4

    a5 = 0.013+pen
    alph5 = 0
    d5 = 0.0665
    theta5 = 0

    A1 = homogeneous_A(a1, alph1, d1, theta1)
    A2 = homogeneous_A(a2, alph2, d2, theta2)
    A3 = homogeneous_A(a3, alph3, d3, theta3)
    A4 = homogeneous_A(a4, alph4, d4, theta4)
    A5 = homogeneous_A(a5, alph5, d5, theta5)




    z0 = numpy.array([[0], [0], [1]])
    z1 = A1[0:3, 2]
    A12 = numpy.dot(A1, A2)
    A13 = numpy.dot(A12, A3)
    A15 = numpy.dot(A13, numpy.dot(A4, A5))
    z2 = A12[0:3, 2]
    z3 = A13[0:3, 2]
    z4 = A15[0:3, 2]

    o4 = A15[0:3, 3]
    o3 = A13[0:3, 3]
    o2 = A12[0:3, 3]
    o1 = A1[0:3, 3]
    Jv = numpy.column_stack((numpy.cross(z0.T, o4.T).T, numpy.cross(z1.T, (o4.T - o1.T)).T, numpy.cross(z2.T, (o4.T - o2.T)).T,
                          numpy.cross(z3.T, (o4.T - o3.T)).T))
    Jw = numpy.column_stack((z0, z1, z2, z3))
    J = numpy.vstack((Jv, Jw))
    return(J)

# Handle joints velocity -> cartesian velocity...
def handle_velocity_forward_kinematics(req):
    print "Received Forward Request"
    q1_dot = req.q1_dot
    q2_dot = req.q2_dot
    q3_dot = req.q3_dot
    q4_dot = req.q4_dot
    qdot = numpy.array([[q1_dot],[q2_dot],[q3_dot],[q4_dot]])
    #[q1,q2,q3] = acquire_joints()..
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3
    q4 = req.q4

    J = get_Jacobian(q1,q2,q3,q4)
    V = numpy.dot(J, qdot)
    Vx = V[0]
    Vy = V[1]
    Vz = V[2]
    Wx = V[3]
    Wy = V[4]
    Wz = V[5]

    print "Completed Forward Vel Kinem"
    return AdeeptVelFKResponse(Vx, Vy, Vz, Wx, Wy, Wz)


# Handle cartesian velocity -> joints velocity
def handle_velocity_inverse_kinematics(req):
    print "Received Inverse Request"

    Vx = req.Vx
    Vy = req.Vy    
    Vz = req.Vz

    Wx = req.Wx
    Wy = req.Wy
    Wz = req.Wz

     #   V = numpy.array([[Vx], [Vy], [Vz], [Wx], [Wy], [Wz]])
    V = numpy.array([Vx,Vy,Vz,Wx,Wy,Wz])
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3
    q4 = req.q4
    J = get_Jacobian(q1,q2,q3,q4)
    Jinv = numpy.linalg.pinv(J)
    q = numpy.dot(Jinv, V)
    print(q)
    q1_dot = q[0]
    q2_dot = q[1]
    q3_dot = q[2]
    q4_dot = q[3]
    print "Completed Inverse Kin"
    return AdeeptVelIKResponse(True, q1_dot, q2_dot, q3_dot, q4_dot)

# Server
def adeept_vk_server():
    rospy.init_node('vel_kin_server')
    s1 = rospy.Service('vel_for_kin', AdeeptVelFK, handle_velocity_forward_kinematics)
    s2 = rospy.Service('vel_inv_kin', AdeeptVelIK, handle_velocity_inverse_kinematics)
    print "Ready for Vel Kin"
    
    rospy.spin()


if __name__ == "__main__":
    adeept_vk_server()
