#!/usr/bin/env python

# Import user defined adeept srv
from adeept_command.srv import AdeeptKinFK, AdeeptKinFKResponse,\
                              AdeeptKinIK, AdeeptKinIKResponse,\
                              CheckKinFK, CheckKinFKResponse,\
                              CheckKinIK, CheckKinIKResponse
from helper import acquire_coordinates, acquire_joints
import rospy
import math


# Check if two eular angles are the same
def check_ang(a1, a2):
    same = False
    # convert to between 0 and 2pi
    a1 = a1 % 6.283 
    a2 = a2 % 6.283
    if abs(a1-a2) < 0.03:
	same = True
    # To close the loop 0 -> 2pi -> 0
    if a1 < 0.03 or a2 < 0.03 :
	if abs(a1+a2-6.283) < 0.03:
	    same = True
    return same


# Check if IK server is correct
def handle_check_ik(req):
    # Acquire robot state from gazebo
    gaze_x, gaze_y, gaze_z, gaze_phi, gaze_theta, gaze_psi = acquire_coordinates()
    gaze_q1, gaze_q2, gaze_q3, gaze_q4 = acquire_joints()

    # Using inv_kin service to calculate joint values
    rospy.wait_for_service('inv_kin')
    try:
        inv_kinematic = rospy.ServiceProxy('inv_kin', AdeeptKinIK)
        res = inv_kinematic(gaze_x, gaze_y, gaze_z, gaze_phi, gaze_theta, gaze_psi)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    q1, q2, q3, q4 = res.q1, res.q2, res.q3, res.q4

    # Judge
    correct = False
    if check_ang(q1,gaze_q1) and check_ang(q2,gaze_q2) and \
       check_ang(q3,gaze_q3) and check_ang(q4,gaze_q4):
	correct = True

    # return both results to compare
    print "Check IK success: ", correct
    return CheckKinIKResponse(gaze_q1, gaze_q2, gaze_q3, gaze_q4, q1, q2, q3, q4, correct)


# Check if FK server is correct
def handle_check_fk(req):
    # Acquire robot state from gazebo
    gaze_x, gaze_y, gaze_z, gaze_phi, gaze_theta, gaze_psi = acquire_coordinates()
    gaze_q1, gaze_q2, gaze_q3, gaze_q4 = acquire_joints()
    
    # Using for_kin service to calculate end-of-effector coordinate
    try:
        for_kinematic = rospy.ServiceProxy('for_kin', AdeeptKinFK)
        res = for_kinematic(gaze_q1, gaze_q2, gaze_q3, gaze_q4)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    x, y, z, psi, theta, phi = res.x, res.y, res.z, res.psi, res.theta, res.phi
    
    # Judge
    correct = False
    if abs(x-gaze_x)<0.01 and abs(y-gaze_y)<0.01 and abs(z-gaze_z)<0.01 and \
       check_ang(phi,gaze_phi) and check_ang(theta,gaze_theta) and check_ang(psi,gaze_psi):
	correct = True

    # return both results to compare
    print "Check FK success: ", correct
    return CheckKinFKResponse(gaze_x, gaze_y, gaze_z, gaze_phi, gaze_theta, gaze_psi, 
                            x, y, z, phi, theta, psi, correct)
    

# Connector services
def adeept_connector():
    rospy.init_node('kin_connector')

    s1 = rospy.Service('check_ik', CheckKinIK, handle_check_ik)
    s2 = rospy.Service('check_fk', CheckKinFK, handle_check_fk)

    rospy.spin()


if __name__ == "__main__":
    adeept_connector()
