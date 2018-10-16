#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def clip_angle(theta, lower, upper):
    return np.clip(theta, radians(lower), radians(upper))

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### FK
        
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 
            
        # Create Modified DH parameters
        DH_table = { alpha0:            0,  a0:      0, d1:    0.75, 
                     alpha1: radians(-90),  a1:   0.35, d2:       0,  q2: q2-radians(90),
                     alpha2:            0,  a2:   1.25, d3:       0,
                     alpha3: radians(-90),  a3: -0.054, d4:    1.50,
                     alpha4:  radians(90),  a4:      0, d5:       0,
                     alpha5: radians(-90),  a5:      0, d6:       0,
                     alpha6:            0,  a6:      0, d7:   0.303}

        # Define Modified DH Transformation matrix
        def DH_TF_Matrix(alpha, a, d, q):
            T = Matrix([[            cos(q),           -sin(q),           0,             a],
                        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                        [                 0,                 0,           0,             1]])
            return T

        # Create individual transformation matrices
        T0_1 = DH_TF_Matrix(alpha0, a0, d1, q1).subs(DH_table)
        T1_2 = DH_TF_Matrix(alpha1, a1, d2, q2).subs(DH_table)
        T2_3 = DH_TF_Matrix(alpha2, a2, d3, q3).subs(DH_table)
        T3_4 = DH_TF_Matrix(alpha3, a3, d4, q4).subs(DH_table)
        T4_5 = DH_TF_Matrix(alpha4, a4, d5, q5).subs(DH_table)
        T5_6 = DH_TF_Matrix(alpha5, a5, d6, q6).subs(DH_table)
        T6_EE = DH_TF_Matrix(alpha6, a6, d7, q7).subs(DH_table)
        
        T0_EE = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE

        # Correction Needed to Account of Orientation Difference Between Definition of
        # Gripper Link in URDF versus DH Convention
        def rot_x(q):
            R_x = Matrix([[      1,        0,        0],
                          [      0,   cos(q),  -sin(q)],
                          [      0,   sin(q),  cos(q)]])
            return R_x
        
        def rot_y(q):              
            R_y = Matrix([[ cos(q),        0,  sin(q)],
                          [      0,        1,       0],
                          [-sin(q),        0, cos(q)]]) 
            return R_y

        def rot_z(q):    
            R_z = Matrix([[ cos(q),  -sin(q),       0],
                          [  sin(q),   cos(q),       0],
                          [       0,        0,      1]])
            return R_z
            
        r, p, y = symbols('r p y')
        R_corr = rot_z(y).subs(y, radians(180))*rot_y(p).subs(p, radians(-90))
        
        R_EE = rot_z(y)*rot_y(p)*rot_x(r)
        R_EE = R_EE * R_corr
    

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

    	    # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### IK code here
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
            EE = Matrix([[px],
                         [py],
                         [pz]])
            WC = EE - DH_table[d7] * R_EE[:, 2]

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1], WC[0])

            # SSS triangle for theta2 and theta3
            side_a = sqrt(DH_table[d4]*DH_table[d4] + DH_table[a3]*DH_table[a3])        
            r = sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - DH_table[a1]
            s = WC[2] - DH_table[d1]            
            side_b = sqrt(r*r + s*s)
            side_c = DH_table[a2]

            angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a)/(2*side_b*side_c))
            angle_b = acos((side_c*side_c + side_a*side_a - side_b*side_b)/(2*side_c*side_a))
                    
            theta2 = radians(90) - angle_a - atan2(s, r)
            theta3 = radians(90) - angle_b - atan2(DH_table[a3], DH_table[d4])

            # Inverse Orientation
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

            R3_6 = R0_3.transpose()*R_EE
            
            # Euler angles from rotation matrix
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            # Choosing between multiple possible solutions:
            if sin(theta5) < 0:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            
            theta1 = clip_angle(theta1, -185, 185)
            theta2 = clip_angle(theta2, -45, 85)
            theta3 = clip_angle(theta3, -210, 65)
            theta4 = clip_angle(theta4, -350, 350)
            theta5 = clip_angle(theta5, -125, 125)
            theta6 = clip_angle(theta6, -350, 350)

            ###


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
