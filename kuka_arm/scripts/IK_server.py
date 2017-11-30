#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        # DH parameter
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # Joint angle
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # Link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # Link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # Twist angle

        # Create Modified DH parameters
        s = { alpha0:         0,  a0:      0, d1:  0.75, q1:           q1,
              alpha1:    -pi/2.,  a1:   0.35, d2:     0, q2:  -pi/2. + q2,
              alpha2:         0,  a2:   1.25, d3:     0, q3:           q3,
              alpha3:    -pi/2.,  a3: -0.054, d4:   1.5, q4:           q4,
              alpha4:     pi/2.,  a4:      0, d5:     0, q5:           q5,
              alpha5:    -pi/2.,  a5:      0, d6:     0, q6:           q6,
              alpha6:         0,  a6:      0, d7: 0.303, q7:            0
            }

        # Define Modified DH Transformation matrix
        def get_tf_matrix(alpha, a, d, q):
            return Matrix([
                    [              cos(q),             -sin(q),            0,               a],
                    [   sin(q)*cos(alpha),   cos(q)*cos(alpha),  -sin(alpha),   -sin(alpha)*d],
                    [   sin(q)*sin(alpha),   cos(q)*sin(alpha),   cos(alpha),    cos(alpha)*d],
                    [                   0,                   0,            0,               1]
                ])
        # Create individual transformation matrices
        T0_1 = get_tf_matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = get_tf_matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = get_tf_matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = get_tf_matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = get_tf_matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = get_tf_matrix(alpha5, a5, d6, q6).subs(s)
        T6_E = get_tf_matrix(alpha6, a6, d7, q7).subs(s)

        # Extract rotation matrices from the transformation matrices
        T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E
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

            ### Your IK code here
            # end-effector rotation matrix
            r, p, y = symbols('r p y')

            ROT_x = Matrix([
                            [1,      0,       0],
                            [0, cos(r), -sin(r)],
                            [0, sin(r),  cos(r)]
                        ])
            ROT_y = Matrix([
                            [ cos(p),      0, sin(p)],
                            [      0,      1,      0],
                            [-sin(p),      0,  cos(p)]
                        ])
            ROT_z = Matrix([
                            [ cos(y),-sin(y),      0],
                            [ sin(y), cos(y),      0],
                            [      0,      0,      1]
                        ])

            ROT_E = ROT_z * ROT_y * ROT_x

            Rot_Error = ROT_z.subs( y, radians(180)) * ROT_y.subs( p, radians(-90) )

            ROT_E = ROT_E * Rot_Error
            ROT_E = ROT_E.subs({r: roll, p: pitch, y: yaw})

            EE = Matrix([[px],[py],[pz]])

            WC = EE - (0.303) * ROT_E[:,2]

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1],WC[0])

            # triangle for theta 2, 3
            s_a = 1.501
            s_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            s_c = 1.25

            angle_a = acos((s_b * s_b + s_c * s_c - s_a * s_a) / (2 * s_b * s_c))
            angle_b = acos((s_a * s_a + s_c * s_c - s_b * s_b) / (2 * s_a * s_c))
            angle_c = acos((s_a * s_a + s_b * s_b - s_c * s_c) / (2 * s_a * s_b))

            theta2 = pi/2. - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi/2. - (angle_b + 0.036)

            R0_3 = T0_1[:3,:3] * T1_2[:3,:3] * T2_3[:3,:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3 })

            # R3_6 = R0_3.inv('LU') * ROT_E
            R3_6 = R0_3.transpose() * ROT_E

            # Euler angles from rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

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
