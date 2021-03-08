#!/usr/bin/env python


import rospy

import tf
import os
import numpy as np
from math import pi
import csv
import glob
import sys




if __name__ == '__main__':

    ###################################################################################
    # Creates the camera, world, and the aruco_end_effector frames to TF to be used later 
    ###################################################################################

    rospy.init_node('object_frame_tf')

    directory = os.path.dirname(os.path.realpath(__file__))

    listener = tf.TransformListener()

    while True:
        try:
            translation, rotation = listener.lookupTransform('j2s7s300_link_7', 'j2s7s300_end_effector', rospy.Time())
            print(translation)
            break  # once the transform is obtained move on
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue  # if it fails try again

    
    rot3 = tf.transformations.quaternion_from_euler(-pi/2, 0, 0)
    # print(rot3)

    # # pose 1
    # target_trans = (.11561, -0.06138, -0.02069)
    # target_rot = (-3.79, 3.95, -1.92, 1.35)
    
    # # pose 2
    # target_trans = (-.12196, -0.0827, .00854)
    # target_rot = (-1.09, 1.32, -3.9, 3.01)
    
    # # pose 3
    # target_trans = (-0.13351, -0.12192, -0.04306)
    # target_rot = (-2.02, 0.69, -4.54, 4.01)

    # # pose 4
    # target_trans = (.09369, -0.18366, -0.05956)
    # target_rot = (-3.22, 3.81, -2.51, 0.77)

    # # pose 5
    # target_trans = (-0.12057, -0.10617, 0.00928)
    # target_rot = (-1.05, 1.37, -3.11, 3.66)

    # # pose 6
    # target_trans = (0.10245, .09025, 0.09029)
    # target_rot = (-3.17, 4.51, 2.37, 0.52)

    # # pose 7
    # target_trans = (0.11042, -0.02369, -0.09094)
    # target_rot = (-3.97, 4.18, -1.31, -1.39)

    # # pose 8
    # target_trans = (0.12979, .07966, -0.06337)
    # target_rot = (-4.31, 4.66, -0.56, -1.44)

    # #pose 9
    # target_trans = (0.09808, .00654, -0.10785)
    # target_rot = (-2.79, 2.84, -1.37, -1.17)

    # pose 10
    target_trans = (-0.05336, -0.0052, -.1382)
    target_rot = (-1.97, 2.42, -3.21, -2.88)
    
    # target_trans = (.09711, -.12544, -.037)
    # target_rot = (-3.92, 2.92, -2.45, 1.94)


    # target_trans = (-0.0536, -.00852, -.1382)
    # target_rot = (-1.97, 2.42, -3.21, -2.88)

    # target_trans = (0.04201, -.04384, .0906)
    # angle = (1.67, -1.15, (2.69 + pi))
    
    # target_trans = (-.01954, -.07889, -.07358)
    # angle = (.13, -.70, (2.55))

    angle = tf.transformations.euler_from_quaternion(target_rot)
    # target_trans = (0, 0, 0)
    # angle = (0, pi/2, 0)
    # print(angle)
    target_rot2 = tf.transformations.quaternion_from_euler(angle[0], angle[1], (angle[2] + pi))
    # target_rot = tf.transformations.quaternion_from_euler(-pi/2, 0, 0)
    goal_angle = tf.transformations.quaternion_from_euler(pi, -pi/2, 0)
    
    object_tf = tf.TransformBroadcaster()
    target_tf = tf.TransformBroadcaster()
    target_tf_goal = tf.TransformBroadcaster()
    goal_tf = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        object_tf.sendTransform((0, .5, .125), tuple(rot3), rospy.Time.now(), 'object_tf', 'world')
        target_tf.sendTransform(target_trans, target_rot2, rospy.Time.now(), 'target_tf', 'object_tf')
        target_tf_goal.sendTransform((0.1, 0, 0), tuple(goal_angle), rospy.Time.now(), 'target_tf_goal', 'target_tf') # 'j2s7s300_link_7') # 'target_tf'
        goal_tf.sendTransform(tuple(translation), rotation, rospy.Time.now(), 'goal_tf', 'target_tf_goal')
        rate.sleep()