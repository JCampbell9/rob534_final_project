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

    
    rot3 = tf.transformations.quaternion_from_euler(-pi/2, 0, 0)
    # print(rot3)

    target_trans = (-.12196, -0.0827, .00854)
    target_rot = (-1.09, 1.32, -3.9, 3.01)
    # target_trans = (.11561, -0.06138, -0.02069)
    # target_rot = (-3.79, 3.95, -1.92, 1.35)
    
    angle = tf.transformations.euler_from_quaternion(target_rot)
    # target_trans = (0, 0, 0)
    # angle = (0, pi/2, 0)
    print(angle)
    target_rot2 = tf.transformations.quaternion_from_euler(angle[0], angle[1], angle[2])
    # target_rot = tf.transformations.quaternion_from_euler(-pi/2, 0, 0)
    object_tf = tf.TransformBroadcaster()
    target_tf = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        object_tf.sendTransform((0, .5, .125), tuple(rot3), rospy.Time.now(), 'object_tf', 'world')
        target_tf.sendTransform(target_trans, target_rot2, rospy.Time.now(), 'target_tf', 'object_tf')
        rate.sleep()