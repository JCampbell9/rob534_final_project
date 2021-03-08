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

    object_tf = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        object_tf.sendTransform((0, .25, .125), tuple(rot3), rospy.Time.now(), 'object_tf', 'world')
        rate.sleep()