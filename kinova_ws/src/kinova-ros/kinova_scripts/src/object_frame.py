#!/usr/bin/env python


import rospy

import tf
import os
import numpy as np
from math import pi
import csv
import glob
import sys


class Frames():
    def __init__(self):
        rospy.init_node('object_frame_tf')

        self.dir_path = os.path.dirname(os.path.realpath(__file__))

        self.pose_list = []

        self.object = rospy.get_param('test_object')
        self.enviroment = rospy.get_param('test_env')
        self.pose_number = rospy.get_param('goal_pose')
        self.ready_trig = rospy.get_param('ready_trig')
        
        ###################################################################################
        # Creates frames for the object and target pose 
        ###################################################################################
        
        self.listener = tf.TransformListener()

        while True:
            try:
                self.translation, self.rotation = self.listener.lookupTransform('j2s7s300_link_7', 'j2s7s300_end_effector', rospy.Time())
                break  # once the transform is obtained move on
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue  # if it fails try again
        
        self.object_y = [0.25, 0.5, 0.75]
        self.object_tf = tf.TransformBroadcaster()
        self.target_tf = tf.TransformBroadcaster()
        self.target_tf_goal = tf.TransformBroadcaster()
        self.goal_tf = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)
        self.rot3 = tf.transformations.quaternion_from_euler(-pi/2, 0, 0)
        self.goal_angle = tf.transformations.quaternion_from_euler(pi, -pi/2, 0)
        self.read_poses()
        self.get_goal_pose()
        self.publish_tfs()
    
    
    def read_poses(self):
        with open(self.dir_path + '/{}_pose_data_1pose.csv'.format(self.object), 'r') as csvfile:
            ofile = csv.reader(csvfile, delimiter=',')
            next(ofile)

            for row in ofile:
                self.pose_list.append([float(i) for i in row])
            
                
    def get_goal_pose(self):

        self.enviroment = rospy.get_param('test_env')
        goal_pose = rospy.get_param('goal_pose')
        if goal_pose >= len(self.pose_list):
            goal_pose = 0
        # else:
        rospy.logerr('pose number: {}'.format(goal_pose))
        
        trans = self.pose_list[goal_pose][:3]
        target_rot = self.pose_list[goal_pose][3:]
        rospy.logerr(self.pose_list[goal_pose])
        self.target_trans = [j/1000 for j in trans]

        angle = tf.transformations.euler_from_quaternion((target_rot[1], target_rot[2], target_rot[3], target_rot[0])) 
        self.target_rot2 = tf.transformations.quaternion_from_euler(angle[0], angle[1], angle[2])

        if goal_pose >= (len(self.pose_list) - 1):
            rospy.set_param('ready_trig', 3)
            rospy.logerr('set trigger to 3')
        else:
            rospy.set_param('ready_trig', 1)

        

    def publish_tfs(self):



        while not rospy.is_shutdown():

            if rospy.get_param('ready_trig') == 4:
                self.get_goal_pose()

            self.object_tf.sendTransform((0, self.object_y[int(self.enviroment)], .125), tuple(self.rot3), rospy.Time.now(), 'object_tf', 'world')
            self.target_tf.sendTransform(self.target_trans, self.target_rot2, rospy.Time.now(), 'target_tf', 'object_tf')
            self.target_tf_goal.sendTransform((0.012, 0, 0), tuple(self.goal_angle), rospy.Time.now(), 'target_tf_goal', 'target_tf') # 'j2s7s300_link_7') # 'target_tf'
            self.goal_tf.sendTransform(tuple(self.translation), self.rotation, rospy.Time.now(), 'goal_tf', 'target_tf_goal')
            self.rate.sleep()



if __name__ == '__main__':
    Frames()
    # print(rot3)

    # dir_path = os.path.dirname(os.path.realpath(__file__))
    # file = open(dir_path + "/results.csv", "r")
    # wr = csv.writer(file, dialect='excel')

    # pose_num = 10

    # wr.writerow(["cylinder", planner, pose_num, self.env, result, run_time]) # [object, planner, pose, env, fail/success, time]
    # file.close()

    # # pose 1
    # target_trans = (.11561, -0.06138, -0.02069)
    # target_rot = (3.95, -1.92, 1.35, -3.79)
    
    # # pose 2
    # target_trans = (-.12196, -0.0827, .00854)
    # target_rot = (1.32, -3.9, 3.01, -1.09)
    
    # # pose 3
    # target_trans = (-0.13351, -0.12192, -0.04306)
    # target_rot = (0.69, -4.54, 4.01, -2.02)

    # pose 4
    # target_trans = (.09369, -0.18366, -0.05956)
    # target_rot = (3.81, -2.51, 0.77, -3.22)

    # # pose 5
    # target_trans = (-0.12057, -0.10617, 0.00928)
    # target_rot = (1.37, -3.11, 3.66, -1.05)

    # # pose 6
    # target_trans = (0.10245, .09025, 0.09029)
    # target_rot = (4.51, 2.37, 0.52, -3.17)

    # # pose 7
    # target_trans = (0.11042, -0.02369, -0.09094)
    # target_rot = (4.18, -1.31, -1.39, -3.97)

    # # pose 8
    # target_trans = (0.12979, .07966, -0.06337)
    # target_rot = (4.66, -0.56, -1.44, -4.31)

    # #pose 9
    # target_trans = (0.09808, .00654, -0.10785)
    # target_rot = (2.84, -1.37, -1.17, -2.79)

    # # pose 10
    # target_trans = (-0.05336, -0.0052, -.1382)
    # target_rot = (2.42, -3.21, -2.88, -1.97)
    
    # target_trans = (.09711, -.12544, -.037)
    # target_rot = (2.92, -2.45, 1.94, -3.92)


    # target_trans = (-0.0536, -.00852, -.1382)
    # target_rot = (-1.97, 2.42, -3.21, -2.88)

    # target_trans = (0.04201, -.04384, .0906)
    # angle = (1.67, -1.15, (2.69 + pi))
    
    # target_trans = (-.01954, -.07889, -.07358)
    # angle = (.13, -.70, (2.55))

    # angle = tf.transformations.euler_from_quaternion(target_rot)
    # # target_trans = (0, 0, 0)
    # # angle = (0, pi/2, 0)
    # # print(angle)
    # target_rot2 = tf.transformations.quaternion_from_euler(angle[0], angle[1], angle[2]) #(angle[2] + pi))
    # # target_rot = tf.transformations.quaternion_from_euler(-pi/2, 0, 0)
    # goal_angle = tf.transformations.quaternion_from_euler(pi, -pi/2, 0)
    
    # object_tf = tf.TransformBroadcaster()
    # target_tf = tf.TransformBroadcaster()
    # target_tf_goal = tf.TransformBroadcaster()
    # goal_tf = tf.TransformBroadcaster()
    # rate = rospy.Rate(10.0)

    # while not rospy.is_shutdown():

        

    #     object_tf.sendTransform((0, .5, .125), tuple(rot3), rospy.Time.now(), 'object_tf', 'world')
    #     target_tf.sendTransform(target_trans, target_rot2, rospy.Time.now(), 'target_tf', 'object_tf')
    #     target_tf_goal.sendTransform((0.012, 0, 0), tuple(goal_angle), rospy.Time.now(), 'target_tf_goal', 'target_tf') # 'j2s7s300_link_7') # 'target_tf'
    #     goal_tf.sendTransform(tuple(translation), rotation, rospy.Time.now(), 'goal_tf', 'target_tf_goal')
    #     rate.sleep()