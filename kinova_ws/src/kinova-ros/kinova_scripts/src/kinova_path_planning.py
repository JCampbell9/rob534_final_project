#! /usr/bin/env python

# Author: Nuha Nishat
# Date: 1/30/20

import rospy
import sys, os
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf, math
import tf.transformations
import pdb
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi 
from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene

# from kinova_scripts.srv import Joint_angles, Joint_anglesResponse
# from kinova_scripts.srv import New_pose, New_poseResponse
import time
import glob
import csv




# move_group_python_interface_tutorial was used as reference

class MoveRobot():
	def __init__(self, env=0):
		# Initialize moveit commander and ros node for moveit
		moveit_commander.roscpp_initialize(sys.argv)

		# Initializing node
		rospy.init_node("move_kinova", anonymous=True)

		# Define robot using RobotCommander. Provided robot info such as
		# kinematic model and current joint state
		self.robot = moveit_commander.RobotCommander()

		# Setting the world 
		self.scene = moveit_commander.PlanningSceneInterface()
		self.env = env
		rospy.sleep(2)
		table_pose = PoseStamped()
		table_pose.header.frame_id = self.robot.get_planning_frame()
		table_pose.pose.position.x = 0.
		table_pose.pose.position.y = 0.
		table_pose.pose.position.z = -0.0125
		self.scene.add_box('table', table_pose, (1, 3, 0.025))



		if self.env == 0:
			cyl_pose = PoseStamped()
			cyl_pose.header.frame_id = self.robot.get_planning_frame()
			cyl_pose.pose.position.x = 0.0
			cyl_pose.pose.position.y = 0.5
			cyl_pose.pose.position.z = 0.125
			self.scene.add_cylinder('cylinder', cyl_pose, .25, 0.025)
			obj_pose = (0, .5, .125/2)

		elif self.env == 1:

			cyl_pose = PoseStamped()
			cyl_pose.header.frame_id = self.robot.get_planning_frame()
			cyl_pose.pose.position.x = 0.0
			cyl_pose.pose.position.y = 0.5
			cyl_pose.pose.position.z = 0.125
			self.scene.add_cylinder('cylinder', cyl_pose, .25, 0.025)
			obj_pose = (0, .5, .125/2)
			# go around or over wall
			wall_pose = PoseStamped()
			wall_pose.header.frame_id = self.robot.get_planning_frame()
			wall_pose.pose.position.x = 0.3
			wall_pose.pose.position.y = 0.25
			wall_pose.pose.position.z = 0.25
			self.scene.add_box('wall', wall_pose, (1, 0.025, .5))

		elif self.env == 2:

			cyl_pose = PoseStamped()
			cyl_pose.header.frame_id = self.robot.get_planning_frame()
			cyl_pose.pose.position.x = 0.0
			cyl_pose.pose.position.y = 0.75
			cyl_pose.pose.position.z = 0.125
			obj_pose = (0, .75, .125/2)
			self.scene.add_cylinder('cylinder', cyl_pose, .25, 0.025)

			wallL_pose = PoseStamped()
			wallL_pose.header.frame_id = self.robot.get_planning_frame()
			wallL_pose.pose.position.x = -0.45
			wallL_pose.pose.position.y = 0.5
			wallL_pose.pose.position.z = 0.5
			self.scene.add_box('wallL', wallL_pose, (0.5, 0.025, 1))

			wallR_pose = PoseStamped()
			wallR_pose.header.frame_id = self.robot.get_planning_frame()
			wallR_pose.pose.position.x = 0.45
			wallR_pose.pose.position.y = 0.5
			wallR_pose.pose.position.z = 0.5
			self.scene.add_box('wallR', wallR_pose, (0.5, 0.025, 1))

			wallT_pose = PoseStamped()
			wallT_pose.header.frame_id = self.robot.get_planning_frame()
			wallT_pose.pose.position.x = 0.0
			wallT_pose.pose.position.y = 0.5
			wallT_pose.pose.position.z = 1.
			self.scene.add_box('wallT', wallT_pose, (0.5, 0.025, 1))

		
		# Define the planning group for the arm you are using
		# You can easily look it up on rviz under the MotionPlanning tab
		self.move_group = moveit_commander.MoveGroupCommander("arm")
		self.move_gripper = moveit_commander.MoveGroupCommander("gripper")
		
		# Set the precision of the robot
		rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
		
		rospy.wait_for_service("/apply_planning_scene", 10.0)
		rospy.wait_for_service("/get_planning_scene", 10.0)

		self.apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
		self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
		rospy.sleep(2)

		# object_tf = tf.TransformBroadcaster()
		# rot = tf.transformations.quaternion_from_euler(pi/2, 0, 0)
		# object_tf.sendTransform(obj_pose, tuple(rot), rospy.Time.now(), 'object_frame', 'j2s7s300_link_base')

		
		# To see the trajectory
		self.disp = moveit_msgs.msg.DisplayTrajectory()

		self.disp.trajectory_start = self.robot.get_current_state()
		
		self.rate = rospy.Rate(10)

		self.move_group.allow_replanning(0)
		# self.move_group.allow_replanning()

		# self.joint_angles_service = rospy.Service('joint_angles', Joint_angles, self.joint_angles)
		# self.new_pose_service = rospy.Service('new_pose', New_pose, self.new_pose)

		self.main()
	
	def set_planner_type(self, planner_name):
		if planner_name == "RRT":
			self.move_group.set_planner_id("RRTConnectkConfigDefault")
		if planner_name == "RRT*":
			self.move_group.set_planner_id("RRTstarkConfigDefault")
		if planner_name == "PRM*":
			self.move_group.set_planner_id("PRMstarkConfigDefault")


	def go_to_joint_state(self, joint_state):
		joint_goal = JointState()
		joint_goal.position = joint_state
		self.move_group.set_joint_value_target(joint_goal.position)

		self.plan = self.move_group.plan()
		self.move_group.go(wait=True)
		self.move_group.execute(self.plan, wait=True)

		self.move_group.stop()
		self.move_group.clear_pose_targets()
		rospy.sleep(2)

	def go_to_goal(self, ee_pose):
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = ee_pose[0]
		pose_goal.position.y = ee_pose[1]
		pose_goal.position.z = ee_pose[2]
		
		if len(ee_pose) == 6:
			# quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]), math.radians(ee_pose[5]))
			quat = tf.transformations.quaternion_from_euler(ee_pose[3], ee_pose[4], ee_pose[5])
			pose_goal.orientation.x = quat[0]
			pose_goal.orientation.y = quat[1]
			pose_goal.orientation.z = quat[2]
			pose_goal.orientation.w = quat[3]

		else:
			pose_goal.orientation.x = ee_pose[3]
			pose_goal.orientation.y = ee_pose[4]
			pose_goal.orientation.z = ee_pose[5]
			pose_goal.orientation.w = ee_pose[6]	

		self.move_group.set_pose_target(pose_goal)
		self.move_group.set_planning_time(1)
		rospy.sleep(2)
		print(self.move_group.go(wait=True))
		self.move_group.stop()

		self.move_group.clear_pose_targets()
		rospy.sleep(2)

	def move_gripper(self, cmd):
		if cmd == "Close":
			self.move_gripper.set_named_target("Close")
		elif cmd == "Open":
			self.move_gripper.set_named_target("Open")
		else: 
			self.move_gripper.set_joint_value_target(cmd)
		self.move_gripper.go(wait=True)
		rospy.sleep(2)

	def display_trajectory(self):
		self.disp_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		self.disp.trajectory.append(self.plan)
		print(self.disp.trajectory)
		self.disp_pub.publish(self.disp)


	# def new_pose(self, request):
	# 	self.main(0, request.pose)
		
	# 	return New_poseResponse(True)


	# def joint_angles(self, request):
	# 	"""calls the main() function giving it the desired joint angles"""
	# 	self.main(1, request.angles)

	# 	return Joint_anglesResponse(True)

	# def main(self, trig, goal):

	# 	# Set up path here

	# 	# Pick planner 
	# 	self.set_planner_type("RRT")

	# 	if trig == 1:
	# 	# Draw a straight line in 90 deg
	# 		rospy.loginfo('moving')
	# 		joint_radians = goal[:7]  # angles for the arm
	# 		finger_radians = goal[7:] # angles for the gripper this code currently doesn't work

	# 		self.go_to_joint_state(joint_radians)
		
	# 	elif trig == 0:
	# 		self.go_to_goal(list(goal))

	def main(self):
		
		# Set up path here
		listener = tf.TransformListener()

		while True:
			try:
				translation, rotation = listener.lookupTransform('world', 'goal_tf', rospy.Time()) #('world', 'object_tf', rospy.Time())
				print(translation)
				break  # once the transform is obtained move on
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue  # if it fails try again

		# dir_path = os.path.dirname(os.path.realpath(__file__))
		# file = open(dir_path + "/object_position.csv", "a")
		# wr = csv.writer(file, dialect='excel')

		# wr.writerow((translation + rotation))
		# file.close()

		# print('translation: {0} \n rotation: {1}'.format(translation, rotation))
		# Pick planner 
		
		self.set_planner_type("RRT")
		# # self.set_planner_type("PRM*")

		# # self.go_to_goal()
		angles = tf.transformations.euler_from_quaternion(rotation)
		# print(angles)


		# # if self.env == 0:
		pose = [translation[0], translation[1], translation[2], angles[0], angles[1], angles[2]]
		# # pose = [translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], rotation[3]]
		# # print(pose)
		self.go_to_goal(pose)

	# 	# elif self.env == 1:
		# self.go_to_goal([0.0, 0.5, 0.125, 270, 0, 270])
			
	# 	# elif self.env == 2:
	# 	# 	self.go_to_goal([0.0, 0.75, 0.125, 270, 0, 295])


	# 	# Draw a straight line in 90 deg
	# 	# env 2
	# 	# self.go_to_goal([0.0, 0.5, 0.125, 270, 0, 270])

	# 	# env 3
	# 	# self.go_to_goal([0.0, 0.75, 0.125, 270, 0, 295])		




if __name__ == '__main__':
	
    world = sys.argv[-1]
    # print(" \n\n  env: {0} \n\n".format(world))

    MoveRobot(env=int(world))
	# rospy.spin()

