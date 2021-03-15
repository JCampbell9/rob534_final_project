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
	def __init__(self):
		# Initialize moveit commander and ros node for moveit
		moveit_commander.roscpp_initialize(sys.argv)

		self.dir_path = os.path.dirname(os.path.realpath(__file__))


		# Initializing node
		rospy.init_node("move_kinova", anonymous=True)
		self.start_poses = []
		# Define robot using RobotCommander. Provided robot info such as
		# kinematic model and current joint state
		self.robot = moveit_commander.RobotCommander()
		# Setting the world 
		self.scene = moveit_commander.PlanningSceneInterface()
		# self.env = env
		self.object = rospy.get_param('test_object')
		self.env = rospy.get_param('test_env')
		self.pose_number = rospy.get_param('goal_pose')
		self.ready = rospy.get_param('ready_trig')
		rospy.sleep(2)


		self.prm_results = []
		self.rrt_results = []
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
		# self.read_csv()
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
		result = self.move_group.go(wait=True)
		self.move_group.execute(self.plan, wait=True)

		self.move_group.stop()
		self.move_group.clear_pose_targets()
		rospy.sleep(2)
		return(result)

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
		result = self.move_group.go(wait=True)
		self.move_group.stop()

		self.move_group.clear_pose_targets()
		rospy.sleep(2)
		return result
		
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

	def read_csv(self):
		with open(self.dir_path + '/start_poses_1.csv', 'r') as csvfile:
			ofile = csv.reader(csvfile, delimiter=',')
			# next(ofile)

			for row in ofile:
				self.start_poses.append([float(i) for i in row])

	def build_env(self):

		obj_pose = PoseStamped()
		obj_pose.header.frame_id = self.robot.get_planning_frame()
		obj_pose.pose.position.x = 0.0
		obj_pose.pose.position.z = 0.125
		

		if self.env == 0:
			obj_pose.pose.position.y = 0.25

		elif self.env == 1:
			obj_pose.pose.position.y = 0.5

			wall_pose = PoseStamped()
			wall_pose.header.frame_id = self.robot.get_planning_frame()
			wall_pose.pose.position.x = 0.3
			wall_pose.pose.position.y = 0.25
			wall_pose.pose.position.z = 0.25
			self.scene.add_box('wall', wall_pose, (1, 0.025, .5))

		elif self.env == 2:
			obj_pose.pose.position.y = 0.75
	
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
			wallT_pose.pose.position.z = 1.0
			self.scene.add_box('wallT', wallT_pose, (0.5, 0.025, 1))

		if self.object == 'Cyl':
			self.scene.add_cylinder(self.object, obj_pose, .25, 0.025)
		
		elif self.object == 'Cube':
			self.scene.add_box(self.object, obj_pose, (.05, .05, .25))

	def teardown_env(self):
		if self.env == 0:
			self.scene.remove_world_object(self.object)
		
		elif self.env == 1:
			self.scene.remove_world_object(self.object)
			self.scene.remove_world_object('wall')
		
		elif self.env == 2:
			self.scene.remove_world_object(self.object)
			self.scene.remove_world_object('wallL')
			self.scene.remove_world_object('wallR')
			self.scene.remove_world_object('WallT')

	def write_csv(self):
		dir_path = os.path.dirname(os.path.realpath(__file__))
		file = open(dir_path + "/{0}_results_redo.csv".format(self.object), "w")
		wr = csv.writer(file, dialect='excel')
		wr.writerow(['obj', 'planner', 'starting_pose', 'goal_pose', 'env', 'result', 'run_time', 'started from start_pose'])  # [self.object, planner, i, self.pose_number, self.env, result, run_time]
		for i in self.rrt_results:
			wr.writerow(i)
		
		for j in self.prm_results:
			wr.writerow(j)
		
		file.close()


	def main(self):

		self.read_csv()

		table_pose = PoseStamped()
		table_pose.header.frame_id = self.robot.get_planning_frame()
		table_pose.pose.position.x = 0.
		table_pose.pose.position.y = 0.
		table_pose.pose.position.z = -0.0125
		self.scene.add_box('table', table_pose, (1, 3, 0.025))
		self.build_env()



		# Set up path here
		listener = tf.TransformListener()


		# print('translation: {0} \n rotation: {1}'.format(translation, rotation))
		# Pick planner 
		home = [270 * pi / 180, 163 * pi / 180, 0 * pi / 180, 43 * pi / 180, 265 * pi / 180, 257 * pi / 180, 280 * pi / 180]

		# dir_path = os.path.dirname(os.path.realpath(__file__))
		# file = open(dir_path + "/{0}_results.csv".format(self.object), "w")
		# wr = csv.writer(file, dialect='excel')
		
		for _ in range(3-self.env):
			# rospy.logerr('in for loop')

			while True:
				self.ready = rospy.get_param('ready_trig')

				if self.ready != 0:
					time.sleep(.5)
					# rospy.logerr('in if statement')
					while True:
						try:
							translation, rotation = listener.lookupTransform('world', 'goal_tf', rospy.Time()) #('world', 'object_tf', rospy.Time())
							print(translation)
							break  # once the transform is obtained move on
						except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
							continue  # if it fails try again
					angles = tf.transformations.euler_from_quaternion(rotation)
					pose = [translation[0], translation[1], translation[2], angles[0], angles[1], angles[2]]
					made_it_to_start = True
					for i, start_ang in enumerate(self.start_poses):
						self.pose_number = rospy.get_param('goal_pose')
						rospy.logerr('in the start loop {}'.format(start_ang))
						planner = "RRT"
						self.set_planner_type(planner)
						made_it_to_start = self.go_to_joint_state(start_ang)
						
						timer_start = time.clock()
						result = self.go_to_goal(pose)
						print("ran{}".format(self.pose_number))
						run_time = time.clock() - timer_start

						self.rrt_results.append([self.object, planner, i, self.pose_number, self.env, result, run_time, made_it_to_start])
						# wr.writerow([self.object, planner, i, self.pose_number, self.env, result, run_time]) # [object, planner, start_pose, pose, env, fail/success, time]

						time.sleep(.01)
						made_it_to_start = self.go_to_joint_state(start_ang)

						planner = "PRM*"
						self.set_planner_type(planner)

						timer_start = time.clock()
						result = self.go_to_goal(pose)

						run_time = time.clock() - timer_start

						self.prm_results.append([self.object, planner, i, self.pose_number, self.env, result, run_time, made_it_to_start])
						# wr.writerow([self.object, planner, i, self.pose_number, self.env, result, run_time]) # [object, planner, pose, env, fail/success, time]

						time.sleep(.01)
						# if len(self.start_poses) == 1:
						# 	made_it_to_start = self.go_to_joint_state(start_ang)
						# elif start_ang == self.start_poses[-1]:
						# 	made_it_to_start = self.go_to_joint_state(self.start_poses[0])
						# else:
						# 	made_it_to_start = self.go_to_joint_state(self.start_poses[i+1])

					if rospy.get_param('ready_trig') == 3:
						rospy.set_param('goal_pose', 0)
						break
					else:
						self.pose_number += 1
						rospy.set_param('goal_pose', self.pose_number)
					
					rospy.set_param('ready_trig', 4)

				if rospy.get_param('ready_trig') == 3:
					break
			rospy.set_param('goal_pose', 0)
			self.teardown_env()
			self.env += 1
			self.build_env()
			rospy.set_param('test_env', self.env)
			rospy.set_param('ready_trig', 4)

		# file.close()

		self.write_csv()




if __name__ == '__main__':
	
    # world = sys.argv[-1]
    # print(" \n\n  env: {0} \n\n".format(world))

    MoveRobot()
	# rospy.spin()

