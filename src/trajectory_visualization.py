#!/usr/bin/env python
import sys
import rospy
from trajectory_msgs.msg import JointTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics


class trajectory_visualization:
	#=====================================
	#         Class constructor
	#  Initializes node and subscribers
	#=====================================
	def __init__(self):
		rospy.init_node('trajectory_visualization')

		self.loadParams()
		self.actual_path_msg = Path()
		self.previous_pose_position = Point()
		print('Default pose:')
		print(self.previous_pose_position.x)
		print(self.previous_pose_position.y)
		print(self.previous_pose_position.z)

		#Setup target trajectory path publisher
		self.target_path_pub = rospy.Publisher(self.target_path_topic, Path, queue_size=10, latch=True)
		#Setup actual trajectory path publisher
		self.actual_path_pub = rospy.Publisher(self.actual_path_topic, Path, queue_size=10, latch=True)
		#Setup subscribers to trajectory message
		self.trajectory_sub = rospy.Subscriber(self.trajectory_topic, JointTrajectory, self.trajectory_callback)
		#Setup subscriber to joint pose
		self.joint_states_sub = rospy.Subscriber(self.joint_states_topic, JointStates, self.joint_states_callback)

		# pykdl_utils setup
		self.kdl_kin = KDLKinematics(self.robot_urdf, self.base_link, self.end_link)
		self.joint_names = self.kdl_kin.get_joint_names()
		print("Robot model has "+str(len(self.joint_names))+" joints")

	#=====================================
	#       function for loading
	#    from ROS parameter server
	#=====================================
	def loadParams(self):
		#Load robot model
		rospy.loginfo('Looking for robot_description at '+ns+'robot_description')
		self.robot_urdf = URDF.from_xml_string(rospy.get_param('robot_description'))
		#Load trajectory topic name
		self.trajectory_topic = rospy.get_param('~trajectory_topic', '~trajectory')
		#Load joint state topic name
		self.joint_states_topic = rospy.get_param('~joint_states_topic', '/joint_states')
		#Load targeted path topic name
		self.target_path_topic = rospy.get_param('~target_path_topic', '~target_path')
		#Load actual path topic name
		self.actual_path_topic = +rospy.get_param('~actual_path_topic', '~actual_path')

		#Load maximum number of poses in actual path
		self.max_poses = rospy.get_param('~max_poses', 1000)
		#Load threshold for adding a pose to actual path
		self.threshold = rospy.get_param('~movement_threshold', 0.001)

		#Loading tracked joint name
		self.tracked_link = rospy.get_param('~tracked_link')
		#Loading base joint name
		self.base_link = rospy.get_param('~base_link', 'base_link')

	#=====================================
	#          Callback function 
	#     when receiving a trajectory
	#=====================================
	def trajectory_callback(self, traj_msg):
		target_path_msg = Path()
		target_path_msg.header.stamp = rospy.Time.now()
		target_path_msg.header.frame_id = self.base_link
		#For every points in trajectory message, run forward kinematic
		for point in traj_msg.points: 
			joint_angles = point.positions
			pose_stamped_msg = PoseStamped()
			transform_mat = self.kdl_kin.forward(joint_angles)
			#Get pose from transform matrix
			pose_stamped_msg.pose.position.x = transform_mat[0,3]
			pose_stamped_msg.pose.position.y = transform_mat[1,3]
			pose_stamped_msg.pose.position.z = transform_mat[2,3]
			target_path_msg.poses.append(pose_stamped_msg)
			rospy.logdebug('adding pose to path')

		#Publish path
		rospy.logdebug('publishing path')
		self.target_path_pub.publish(target_path_msg)

	#=====================================
	#          Callback function 
	#     when receiving joint states
	#=====================================
	def joint_states_callback(self, joint_states_msg):
		joint_angles = joint_angles_msg.position
		transform_mat = self.kdl_kin.forward(joint_angles)
		#If the joint has move more than a set threshold, add it to the path message and publish
		if (abs(self.previous_pose_position.x - transform_mat[0,3]) > self.threshold) 
		 & (abs(self.previous_pose_position.y - transform_mat[1,3]) > self.threshold)
		 & (abs(self.previous_pose_position.z - transform_mat[2,3]) > self.threshold):
			rospy.logdebug('Exceding threshold, adding pose to path')
			#Add current pose to path
			self.actual_path_msg.header.stamp = rospy.Time.now()
			self.actual_path_msg.header.frame_id = self.base_link
			pose_stamped_msg = PoseStamped()
			pose_stamped_msg.pose.position.x = transform_mat[0,3]
			pose_stamped_msg.pose.position.y = transform_mat[1,3]
			pose_stamped_msg.pose.position.z = transform_mat[2,3]

			#If max number of poses in path has not been reach, just add pose to message
			if len(self.actual_path_msg.poses) < self.max_poses:
				self.actual_path_msg.poses.append(pose_stamped_msg)
			#Else rotate the list to dismiss oldest value and add newer value at the end
			else :
				rospy.logdebug('Max number of poses reached, erasing oldest pose')
				self.actual_path_msg.poses = self.actual_path_msg.poses[1:]
				self.actual_path_msg.poses.append(pose_stamped_msg)

			self.previous_pose_position = pose_msg.pose.position
			self.actual_path_pub.publish(self.actual_path_msg)

#=====================================
#               Main
#=====================================
if __name__ == "__main__":
	trajectoryVisualization = trajectory_visualization()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"