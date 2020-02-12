#!/usr/bin/env python
import sys
import rospy
from trajectory_msgs.msg import JointTrajectory
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
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

		#Setup target trajectory path publisher
		self.target_path_pub = rospy.Publisher('~target_path', Path, queue_size=10, latch=True)
		#Setup actual trajectory path publisher
		self.actual_path_pub = rospy.Publisher('~actual_path', Path, queue_size=10, latch=True)
		#Setup subscribers to trajectory message
		self.trajectory_sub = rospy.Subscriber('~trajectory', JointTrajectory, self.trajectory_callback)
		#Setup subscriber to joint pose
		self.joint_states_sub = rospy.Subscriber('joint_states', JointState, self.joint_states_callback)

		# pykdl_utils setup
		self.kdl_kin = KDLKinematics(self.robot_urdf, self.base_link, self.tracked_link)
		self.joint_names = self.kdl_kin.get_joint_names()
		self.number_of_joints = len(self.joint_names)
		rospy.logdebug("Trajectory visualization: Robot model has "+str(self.number_of_joints)+" joints")
		rospy.logdebug("Trajectory visualization: Joints are: "+str(self.joint_names))
		rospy.loginfo("Trajectory visualization ready for robot: "+self.robot_urdf.name)

	#=====================================
	#       function for loading
	#    from ROS parameter server
	#=====================================
	def loadParams(self):
		#Load robot model
		self.robot_urdf = URDF.from_xml_string(rospy.get_param('robot_description'))

		#Load maximum number of poses in actual path
		self.max_poses = rospy.get_param('~max_poses', 1000)
		#Load threshold for adding a pose to actual path
		self.threshold = rospy.get_param('~movement_threshold', 0.001)

		#Loading tracked joint name
		self.tracked_link = rospy.get_param('~tracked_link', 'gripper_closed_endpoint')
		#Loading base joint name
		self.base_link = rospy.get_param('~base_link', 'base')

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
		rospy.logdebug("received joint message with joint angles: "+str(joint_states_msg.position))
		#Process message only enough joint values inside
		if (len(joint_states_msg.position)) >= self.number_of_joints:
			joint_angles_list = list(joint_states_msg.position)
			joint_names_list = list(joint_states_msg.name)
			#Remap joint values in needed
			joint_angles = [0] * self.number_of_joints
			try:
				for i in range(self.number_of_joints):
					found = False
					for j in range(len(joint_names_list)):
						#If names fit, put in joint_angles and delete from list
						if self.joint_names[i] == joint_names_list[j]:
							joint_angles[i] = joint_angles_list[j]
							del joint_angles_list[j]
							del joint_names_list[j]
							found = True
							break
					if not found:
					 	raise Exception("No joint value found for joint "+self.joint_names[i]+", ignoring message")

				transform_mat = self.kdl_kin.forward(joint_angles)
				#If the pose has move more than a set threshold, add it to the path message and publish
				if ((abs(self.previous_pose_position.x - transform_mat[0,3]) > self.threshold)
				 or (abs(self.previous_pose_position.y - transform_mat[1,3]) > self.threshold)
				 or (abs(self.previous_pose_position.z - transform_mat[2,3]) > self.threshold)):
					rospy.loginfo('Exceding threshold, adding pose to path')
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

					self.previous_pose_position = pose_stamped_msg.pose.position
					self.actual_path_pub.publish(self.actual_path_msg)

			except ValueError as e:
				rospy.loginfo("Error: "+e)
				pass	
 
#=====================================
#               Main
#=====================================
if __name__ == "__main__":
	trajectoryVisualization = trajectory_visualization()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"