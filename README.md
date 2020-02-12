# ROS
# fk_trajectory_visualization

## 1. Package Summary
A small tool for ROS, to display a robot planned and actual link movement in RViz using forward kinematics (and path messages). Thanks to [Joseph Coombe](https://answers.ros.org/question/281272/get-forward-kinematics-wo-tf-service-call-from-urdf-joint-angles-kinetic-python/) for giving a way to get forward kinematics from URDF and joint angles.

<p align="center">
  <img src="https://github.com/KTH-SML/kth_hebi_arm/blob/master/kth_hebi_arm_visualization/resource/documentation/pictures/pic_2.png" width="500"/>
</p>

## 2. Requirements
* ROS messages packages: trajectory_msgs, nav_msgs, sensor_msgs and geometry_msgs
* [urdfdom_py](https://wiki.ros.org/urdfdom_py)
* [pykdl_utils](http://wiki.ros.org/pykdl_utils): to use with more recent ROS distro than hydro, copy the repo from [here](https://github.com/gt-ros-pkg/hrl-kdl) to your workspace

## 3. Usage
The node requires the URDF robot model to be loaded in the parameter server. To run the visualization node, set the appropriate parameters properly, (see below) and launch with the following command:
```
rosrun fk_trajectory_visualization trajectory_visualization.py
```
Broadcasting a JointTrajectory message on the topic '~trajectory' will output a Path message on the topic '~target_path' for the specified tracked link. This path represent the planned trajectory of the tracked link.

Broadcasting JointStates messages in real-time on 'joint_state' will output another Path message on the topic '~actual_path' for the specified tracked link. This path represent the actual trajectory followed by the tracked link.

The paths can be visualized on RViz. Managing the buffer length of the path objects on RViz can allow the display of multiple paths.

### 3.1 Subscribed topics:
  * ~trajectory (*trajectory_msgs/JointTrajectory*): planned trajectory
  * joint_states (*sensor_msgs/JointStates*): Joint states for the loaded URDF
### 3.2 Published topics:
  * ~target_path (*nav_msgs/Path*): planned joint path
  * ~actual_path (*nav_msgs/Path*): actual joint path
### 3.3 Parameters:
  * robot_description (String, default: None): Contents of the URDF file for the robot
  * ~tracked_link (String, default: 'end_effector'): Name of the link to track
  * ~max_poses (int, default: 1000): Maximum number of poses in the actual followed path.
  * ~movement_threshold (double, default: 0.001): Threshold for adding a new pose in the actual followed path (in meters).
