
Collaborative Manipulation Project - Trajectory Adaptation

Abhinav Jain (jain@gatech.edu), Daphne Chen, Dhruva Bansal, Sam Scheele, Mayank Kishore, Hritik Sapra, David Kent, Harish Ravichandar and Sonia Chernova

# Repositories

## co_manipulation

Repository of ROS packages. Contains the following packages

1. human_traj_pred -- package for ROS pipeline setup for taking in output from the Azure kinect (MarkerArray messages) and then using UOLA to get the mean predictions and variances. 
1. conimbus_bringup, conimbus_description, conimbus_moveit_config -- packages for launching drivers for J2S7S300 Kinova Jaco2 7DOF Robot
2. co_manipulation_demo, co_task_optimizer, co_traj_optimizer -- packages for September 2019 demo
3. human_traj_display -- package for visualizing human trajectory in RVIZ

### human_traj_pred

The packages includes a copy of the [unsupervised_online_reaching_prediction repository](https://github.com/WPI-ARC/unsupervised_online_reaching_prediction) (UOLA), which is used for human pose prediction.

The `phantom_human_traj_stream.py` file is responsible for reading a csv with some pre-recorded trajectory, convert its data into MarkerArray messages and then publish that information on the human_traj_stream topic. It was created to emulate output from the Azure Kinect so that when we use this pipeline with the real data, we don't face any problems. 

The `predict_human_traj.py` file is the core prediction file. It listens to the human_traj_stream topic for MarkerArray messages, parses it and then uses UOLA (specfically `UOLA_predict.m`) to get the prediction means and variances. 

Finally, `parsePrediction.py` file parses the output from the predict_human_traj.py and puts it in the format required by trajopt. 

### conimbus

The launch file `conimbus_bringup.launch` in the conimbus_bringup package launches the full robot with sensing. Parameters inside the launch file can be set to modify the sensors. Trajectory controller and MoveIt! can be enabled/disabled as well.

## trajopt

The trajopt repo is a fork of [trajopt](https://github.com/joschu/trajopt) from Pieter Abbeel's lab based on [this paper](http://joschu.net/docs/trajopt-paper.pdf). For this project, we have added implementations of our costs in C++ in trajopt/src/trajopt/kinematic_terms.cpp. Cost containers are added to trajopt/src/trajopt/problem_description.cpp. Only these files and corresponding header files have been modified in C++.

The trajopt/comanipulationpy directory contains python interface for the project. The following files are used:

1. comanipulationpy.py -- all functions used to add trajopt costs to a trajopt problem. Some helper functions to update human pose trajectories
2. arm_control.py -- functions used to interface with ROS control for robot arm
3. testing_framework.py -- the full testing framework that contains all testing functions inside a class. 
4. testing.py -- file used for creating an instance of testing_framework and running tests. This file is used to run the code.
5. plots.py and trajectoryViz.py -- used for plotting human and robot trajectories

Other python files are obsolete and not used any longer.

# Description

## Full pipeline

- Planning request is passed
- Comanipulationpy node waits for updated prediction
    - Pose prediction node is constantly updating human pose prediction from camera observations
    - Passes updated prediction as a string
- Comanipulationpy node updates collision object from camera (see below)
- New trajopt optimization problem is setup with human pose prediction, robot state and goal joint position
- Problem is optimized
- Optimized trajectory is executed using trajectory controller

## Collision objects from camera:

- MoveGroup node has to be running (either through conimbus_bringup or from movegroup.launch from conimbus_moveit_config).
- Octomap from moveit’s planning scene is used
- Octomap is published in binary
- Converted using octomap ros library and republished using custom message format
- Each voxel in octomap is added individually as a collision object.
- Size of each voxel is 0.025 meters

## Robot model inside OpenRAVE/trajopt

OpenRAVE uses collada models for robot description whereas ROS uses URDF. We use [collada_urdf](http://wiki.ros.org/collada_urdf) to convert ROS robot urdf files to collada dae files. Note that some modifications have to made to the fies to made the compatible with trajopt. See jaco-test.dae and iiwa.dae files in trajopt/data.

# Installation

## Dependencies

1. ROS Melodic (see [here](http://wiki.ros.org/melodic/Installation/Ubuntu))
2. OpenRAVE (run `install_all.sh` in `/trajopt/openrave_installation/` or see [here](https://github.com/crigroup/openrave-installation))
3. MATLAB and matlab.engine (see [here](https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html))
4. Kinect Azure Libraries (see [here](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download))
    1. libk4abt1.0-dev
    2. libk4a1.3-dev
5. ROS Packages
    1. kinova-ros ([GT-RAIL github](https://github.com/GT-RAIL/kinova-ros)) - `melodic-7dof` branch
    2. robotiq_85_gripper ([GT-RAIL github](https://github.com/GT-RAIL/robotiq_85_gripper)) - `melodic-devel` branch
    3. robotiq_85_gripper_actions ([GT-RAIL github](https://github.com/GT-RAIL/robotiq_85_gripper_actions)) - `melodic-7dof` branch
    4. rail_manipulation_msgs ([GT-RAIL github](https://github.com/GT-RAIL/rail_manipulation_msgs)) - `melodic-7dof` branch
    5. Azure_Kinect_ROS_Driver ([microsoft github](https://github.com/microsoft/Azure_Kinect_ROS_Driver))
    6. Iiwa_stack ([Abhinav Jain](https://github.com/abhinavj30/iiwa_stack)) - `master` branch

## Building

The co_manipulation repository goes into a catkin workspace along with the ROS dependencies (see [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for instructions).

The trajopt repository is built using cmake. Follow build instructions here. Note that OpenRAVE, OpenSceneGraph and Boost are installed by the openrave installation scripts above. Be sure to update `PYTHONPATH` after building trajopt.

Run the following commands to build trajopt:

```
mkdir build && cd build
cmake ..
make -j `nproc`
```

Note that trajopt unittests may fail due to unstable OpenRAVE/OpenRAVEpy. OpenRAVE often crashes on exit.

Verify installation: 

- Build the catkin workspace using `catkin build`
- Start ROS using `roscore`
- Run trajopt using `python2 testing.py`
- Run human motion predictor using `rosrun human_traj_pred predict_human_traj.py`
- Run phantom trajectory stream using `rosrun human_traj_pred phantom_human_traj_stream.py`

# Running

- First build the catkin workspace using `catkin build`.
- Start a ROS instance using `roscore`.
- If a Jaco is being used, run `roslaunch conimbus_bringup conimbus_bringup.launch` to start the robot.
- Use the command `rosrun human_traj_pred filename.py` to run the human trajectory prediction files. These files have to be run from catkin_ws/src/co-manipulation/human_traj_pred/src/. You must also make all python files executable. This can be done using the command chmod +x filename.py. 
- Trajopt is run through `testing.py` in trajopt/comanipulationpy.


# TODO

## human_traj_pred

- Update human_traj_pred package to be independent of directory run from. Consider setting up launch files for this.

## trajopt

- Improve FK speed. Currently uses OpenRAVE for FK, which can be very slow when several FK calls are made per cost computation. Consider using simpler calculations (maybe based on DH parameters) for calculating FK or a lighter external library for the same.
- New distance cost. Current distance cost calculates human joint - robot joint pairwise values. Try vectories human-robot joint distances to calculate the full cost.

## comanipulationpy

- Currently parsing in octomap from MoveIt!. Update to use [octomap ros package](http://wiki.ros.org/octomap) to directly take point cloud from camera and generate parsable octomap.
