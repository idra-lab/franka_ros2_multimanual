# IDRA Franka Launch

This repository serves as implementation of a bimanual manipulator approach using two Franka Research 3 robots, also creating a custom hardware interface for a better precision control of the robots.

## Requirements

- Having ROS2 humble set up and working on a Ubuntu Real-Time kernel.
- Having at least two FR3 robots with FCI activated.

## Installation

This package depends on [franka_ros2](https://github.com/frankarobotics/franka_ros2) package that must be installed in the system or in the same workspace of this package.

To compile the package:

```
colcon build --symlink-install
```

Remember to source the workspace:

```
source install/setup.bash
```

## Usage

There is a basic launcher to launch the bimanual implementation of the two robots, either in simulated environment (Gazebo) or controlling the real robots. 

For running the simulated environment:

```
ros2 launch idra_franka_launch bimanual.launch.py use_gazebo:=true
```

For connection with the real robots:

```
ros2 launch idra_franka_launch bimanual.launch.py left_ip:=<left_ip> right_ip:=<right_ip>
```

## Test

The default launch file loads all the controllers that are defined inside `franka_mm_control/config/basic_controller.yaml`, activating the joint velocity controller by default. To change the default behavior change the launch file or use `rqt_controller_manager` package.

Inside `franka_mm_control/scripts` there are some basic scripts for testing the various controller types implemented inside te robot hardware interface.
To launch velocity control script:

```
ros2 run franka_mm_control velocity_test.py
```