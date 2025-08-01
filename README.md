# IDRA Franka Launch

This repository serves as implementation of a bimanual manipulator approach using two Franka Research 3 robots, also creating a custom hardware interface for a better precision control of the robots.

## Video examples
To show the capabilities of a bimanual approach of the robots, two examples are proposed.

The first one shows the two robots in a disassembly task of electronic components.

<p align="center">
  <img src="webdocs/images/Bimanual-NIST.mp4" width="620" alt="NIST Overview" />
</p>

The second example show how a robot can be programmed to move a tray with both arms in synchrony.

<p align="center">
    <video width="620"autoplay muted>
        <source src="webdocs/images/Bimanual-NIST.mp4" type="video/mp4" alt="Tray overview">
    Video tag not supported
    </video>
</p>

## Requirements

- Having ROS2 humble set up and working on a Ubuntu Real-Time kernel.
- Having at least two FR3 robots with FCI activated.

## Installation

This package depends on [franka_ros2](https://github.com/frankarobotics/franka_ros2) package that must be installed in the system or in the same workspace of this package.

To compile the package:

```shell
colcon build --symlink-install
```

Remember to source the workspace:

```shell
source install/setup.bash
```

## Usage

There is a basic launcher to launch the bimanual implementation of the two robots, either in simulated environment (Gazebo) or controlling the real robots. 

For running the simulated environment:

```shell
ros2 launch idra_franka_launch bimanual.launch.py use_gazebo:=true
```

For connection with the real robots:

```shell
ros2 launch idra_franka_launch bimanual.launch.py left_ip:=<left_ip> right_ip:=<right_ip>
```

## Test

The default launch file loads all the controllers that are defined inside `franka_mm_control/config/basic_controller.yaml`, activating the cartesian impedance controller by default. To change the default behavior change the launch file or use `rqt_controller_manager` package.

Inside `franka_mm_control/scripts` there are some basic scripts for testing the various controller types implemented inside te robot hardware interface.
To launch impedance control script:

```shell
ros2 run franka_mm_control cart_impedance_test.py
```