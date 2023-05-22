# ros2_kortex
ROS2 Kortex is the official ROS2 package to interact with Kortex and its related products. It is built upon the Kortex API, documentation for which can be found in the [GitHub Kortex repository](https://github.com/Kinovarobotics/kortex).

**Warning** Currently the only packages ported to ROS2 are 'kortex_description`, 'kortex2_driver`, 'kortex2_bringup` and 'gen3_move_it_config`

## Build status

ROS2 Distro | Branch | Build status
:---------: | :----: | :----------:
**Rolling** | [`main`](https://github.com/PickNikRobotics/ros2_kortex/tree/main) | [![Rolling Binary Build](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/rolling-binary-build.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/rolling-binary-build.yml?branch=main) <br /> [![Rolling Semi-Binary Build](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/rolling-semi-binary-build.yml?branch=main) <br /> [![Rolling Source Build](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/rolling-source-build.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/rolling-source-build.yml?branch=main)
**Humble** | [`main`](https://github.com/PickNikRobotics/ros2_kortex/tree/main) | [![Humble Binary Build](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/humble-binary-build.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/humble-binary-build.yml?branch=main) <br /> [![Humble Source Build](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/humble-source-build.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/humble-source-build.yml?branch=main)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


## Getting started

1. [Install ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)
2. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

3. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros2_kortex_ws
   mkdir -p $COLCON_WS/src
   ```

4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
   ```
   cd $COLCON_WS
   git clone https://github.com/PickNikRobotics/ros2_kortex.git src/ros2_kortex
   vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.repos
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

5. To simulate the robot with ignition or gazebo make sure to pull and build additional packages:
   ```
   vcs import src --skip-existing --input src/ros2_kortex/simulation.repos
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Usage
To launch and view the robots URDF run:

        ros2 launch kortex_description view_robot.launch.py

To simulate the 7 DoF Kinova Gen3 robot arm with mock hardware:

        ros2 launch kortex2_bringup gen3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true

To generate motion plans and execute them with a simulated 7 DoF Kinova Gen3 arm with mock hardware:

        ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true

To simulate the robot with gazebo run the following:
   ```
   ros2 launch kortex2_bringup kortex_sim_control.launch.py sim_gazebo:=true sim_ignition:=false
   ```

To simulate the robot with ignition run the following:
   ```
   ros2 launch kortex2_bringup kortex_sim_control.launch.py
   ```

To work with a physical robot and generate/execute paths with MoveIt run the following:

        ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.1.10

**Note: If you have reassigned your physical arm's robot IP address, then you will need to assign that ip address to `robot_ip`**

The Robotiq 2f 85 Gripper will be available on the Action topic:

        /robotiq_gripper_controller/gripper_cmd

You can test the gripper by calling the Action server with the following command and setting the desired `position` of thr gripper (`0.0=open`, `0.8=close`)

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"
```


## Download links

You can refer to the [Kortex repository "Download links" section](https://github.com/Kinovarobotics/kortex#download-links) to download the firmware package and the release notes.

### Accessing the color and depth streams

To access the color and depth streams, you will need to clone and follow the instructions to install the [ros_kortex_vision repository ](https://github.com/Kinovarobotics/ros_kortex_vision).
## Installation

### Setup

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

This package has been tested under ROS Kinetic (Ubuntu 16.04) and ROS Melodic (Ubuntu 18.04).
You can find the instructions to install ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and ROS Melodic [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

[Google Protocol Buffers](https://developers.google.com/protocol-buffers/) is used by Kinova to define the Kortex APIs and to automatically generate ROS messages, services and C++ classes from the Kortex API `.proto` files. The installation of Google Protocol Buffers is required by developers implementing new APIs with the robot. However, since we already provide all the necessary generated files on GitHub, this is not required for most end users of the robot.

### Build

These are the instructions to run in a terminal to create the workspace, clone the `ros_kortex` repository and install the necessary ROS dependencies:

        sudo apt install python3 python3-pip
        sudo python3 -m pip install conan
        conan config set general.revisions_enabled=1
        conan profile new default --detect > /dev/null
        conan profile update settings.compiler.libcxx=libstdc++11 default
        mkdir -p catkin_workspace/src
        cd catkin_workspace/src
        git clone https://github.com/Kinovarobotics/ros_kortex.git
        cd ../
        rosdep install --from-paths src --ignore-src -y

Then, to build and source the workspace:

        catkin_make
        source devel/setup.bash

You can also build against one of the ARMv8 builds of the Kortex API with Conan if you specify the `CONAN_TARGET_PLATFORM` CMake argument when using `catkin_make`. The following platforms are supported:

- Artik 710:

        catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=artik710
        source devel/setup.bash

- IMX6:

        catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=imx6
        source devel/setup.bash

- NVidia Jetson:

        catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=jetson
        source devel/setup.bash

As you see, there are instructions to install the Conan package manager. You can learn more about why we use Conan or how to simply download the API and link against it [in this specific section of the kortex_driver readme](kortex_driver/readme.md#conan). You can also decide

### pre-commit Formatting Checks

In this repository we have a pre-commit check that runs in CI. You can use this locally and set it up to run automatically before you commit something. To install, use pip:

    pip3 install pre-commit

To run over all the files in the repo manually:

    pre-commit run -a

To run pre-commit automatically before committing in a local repo, install git hooks:

    pre-commit install

## Contents

The following is a description of the packages included in this repository.

### kortex_control
This package implements the simulation controllers that control the arm in Gazebo. For more details, please consult the [README](kortex_control/readme.md) from the package subdirectory.

**Note** The `ros_control` controllers for the real arm are not yet implemented and will be in a future release of `ros_kortex`.

### kortex_description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots. For more details, please consult the [README](kortex_description/readme.md) from the package subdirectory.

### kortex_driver
This package implements a ROS node that allows communication between a node and a Kinova Gen3 or Gen3 lite robot. For more details, please consult the [README](kortex_driver/readme.md) from the package subdirectory.

### kortex_examples
This package holds all the examples needed to understand the basics of `ros_kortex`. Most of the examples are written in both C++ and Python. Only the MoveIt! example is available exclusively in Python for now.
A more detailed [description](kortex_examples/readme.md) can be found in the package subdirectory.

### kortex_gazebo
This package contains files to simulate the Kinova Gen3 and Gen3 lite robots in Gazebo. For more details, please consult the [README](kortex_gazebo/readme.md) from the package subdirectory.

### kortex_move_it_config
This metapackage contains the auto-generated MoveIt! files to use the Kinova Gen3 and Gen3 lite arms with the MoveIt! motion planning framework. For more details, please consult the [README](kortex_move_it_config/readme.md) from the package subdirectory.

### kortex2_controllers
Custom `ros2_control` based controllers for kinova arms. More information can be found [here](kortex2_controllers/README.md).

### third_party
This folder contains the third-party packages we use with the ROS Kortex packages. Currently, it consists of two packages used for the simulation of the Robotiq Gripper in Gazebo. We use [gazebo-pkgs](third_party/gazebo-pkgs/README.md) for grasping support in Gazebo and [roboticsgroup_gazebo_plugins](third_party/roboticsgroup_gazebo_plugins/README.md) to mimic joint support in Gazebo.
