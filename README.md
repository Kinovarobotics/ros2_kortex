# ROS 2 Kortex
> Kinova® Kortex™ is the common software platform behind all of the products in the Gen3 family (Gen3 and Gen3 lite). It unifies the inner workings of the various robots and their related external tools, like the API. <br />
> https://www.kinovarobotics.com/product/gen3-robots

<center><img src="doc/resources/kinova-gen3-7dof-robotiq-2f-85.jpg" alt="Kinova Gen3 7DoF manipulator with Intel RealSense 3D Vision Module and Robotiq 2F-85 2 Finger 85mm Adaptive Gripper" style="width: 50%"/></center>

ROS2 Kortex is the official ROS2 package to interact with Kortex and its related products. It is built upon the Kortex API, documentation for which can be found in the [GitHub Kortex repository](https://github.com/Kinovarobotics/kortex).

## Build status

<table width="100%">
  <tr>
    <th>ROS 2 Distro</th>
    <th>Humble</th>
    <th>Iron</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/PickNikRobotics/ros2_kortex/tree/main">main</a></td>
    <td><a href="https://github.com/PickNikRobotics/ros2_kortex/tree/main">main</a></td>
    <td><a href="https://github.com/PickNikRobotics/ros2_kortex/tree/main">main</a></td>
  </tr>
  <tr>
    <th>Build Status</th>
    <td>
      <a href="https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/humble-binary-build.yml">
        <img src="https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/humble-binary-build.yml/badge.svg?event=push&branch=main" alt="Humble Binary Build"/>
      </a>
    </td>
    <td>
      <a href="https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/iron-binary-build.yml">
        <img src="https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/iron-binary-build.yml/badge.svg?event=push&branch=main" alt="Iron Binary Build"/>
      </a>
    </td>
    <td>
      <a href="https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/rolling-binary-build.yml">
        <img src="https://github.com/PickNikRobotics/ros2_kortex/actions/workflows/rolling-binary-build.yml/badge.svg?event=push&branch=main" alt="Rolling Binary Build"/>
      </a>
    </td>
  </tr>
  <tr>
    <th>Release Status</th>
    <td>coming soon<!-- TODO(moriarty) add build.ros2.org status badge once released --></td>
    <td>coming soon<!-- TODO(moriarty) add build.ros2.org status badge once released --></td>
    <td>coming soon<!-- TODO(moriarty) add build.ros2.org status badge once released --></td>
  </tr>
</table>


**Note:** There are several CI jobs checking against future upstream changes see [detailed build status](.github/workflows/README.md) for a full list of CI jobs and for more information.


## Getting started

1. Install ROS 2.

   If you're a developer, we recommend using Rolling to get the latest features and fixes.

   Rolling Release: [Install ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)<br/>
   Latest Release: [Install ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)</br>
   Stable LTS Release: [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

   After installing a version of ROS, source the setup.bash, which will set the `$ROS_DISTRO` environment variable.

2. Install this package from binary
   ```
   sudo apt install ros-$ROS_DISTRO-kortex-bringup
   ```

3. Optional: install MoveIt Configuration and Cyclone DDS

   If you have a 7dof arm:
   ```
   sudo apt install ros-$ROS_DISTRO-kinova-gen3-7dof-robotiq-2f-85-moveit-config
   ```
   If you have a 6dof arm:
   ```
   sudo apt install ros-$ROS_DISTRO-kinova-gen3-6dof-robotiq-2f-85-moveit-config
   ```
   If you plan to use MoveIt, it is recommended to install and use Cyclone DDS.
   ```
   sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

4. Go to Usage section

## Contributing to this repository or building from source

Note: It is recommended to use a released binary version of this package and apt install it.
If you want the latest version of this repository for testing latest fixes
check out testing with pre-released binaries: https://docs.ros.org/en/rolling/Installation/Testing.html

If the bug fix you need isn't in a released version or If you want to build this repository from source or contribute back to the repository read on.

1. Make sure that `colcon`, its extensions, and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

2. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros2_kortex_ws
   mkdir -p $COLCON_WS/src
   ```

3. Pull relevant packages, install dependencies, compile, and source the workspace by using:
   ```
   cd $COLCON_WS
   git clone https://github.com/PickNikRobotics/ros2_kortex.git src/ros2_kortex
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

4. To simulate the robot with ignition or gazebo make sure to pull and build additional packages:
   ```
   vcs import src --skip-existing --input src/ros2_kortex/simulation.repos
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Simulation Issues

Please note, at this time there are two known issues you with simulation

1. Gazebo + Mimic Joints for the Robotiq Gripper
2. Protobuf version mismatch

# Gazebo and Mimic Joints

A pull request has been made to gz_ros2_control which is how this repository was tested in simulation.
The pull request won't be merged as the fix should be done upstream in gz-sim.
Once a fix is available ros2_robotiq_gripper will be re-released and an update should fix any workarounds.

In the meantime if you need simulation checkout the upstream pull request link:

- Upstream Issue: https://github.com/gazebosim/gz-sim/issues/1684
- Upstream Pull Request: https://github.com/ros-controls/gz_ros2_control/pull/86
- Tracking Issue: https://github.com/PickNikRobotics/ros2_robotiq_gripper/issues/7

# Protobuf

Due to mismatched protobuf version that ships system and used by Gazebo simulator compiling twice may be required.
You will only run into this if you have certain other gazebo related code in your workspace while compiling this repository.
If errors are encounter you must clean your workspace and run colcon build in two steps:

1. build everything except kortex related packages
2. build the packages that where skipped

```
sudo apt install python3-colcon-clean # if you don't have colcon-clean installed already
colcon clean workspace -y
colcon build --packages-skip-regex '.*kortex.*' '.*gen3.*'
colcon build --packages-select-regex '.*kortex.*' '.*gen3.*'
```

## Usage
<!-- TODO(moriarty) this section is an information overload -->

To launch and view the robots URDF run:

```bash
ros2 launch kortex_description view_robot.launch.py
```

To simulate the 7 DoF Kinova Gen3 robot arm with mock hardware:

```bash
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
```

To generate motion plans and execute them with a simulated 7 DoF Kinova Gen3 arm with mock hardware:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
```

Alternatively, if you wish to use the Kinova Gen3's 6 DoF variant:

```bash
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true \
  dof:=6
```

and to bring up the Kinova Gen3 6 DoF with MoveIt:

```bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
```

Alternatively, if you wish to use the Kinova Gen3_lite's 6 DoF variant:

```bash
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true \
  robot_type:=gen3_lite \
  gripper:=gen3_lite_2f \
  dof:=6
```

To simulate the 7dof Kinova Gen3 robot with ignition run the following:

```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py \
  dof:=7 \
  use_sim_time:=true \
  launch_rviz:=false
```

and to use MoveIt to command the robot:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config sim.launch.py \
  use_sim_time:=true
```

To work with a physical robot and generate/execute paths with MoveIt run the following:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10
```
**Note: If you have reassigned your physical arm's robot IP address, then you will need to assign that ip address to `robot_ip`**

You can command the arm by publishing Joint Trajectory messages directly to the joint trajectory controller:
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
  ]
}" -1
```

You can also command the arm using Twist messages. Before doing so, you must active the `twist_controller` and deactivate the `joint_trajectory_controller`:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [twist_controller],
  deactivate_controllers: [joint_trajectory_controller],
  strictness: 1,
  activate_asap: true,
}"
```

Once the `twist_controller` is activated, You can publish Twist messages on the `/twist_controller/commands` topic to command the arm.

For example, you can jog the arm using [Teleop Twist Keyboard](https://index.ros.org/p/teleop_twist_keyboard/github-ros2-teleop_twist_keyboard/) with the following command:

**WARNING: you are responsible for collision checking, including self collisions when in this mode.**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/twist_controller/commands
```

If you wish to use the `joint_trajectory_controller` again to command the arm using JointTrajectory messages, run the following:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [joint_trajectory_controller],
  deactivate_controllers: [twist_controller],
  strictness: 1,
  activate_asap: true,
}"
```

The Robotiq 2f 85 Gripper will be available on the Action topic:

```bash
/robotiq_gripper_controller/gripper_cmd
```

You can test the gripper by calling the Action server with the following command and setting the desired `position` of thr gripper (`0.0=open`, `0.8=close`)

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"
```

## Contents

The following is a description of the packages included in this repository.

### kortex_description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots. For more details, please consult the [README](kortex_description/readme.md) from the package subdirectory.

### kortex_driver
This package implements a ROS node that allows communication between a node and a Kinova Gen3 or Gen3 lite robot. For more details, please consult the [README](kortex_driver/readme.md) from the package subdirectory.

### kortex_moveit_config
This metapackage contains the auto-generated MoveIt! files to use the Kinova Gen3 and Gen3 lite arms with the MoveIt! motion planning framework. For more details, please consult the [README](kortex_moveit_config/readme.md) from the package subdirectory.
