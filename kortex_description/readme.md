<!--
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
* -->

# Kortex Description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots.

## Usage

To load the description of a robot, you simply have to load the **ARM.xacro** or the **ARM_GRIPPER.xacro** file, with **ARM** being your arm's name (gen3, gen3_lite), and if you have a gripper, **GRIPPER** being your gripper's name (robotiq_2f_85, gen3_lite_2f).

**Arguments**:
- **sim** : If this argument is true, the Gazebo-specific files will be loaded. The default value is **false **.

For example:

- To load the Gen3 description with a Robotiq 2-F 85 gripper for simulation, you would put in your launch file :
<code><param name="robot_description" command="$(find xacro)/xacro --inorder $(find kortex_description)/robots/gen3_robotiq_2f_85.xacro sim:=true"\/></code>

- To load the Gen3 lite description, you would put in your launch file :
<code><param name="robot_description" command="$(find xacro)/xacro --inorder $(find kortex_description)/robots/gen3_lite_gen3_lite_2f.xacro sim:=false"\/></code>

## Tool frame

The `tool_frame` link refers to the tool frame used by the arm when it reports end effector position feedback.

## Xacro Parameters for `load_robot` macro

Param | Description | Example Value |
:--- | :--------- | :------------------- |
`parent` | Parent link in the URDF the arm should be attached to | `parent="world"` |
`origin` | Origin of the robot relative to the specified parent link | `<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />`|
`prefix` | This is an optional prefix for all joint and link names in the kortex_description. It is used to allow differentiating between different arms in the same URDF. | `prefix=""`|
`arm` | Name of your robot arm model. | `arm="gen3"` or `arm="gen3_lite"` |
`gripper` | Name of gripper type | `gripper="robotiq_2f_85"` or `gripper="robotiq_2f_140"` |
`gripper_joint_name` | Name of gripper joint to be actuated | `gripper_joint_name="robotiq_85_left_knuckle_joint" or gripper_joint_name="finger_joint"` |
`dof` | Number of DOFs of your robot. | `dof="7"` or `dof="6"`|
`vision` | Boolean value to indicate if your arm has a Vision Module. This argument only affects the visual representation of the arm in RViz. | `vision="true"` or `vision="false"` |
`robot_ip` | The IP address of the robot you're connection to. | `robot_ip="192.168.1.10"` |
`username` | The username for the robot connection. | `username="admin"` |
`password` | The password for the robot connection. | `password="admin"` |
`port` | Port for Kortex hardware driver | `port="10000"` |
`port_realtime` | Realtime port for Kortex hardware driver | `port_realtime="10001"` |
`session_inactivity_timeout_ms` | The duration after which the robot will clean the client session if the client hangs up the connection brutally (should not happen with the ROS driver). | `session_inactivity_timeout_ms="60000"` |
`connection_inactivity_timeout_ms` | The duration after which a connection is destroyed by the robot if no communication is detected between the client and the robot. | `connection_inactivity_timeout_ms="2000"` |
`use_internal_bus_gripper_comm` | Boolean value to indicate if your gripper will be communicated with through the internal Kinova communication interface. The default value is **false**. | `use_internal_bus_gripper_comm="false"` |
`use_fake_hardware` | Boolean value to indicate whether or not the hardware components will be mocked. If true the hardware params will be ignored and the hardware components will be mocked. Defaults to **false** | `use_fake_hardware="false"` |
`fake_sensor_commands` | Boolean value. If set to true will create fake command interfaces for faking sensor measurements with an external command. Defaults to **false**. | `fake_sensor_commands="false"` |
`sim_gazebo` | Boolean value to indicate whether or not the gazebo_ros2_control/GazeboSystem plugin will be loaded. Defaults to **false**. | `sim_gazebo="false"` |
`sim_ignition` | Boolean value to indicate whether or not the ign_ros2_control/IgnitionSystem plugin will be loaded. Defaults to **false**. | `sim_ignition="false"` |
`sim_isaac` | Boolean value to indicate whether or not the topic_based_ros2_control/TopicBasedSystem plugin will be loaded and the "joint_commands_topic" and "joint_states_topic" parameters will be set to the `isaac_joint_commands` and `isaac_joint_states` values respectively. Defaults to **false**. | `sim_isaac="false"` |
`isaac_joint_commands` | Name of the joint commands topic to be used by Isaac Sim. Defaults to **/isaac_joint_commands**. | `isaac_joint_commands="/isaac_joint_commands"` |
`isaac_joint_states` | Name of the joint states topic to be used by Isaac Sim. Defaults to **/isaac_joint_states**. | `isaac_joint_states="/isaac_joint_states"` |
`use_external_cable` | Boolean value that sets joint limits to avoid wrapping of external cables if true. Defaults to **false**. | `use_external_cable="false"` |
`initial_positions` | Dictionary of initial joint positions. Defaults to `{joint_1: 0.0, joint_2: 0.0, joint_3: 0.0, joint_4: 0.0, joint_5: 0.0, joint_6: 0.0, joint_7: 0.0}`. | `initial_positions="${dict(joint_1=0.0,joint_2=0.0,joint_3=0.0,joint_4=0.0,joint_5=0.0,joint_6=0.0,joint_7=0.0)}"` |
`gripper_max_velocity` | Desired velocity in percentage (0.0-100.0%) with which the position will be set. Defaults to **100.0**. | `gripper_max_velocity="100.0"`|
`gripper_max_force` | Desired force in percentage (0.0-100.0%) with which the position will be set. Defaults to **100.0**. NOTE: deprecated according to the [Kortex repo](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/GripperCyclic/MotorCommand.md). | `gripper_max_force="100.0"`|
`gripper_include_ros2_control` | Boolean value that will launch a ros2_controller instance for the gripper if true. Defaults to **true**. Should be set to true if communicating directly to the gripper from the PC running ROS via USB or if running in simulation. | `gripper_include_ros2_control="true"` |
`gripper_com_port` | Specifies the USB port that the gripper is plugged in on. Defaults to **/dev/ttyUSB0**. | `gripper_com_port="/dev/ttyUSB0"` |
